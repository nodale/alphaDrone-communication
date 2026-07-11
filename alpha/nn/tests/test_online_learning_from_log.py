"""
Simulate online learning from a pre-converted .pt flight log tensor.
No SHM, no coordinator, no running services required.

Usage (from alpha/):
    python nn/tests/test_online_learning_from_log.py

Paths are read from nn/config/online_learner.yaml:
  model_run_dir  — Hydra run dir with .hydra/config.yaml + *.pth checkpoint
  flight_log_pt  — (T, 23) float32 tensor produced by Thesis-NN/data/converter.py
"""

import sys
import tempfile
from pathlib import Path

import numpy as np
import torch
import yaml
import zarr
from torch.utils.data import DataLoader

# ── Bootstrap: load config early to resolve Thesis-NN path portably ──────────
_CONFIG_PATH = Path(__file__).parent.parent / "config" / "online_learner.yaml"

def _load_cfg() -> dict:
    with open(_CONFIG_PATH) as f:
        return yaml.safe_load(f)

# model_run_dir is <thesis_nn>/latest_model/<date>/<run> — 3 levels deep
_THESIS_NN = Path(_load_cfg()["model_run_dir"]).parents[2]
sys.path.insert(0, str(_THESIS_NN))

from data.dataset import FlightLog, QuickDataset2
from data.denormaliser import denormalise
from deploy.estimator import (
    QuickEstimator,
    _load_hydra_cfg, _find_checkpoint, _build_model, _strip_compile_prefix,
)
from evaluate.metrics import metric_ate

_N_DIM = 23


def _save_as_episodes(tensor: torch.Tensor, zarr_path: Path, episode_len: int):
    """Chop tensor into episodes in the CollectorThread zarr format."""
    data = tensor.cpu().numpy()
    T    = (len(data) // episode_len) * episode_len
    eps  = data[:T].reshape(-1, episode_len, _N_DIM).astype(np.float32)
    store = zarr.open(str(zarr_path), mode="w")
    store.create_dataset("episodes", data=eps, chunks=(1, episode_len, _N_DIM), shape=(1, episode_len, _N_DIM))
    return eps.shape[0]


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_data_conversion():
    cfg    = _load_cfg()
    tensor = torch.load(cfg["flight_log_pt"])
    assert tensor.ndim == 2 and tensor.shape[1] == _N_DIM, f"bad shape {tensor.shape}"
    print(f"[OK] data loaded: {tensor.shape}")


def test_inference_from_log():
    """Simulate deployment: feed .pt log observations tick-by-tick through
    QuickEstimator.step(), mirroring the per-control-tick call in DroneEnv.
    The estimator overwrites pos/vel channels with its own running prediction
    (not ground truth), exactly as in real flight.
    """
    cfg      = _load_cfg()
    run      = Path(cfg["model_run_dir"])
    device   = torch.device(cfg.get("device", "cpu"))
    mcfg     = _load_hydra_cfg(run)
    pred_dim = mcfg["models"]["out_dim"]

    estimator = QuickEstimator.from_run(run_dir=run, num_envs=1, device=device, pred_dim=pred_dim)

    tensor = torch.load(cfg["flight_log_pt"])  # (T, 23) normalised
    T = len(tensor)

    predictions, ground_truth = [], []
    for t in range(T):
        obs = tensor[t].unsqueeze(0).to(device)  # (1, 23)
        est = estimator.step(obs)                # None until history fills
        if est is not None:
            predictions.append(est.cpu())                                       # denormalised
            ground_truth.append(denormalise(tensor[t, :pred_dim].unsqueeze(0)))

    assert predictions, "no predictions produced — log too short to fill history"
    pred  = torch.cat(predictions)   # (N, pred_dim)
    truth = torch.cat(ground_truth)  # (N, pred_dim)
    ate   = metric_ate(pred, truth)
    print(f"[OK] deployment sim: {T} steps → {len(predictions)} predictions, ATE={ate:.4f} m")


def _dispatch_train(mode, loader, model, optimizer, cfg, pred_dim, device, gen,
                    rollout_steps=16, cycle=0, n_cycles=1):
    """Call the appropriate trainer.py training function for the given mode.

    gen           — persistent torch.Generator; caller owns it and lets it advance
    rollout_steps — from mcfg["training"]["rollout_steps"]
    cycle         — current cycle index (0-based), used for schedule-prob ramp
    n_cycles      — total cycles, used for schedule-prob ramp
    """
    from train.trainer import (
        train_loop, train_rollout_loop, train_rollout_horizon_loop,
        train_gml_loop, train_gml_rollout_loop,
    )
    from nn.online_learner import _sched_prob

    kw = dict(batch_size=cfg.get("batch_size", 4), pred_dim=pred_dim)
    if mode == "standard":
        train_loop(loader, model, optimizer, **kw)
    elif mode in ("rollout", "rollout_horz", "gml_rollout"):
        sched = _sched_prob(cycle, n_cycles, cfg.get("max_schedule_prob", 0.0))
        rollout_kw = dict(
            rollout_max_steps=rollout_steps,
            schedule_prob=sched,
            **kw,
        )
        if mode == "rollout":
            train_rollout_loop(loader, model, optimizer, gen, **rollout_kw)
        elif mode == "rollout_horz":
            train_rollout_horizon_loop(loader, model, optimizer, gen, **rollout_kw)
        else:
            train_gml_rollout_loop(loader, model, optimizer, gen, **rollout_kw)
    elif mode == "gml":
        train_gml_loop(loader, model, optimizer,
                       batch_size=cfg.get("batch_size", 4), device=str(device))
    else:
        raise ValueError(f"unknown training_mode: {mode!r}")


def test_online_training_step():
    cfg    = _load_cfg()
    run    = Path(cfg["model_run_dir"])
    device = torch.device(cfg.get("device", "cpu"))
    mcfg   = _load_hydra_cfg(run)
    model  = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(run), map_location=device))
    )
    model.to(device)

    input_len  = mcfg["input_len"]
    output_len = mcfg["output_len"]
    pred_dim   = mcfg["models"]["out_dim"]
    mode       = cfg.get("training_mode", "rollout")
    window     = input_len + output_len + mcfg["training"]["rollout_steps"]

    tensor = torch.load(cfg["flight_log_pt"])
    gen    = torch.Generator(device=device).manual_seed(cfg.get("seed", 0))

    with tempfile.TemporaryDirectory() as tmp:
        zarr_path  = Path(tmp) / "online.zarr"
        n_episodes = _save_as_episodes(tensor, zarr_path, episode_len=window * 10)
        assert n_episodes > 0, "log too short to form any episodes"

        dataset   = QuickDataset2(path=str(zarr_path),
                                  training_size=cfg.get("m_train_steps", 10),
                                  window_size=window)
        loader    = DataLoader(dataset, batch_size=cfg.get("batch_size", 4), num_workers=0)
        optimizer = torch.optim.AdamW(model.parameters(),
                                      lr=mcfg["training"]["lr"],
                                      weight_decay=cfg.get("weight_decay", 0.0))

        _dispatch_train(mode, loader, model, optimizer, cfg, pred_dim, device, gen,
                        rollout_steps=mcfg["training"]["rollout_steps"])

    print(f"[OK] training step ({mode})")


def test_online_learner_loop():
    """Simulate online_learner.main(): p_cycles of train → eval (ATE) → swap-if-better.

    Sliding window over the .pt log: each cycle shifts the training window
    forward by one segment, dropping the oldest data and adding the newest.
    Zarr size stays constant across cycles (no memory growth).

    Challenger persists across rejections — it keeps training on new windows
    rather than being reset to the champion's weights each cycle.
    """
    import copy
    from data.dataset import QuickDatasetStraight
    from evaluate.evaluator import evaluate

    cfg    = _load_cfg()
    run    = Path(cfg["model_run_dir"])
    device = torch.device(cfg.get("device", "cpu"))
    mcfg   = _load_hydra_cfg(run)
    model  = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(run), map_location=device))
    )
    model.to(device).eval()

    input_len    = mcfg["input_len"]
    output_len   = mcfg["output_len"]
    pred_dim     = mcfg["models"]["out_dim"]
    mode         = cfg.get("training_mode", "rollout")
    n_cycles     = cfg.get("p_cycles", 2)
    window_width = cfg.get("window_width", 2)  # segments kept in the sliding window
    window       = input_len + output_len + mcfg["training"]["rollout_steps"]

    tensor  = torch.load(cfg["flight_log_pt"])
    gen     = torch.Generator(device=device).manual_seed(cfg.get("seed", 0))
    # Divide the log so the window can slide n_cycles times
    n_segs  = n_cycles + window_width - 1
    seg_len = len(tensor) // n_segs
    assert seg_len > window * 2, (
        f"log too short: seg_len={seg_len} must be > {window * 2} frames"
    )

    def _build_window_zarr(zarr_path: Path, cycle: int) -> int:
        start  = cycle * seg_len
        end    = start + window_width * seg_len
        sliced = tensor[start:end]
        return _save_as_episodes(sliced, zarr_path, episode_len=window * 10)

    def _ate(m, zarr_path, ep_idx=0):
        m.eval()
        ds     = QuickDatasetStraight(path=str(zarr_path), episode_idx=ep_idx, window_size=window)
        loader = DataLoader(ds, batch_size=None, num_workers=0)
        pred, truth = evaluate(m, loader, input_len, output_len, device, pred_dim=pred_dim)
        return metric_ate(pred, truth)

    # Challenger persists across rejections — only reset after a swap
    challenger = copy.deepcopy(model)
    optimizer  = torch.optim.AdamW(challenger.parameters(),
                                   lr=mcfg["training"]["lr"],
                                   weight_decay=cfg.get("weight_decay", 0.0))

    with tempfile.TemporaryDirectory() as tmp:
        zarr_path = Path(tmp) / "online.zarr"

        for cycle in range(n_cycles):
            t_start = cycle * seg_len
            t_end   = t_start + window_width * seg_len
            n_eps   = _build_window_zarr(zarr_path, cycle)
            assert n_eps > 0, f"cycle {cycle}: window too short to form episodes"

            dataset = QuickDataset2(path=str(zarr_path),
                                    training_size=cfg.get("m_train_steps", 20),
                                    window_size=window,
                                    seed=cfg.get("seed", 0) + cycle)
            loader  = DataLoader(dataset, batch_size=cfg.get("batch_size", 4), num_workers=0)
            _dispatch_train(mode, loader, challenger, optimizer, cfg, pred_dim, device, gen,
                            rollout_steps=mcfg["training"]["rollout_steps"],
                            cycle=cycle, n_cycles=n_cycles)

            cur_ate = _ate(model,      zarr_path)
            new_ate = _ate(challenger, zarr_path)
            swapped = new_ate < cur_ate
            if swapped:
                model      = challenger
                challenger = copy.deepcopy(model)
                optimizer  = torch.optim.AdamW(challenger.parameters(),
                                               lr=mcfg["training"]["lr"],
                                               weight_decay=cfg.get("weight_decay", 0.0))
            print(f"[cycle {cycle}] frames {t_start}–{t_end}  "
                  f"cur_ATE={cur_ate:.4f}  new_ATE={new_ate:.4f}  "
                  f"→ {'swapped' if swapped else 'retained'} ({mode})")

    print("[OK] online_learner loop simulation complete")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    test_data_conversion()
    #test_inference_from_log()
    #test_online_training_step()
    test_online_learner_loop()
    print("\nAll tests passed.")
