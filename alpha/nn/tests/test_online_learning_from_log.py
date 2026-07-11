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
    window     = input_len + output_len

    tensor = torch.load(cfg["flight_log_pt"])

    with tempfile.TemporaryDirectory() as tmp:
        zarr_path  = Path(tmp) / "online.zarr"
        n_episodes = _save_as_episodes(tensor, zarr_path, episode_len=window * 10)
        assert n_episodes > 0, "log too short to form any episodes"

        dataset = QuickDataset2(path=str(zarr_path), training_size=10, window_size=window)
        loader  = DataLoader(dataset, batch_size=4, num_workers=0)

        optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
        model.train()
        batch  = next(iter(loader)).to(device)     # (4, window, 23)
        out    = model(batch[:, :input_len])       # (4, output_len, out_dim)
        loss   = torch.nn.functional.mse_loss(out[:, 0, :10], batch[:, input_len, :10])
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    print(f"[OK] training step: loss={loss.item():.4f}")


def test_online_learner_loop(n_cycles: int = 2, train_steps: int = 20):
    """Simulate online_learner.main(): n_cycles of train → eval (ATE) → swap-if-better.

    Replaces SHM/CollectorThread with log data; no coordinator required.
    """
    import copy
    from data.dataset import QuickDatasetStraight
    from evaluate.evaluator import evaluate
    from evaluate.metrics import metric_ate

    cfg    = _load_cfg()
    run    = Path(cfg["model_run_dir"])
    device = torch.device(cfg.get("device", "cpu"))
    mcfg   = _load_hydra_cfg(run)
    model  = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(run), map_location=device))
    )
    model.to(device).eval()

    input_len = mcfg["input_len"]
    output_len = mcfg["output_len"]
    out_dim   = mcfg["models"]["out_dim"]
    window    = input_len + output_len

    tensor = torch.load(cfg["flight_log_pt"])

    def _ate(m, zarr_path, ep_idx=0):
        m.eval()
        ds     = QuickDatasetStraight(path=str(zarr_path), episode_idx=ep_idx, window_size=window)
        loader = DataLoader(ds, batch_size=None, num_workers=0)
        pred, truth = evaluate(m, loader, input_len, output_len, device, pred_dim=out_dim)
        return metric_ate(pred, truth)

    with tempfile.TemporaryDirectory() as tmp:
        zarr_path = Path(tmp) / "online.zarr"
        n_eps = _save_as_episodes(tensor, zarr_path, episode_len=window * 10)
        assert n_eps > 0, "log too short to form any episodes"

        for cycle in range(n_cycles):
            challenger = copy.deepcopy(model)
            challenger.train()
            optimizer = torch.optim.AdamW(challenger.parameters(), lr=1e-4)

            dataset = QuickDataset2(path=str(zarr_path), training_size=train_steps, window_size=window)
            loader  = DataLoader(dataset, batch_size=4, num_workers=0)
            for batch in loader:
                batch = batch.to(device)
                out  = challenger(batch[:, :input_len])
                loss = torch.nn.functional.mse_loss(out[:, 0, :out_dim], batch[:, input_len, :out_dim])
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            cur_ate = _ate(model,      zarr_path)
            new_ate = _ate(challenger, zarr_path)
            swapped = new_ate < cur_ate
            if swapped:
                model = challenger
            print(f"[cycle {cycle}] cur_ATE={cur_ate:.4f}  new_ATE={new_ate:.4f}  "
                  f"→ {'swapped' if swapped else 'retained'}")

    print("[OK] online_learner loop simulation complete")


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    test_data_conversion()
    test_inference_from_log()
    test_online_training_step()
    test_online_learner_loop()
    print("\nAll tests passed.")
