"""
Simulate online learning from a pre-converted .pt flight log tensor.
No SHM, no coordinator, no running services required.

Usage (from alpha/):
    python nn/tests/test_online_learning_from_log.py <flight_log.pt>

The .pt file must be a (T, 23) float32 tensor in the same normalised layout
the model was trained on (pos/3, lin_vel/0.8, quat_frd, ang_vel/0.5,
acc/25, thrust/9.81, setpoint/3). Produce one with Thesis-NN/data/converter.py.

Model path: set MODEL_RUN_DIR below, or in nn/config/inferrer.yaml (model_run_dir key).
"""

import sys
import tempfile
from pathlib import Path

import numpy as np
import torch
import zarr
from torch.utils.data import DataLoader

_ALPHA     = Path(__file__).parent.parent.parent
_THESIS_NN = _ALPHA.parent.parent / "Thesis-NN"
sys.path.insert(0, str(_THESIS_NN))

from data.dataset import FlightLog, QuickDataset2
from deploy.estimator import (
    _load_hydra_cfg, _find_checkpoint, _build_model, _strip_compile_prefix,
)

# ── Config ────────────────────────────────────────────────────────────────────
MODEL_RUN_DIR = Path("/home/nodale/Thesis/Thesis-NN/latest_model/09-22-02/0")

_N_DIM = 23


def _save_as_episodes(tensor: torch.Tensor, zarr_path: Path, episode_len: int):
    """Chop tensor into episodes in the CollectorThread zarr format."""
    data = tensor.numpy()
    T    = (len(data) // episode_len) * episode_len
    eps  = data[:T].reshape(-1, episode_len, _N_DIM).astype(np.float32)
    store = zarr.open(str(zarr_path), mode="w")
    store.create_dataset("episodes", data=eps, chunks=(1, episode_len, _N_DIM))
    return eps.shape[0]


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_data_conversion(pt_path: Path):
    tensor = torch.load(pt_path)
    assert tensor.ndim == 2 and tensor.shape[1] == _N_DIM, f"bad shape {tensor.shape}"
    print(f"[OK] data loaded: {tensor.shape}")


def test_inference_from_log(pt_path: Path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    mcfg   = _load_hydra_cfg(MODEL_RUN_DIR)
    model  = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(MODEL_RUN_DIR), map_location=device))
    )
    model.to(device).eval()

    input_len  = mcfg["input_len"]
    output_len = mcfg["output_len"]

    tensor  = torch.load(pt_path)
    dataset = FlightLog(data=tensor, window_size=input_len + output_len)
    loader  = DataLoader(dataset, batch_size=None, num_workers=0)

    with torch.inference_mode():
        window = next(iter(loader)).to(device)         # (input_len+output_len, 23)
        out    = model(window[:input_len].unsqueeze(0))  # (1, output_len, out_dim)

    print(f"[OK] inference: {window.shape} → {out.shape}")


def test_online_training_step(pt_path: Path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    mcfg   = _load_hydra_cfg(MODEL_RUN_DIR)
    model  = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(MODEL_RUN_DIR), map_location=device))
    )
    model.to(device)

    input_len  = mcfg["input_len"]
    output_len = mcfg["output_len"]
    window     = input_len + output_len

    tensor = torch.load(pt_path)

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


def test_online_learner_loop(pt_path: Path, n_cycles: int = 2, train_steps: int = 20):
    """Simulate online_learner.main(): n_cycles of train → eval (ATE) → swap-if-better.

    Replaces SHM/CollectorThread with log data; no coordinator required.
    """
    import copy
    from data.dataset import QuickDatasetStraight
    from evaluate.evaluator import evaluate
    from evaluate.metrics import metric_ate

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    mcfg   = _load_hydra_cfg(MODEL_RUN_DIR)
    model  = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(MODEL_RUN_DIR), map_location=device))
    )
    model.to(device).eval()

    input_len = mcfg["input_len"]
    output_len = mcfg["output_len"]
    out_dim   = mcfg["models"]["out_dim"]
    window    = input_len + output_len

    tensor = torch.load(pt_path)

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
    if len(sys.argv) < 2:
        print("Usage: python nn/tests/test_online_learning_from_log.py <flight_log.pt>")
        sys.exit(1)

    pt_path = Path(sys.argv[1])
    test_data_conversion(pt_path)
    test_inference_from_log(pt_path)
    test_online_training_step(pt_path)
    test_online_learner_loop(pt_path)
    print("\nAll tests passed.")
