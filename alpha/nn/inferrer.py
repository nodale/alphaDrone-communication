"""
nn/inferrer.py

Reads necessary_msg from SHM (ESTIMATED_STATE, ACTUATION, GENERAL_SETPOINT),
feeds a rolling window into a trained JeuralJetwork, and logs each observation
+ model output to a Zarr store.

Feature layout fed to the model (23 dims, normalised — same as training data):
  [0:3]   pos    / 3.0     from ESTIMATED_STATE
  [3:6]   vel    / 0.8     from ESTIMATED_STATE
  [6:10]  quat             from ESTIMATED_STATE  (no scaling)
  [10:13] angvel / 0.5     from ESTIMATED_STATE
  [13:16] acc    / 25      not available — set to zero
  [16:20] thrust / 9.81    from ACTUATION (johnny_status actuation field)
  [20:23] setpoint / 3.0   from GENERAL_SETPOINT

Run from alphaDrone-communication/alpha/:
    python -m nn.inferrer
"""

import gc
import sys
import signal
import time
from pathlib import Path

import hydra
import torch
import zarr
import numpy as np
from omegaconf import DictConfig

# Thesis-NN lives two levels above alphaDrone-communication/
_THESIS_ROOT = Path(__file__).parent.parent.parent.parent
sys.path.insert(0, str(_THESIS_ROOT / "Thesis-NN"))

from deploy.estimator import (
    _load_hydra_cfg,
    _find_checkpoint,
    _build_model,
    _strip_compile_prefix,
)
from shm.bus import ShmReader
from shm import channels
from nn import await_ctrl

# ---------------------------------------------------------------------------
_N_DIM = 23
_NORM = np.array(
    [3, 3, 3, 0.8, 0.8, 0.8, 1, 1, 1, 1, 0.5, 0.5, 0.5,
     25, 25, 25, 9.81, 9.81, 9.81, 9.81, 3, 3, 3],
    dtype=np.float32,
)


def _build_obs(est, act, sp):
    """Assemble normalised 23-dim feature vector from SHM arrays."""
    v = np.zeros(_N_DIM, np.float32)
    v[0:3]   = est[0:3]    # pos
    v[3:6]   = est[3:6]    # vel
    v[6:10]  = est[6:10]   # quat  (acc[13:16] stays zero)
    v[10:13] = est[10:13]  # angvel
    v[16:20] = act[0:4]    # thrust
    v[20:23] = sp[0:3]     # setpoint
    return v / _NORM


# ---------------------------------------------------------------------------

@hydra.main(version_base=None, config_path="config", config_name="inferrer")
def main(cfg: DictConfig):
    gc.collect()
    gc.disable()

    dev = torch.device(cfg.device)
    run = Path(cfg.model_run_dir)

    # Load model from Hydra run dir
    mcfg  = _load_hydra_cfg(run)
    model = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(run), map_location=dev))
    )
    model.to(dev).eval()

    input_len = mcfg["input_len"]
    out_dim   = mcfg["models"]["out_dim"]

    # Wait for coordinator to create NN_CTRL, then attach
    print("[nn.inferrer] waiting for coordinator…")
    ctrl_r = await_ctrl()
    print("[nn.inferrer] coordinator ready — starting (disabled until 'i' is pressed)")

    # SHM readers (no ownership — never unlink)
    sr  = ShmReader(channels.ESTIMATED_STATE)
    ar  = ShmReader(channels.ACTUATION)
    spr = ShmReader(channels.GENERAL_SETPOINT)

    # Zarr output
    store = zarr.open(cfg.zarr_output, mode="a")
    for name, dim in [("obs", _N_DIM), ("output", out_dim)]:
        if name not in store:
            store.create_dataset(name, shape=(0, dim), chunks=(1024, dim), dtype="f4")

    # Per-step rolling history
    hist   = torch.zeros(1, input_len, _N_DIM, device=dev)
    filled = 0
    obs_buf, out_buf = [], []

    running = True
    def _stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT,  _stop)

    period = 1.0 / cfg.loop_hz
    tnext  = time.perf_counter()

    with torch.inference_mode():
        while running:
            # Pause when coordinator disables inferrer (NN_CTRL[0] == 0)
            if not ctrl_r.data[0]:
                tnext = time.perf_counter()
                time.sleep(0.05)
                continue

            obs = _build_obs(sr.data, ar.data, spr.data)

            hist = torch.roll(hist, -1, 1)
            hist[0, -1] = torch.from_numpy(obs).to(dev)
            filled = min(filled + 1, input_len)

            if filled >= input_len:
                out = model(hist)[0, 0, :out_dim].cpu().numpy()
                obs_buf.append(obs)
                out_buf.append(out)

            if len(out_buf) >= cfg.flush_every:
                store["obs"].append(np.stack(obs_buf))
                store["output"].append(np.stack(out_buf))
                obs_buf.clear()
                out_buf.clear()

            tnext += period
            sl = tnext - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            else:
                tnext = time.perf_counter()

    # Final flush
    if out_buf:
        store["obs"].append(np.stack(obs_buf))
        store["output"].append(np.stack(out_buf))

    sr.close()
    ar.close()
    spr.close()
    ctrl_r.close()
    print("[nn.inferrer] stopped.")


if __name__ == "__main__":
    main()
