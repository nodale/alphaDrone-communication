"""
nn/online_learner.py

Online generational NN learning against Vicon ground truth.
Controlled by the coordinator via NN_CTRL SHM channel (NN_CTRL[1]).

Three concurrent roles:
  InferenceThread  — SHM → rolling history → forward pass → zarr log
                     Uses high-priority CUDA stream; runs only while NN_CTRL[1]==1
  CollectorThread  — SHM + VICON_STATE → vicon-supervised episodes → online zarr
                     Runs only while NN_CTRL[1]==1
  Main thread      — P cycles of: train → eval → swap-if-better
                     Each cycle waits for NN_CTRL[1]==1 before starting

GPU streams (Jetson Orin Nano unified memory):
  infer_stream  (priority 0):  current-gen forward passes
  train_stream  (priority -1): challenger training
  eval_stream   (priority -1): evaluation

Model swap uses a CPython-GIL-atomic attribute swap, so inference is never blocked.

Generations saved as .pth under cfg.gen_dir.

Run from alphaDrone-communication/alpha/:
    python -m nn.online_learner
"""

import gc
import sys
import math
import signal
import time
import threading
from pathlib import Path

import hydra
import torch
import zarr
import numpy as np
from omegaconf import DictConfig
from torch.utils.data import DataLoader

_HERE       = Path(__file__).parent
_THESIS     = _HERE.parent.parent.parent.parent     # …/Thesis/
sys.path.insert(0, str(_THESIS / "Thesis-NN"))

from model.network import JeuralJetwork
from deploy.estimator import (
    _load_hydra_cfg,
    _find_checkpoint,
    _build_model,
    _strip_compile_prefix,
)
from data.dataset import QuickDataset2, QuickDatasetStraight
from evaluate.evaluator import evaluate
from evaluate.metrics import metric_ate
from train.trainer import (
    train_loop,
    train_rollout_loop,
    train_rollout_horizon_loop,
    train_gml_rollout_loop,
)

from shm.bus import ShmReader
from shm import channels
from nn import await_ctrl

# ---------------------------------------------------------------------------
# Feature layout (23 dims, same as Thesis-NN training data)
# ---------------------------------------------------------------------------

_N_DIM = 23
_NORM = np.array(
    [3, 3, 3, 0.8, 0.8, 0.8, 1, 1, 1, 1, 0.5, 0.5, 0.5,
     25, 25, 25, 9.81, 9.81, 9.81, 9.81, 3, 3, 3],
    dtype=np.float32,
)


def _obs_norm(est, act, sp):
    """Normalised 23-dim vector for inference (pos/vel from ESTIMATED_STATE)."""
    v = np.zeros(_N_DIM, np.float32)
    v[0:3]   = est[0:3]
    v[3:6]   = est[3:6]
    v[6:10]  = est[6:10]
    v[10:13] = est[10:13]
    v[16:20] = act[0:4]
    v[20:23] = sp[0:3]
    return v / _NORM


def _frame_vicon(est, act, sp, vic_pos, vic_vel):
    """Normalised 23-dim vector replacing pos/vel with Vicon ground truth."""
    v = np.zeros(_N_DIM, np.float32)
    v[0:3]   = vic_pos  / 3.0
    v[3:6]   = vic_vel  / 0.8
    v[6:10]  = est[6:10]
    v[10:13] = est[10:13] / 0.5
    v[16:20] = act[0:4]   / 9.81
    v[20:23] = sp[0:3]    / 3.0
    return v


# ---------------------------------------------------------------------------
# Scheduled sampling: sigmoid ramp 0 → max_prob over P cycles
# ---------------------------------------------------------------------------

def _sched_prob(cycle, p_cycles, max_prob):
    x = 10.0 * (cycle / max(p_cycles - 1, 1) - 0.5)
    return max_prob / (1.0 + math.exp(-x))


# ---------------------------------------------------------------------------
# Model helpers
# ---------------------------------------------------------------------------

def _make_model(mcfg, mode, dev):
    out_dim = mcfg["models"]["out_dim"]
    if mode not in ("standard", "rollout", "rollout_horz"):
        out_dim *= 2
    return JeuralJetwork(
        n_dim=mcfg["models"]["n_dim"],
        out_dim=out_dim,
        input_len=mcfg["input_len"],
        output_len=mcfg["output_len"],
        **mcfg["models"]["architecture"],
    ).to(dev)


def _state_dict(model):
    return model._orig_mod.state_dict() if hasattr(model, "_orig_mod") else model.state_dict()


def _save_ckpt(model, path):
    torch.save(_state_dict(model), path)


# ---------------------------------------------------------------------------
# Training dispatch
# ---------------------------------------------------------------------------

def _run_train(mode, loader, model, optimizer, gen, cfg, mcfg, cycle):
    pred_dim = mcfg["models"]["out_dim"]
    sched    = _sched_prob(cycle, cfg.p_cycles, cfg.max_schedule_prob)
    kw = dict(batch_size=cfg.batch_size, pred_dim=pred_dim)

    if mode == "standard":
        train_loop(loader, model, optimizer, **kw)
    elif mode == "rollout":
        train_rollout_loop(loader, model, optimizer, gen,
                           rollout_max_steps=cfg.rollout_steps, schedule_prob=sched, **kw)
    elif mode == "rollout_horz":
        train_rollout_horizon_loop(loader, model, optimizer, gen,
                                   rollout_max_steps=cfg.rollout_steps, schedule_prob=sched, **kw)
    elif mode == "gml_rollout":
        # Warm-up cycle uses plain rollout; subsequent cycles use GML
        if cycle < 1:
            train_rollout_loop(loader, model, optimizer, gen,
                               rollout_max_steps=cfg.rollout_steps, schedule_prob=0.0, **kw)
        else:
            train_gml_rollout_loop(loader, model, optimizer, gen,
                                   rollout_max_steps=cfg.rollout_steps, schedule_prob=sched, **kw)


# ---------------------------------------------------------------------------
# Evaluation: equal mix of sim + online trajectories
# ---------------------------------------------------------------------------

def _eval_score(model, sim_path, online_path, n_traj, mcfg, dev):
    input_len  = mcfg["input_len"]
    output_len = mcfg["output_len"]
    window     = input_len + output_len
    model.eval()

    def _ate_on(path, idxs):
        ates = []
        for idx in idxs:
            ds     = QuickDatasetStraight(path=str(path), episode_idx=int(idx), window_size=window)
            loader = DataLoader(ds, batch_size=None, num_workers=0)
            pred, truth = evaluate(model, loader, input_len, output_len, dev)
            ates.append(metric_ate(pred, truth))
        return float(np.mean(ates)) if ates else float("inf")

    def _n_eps(path):
        try:
            return zarr.open(str(path), mode="r")["episodes"].shape[0]
        except Exception:
            return 0

    n_sim = _n_eps(sim_path)
    n_onl = _n_eps(online_path)
    n     = min(n_traj, n_sim)

    if n_onl > 0:
        n = min(n, n_onl)
        return (
            _ate_on(sim_path,    np.random.choice(n_sim, n, replace=False)) +
            _ate_on(online_path, np.random.choice(n_onl, n, replace=False))
        ) / 2
    return _ate_on(sim_path, np.random.choice(n_sim, n, replace=False)) if n > 0 else float("inf")


# ---------------------------------------------------------------------------
# Inference thread
# ---------------------------------------------------------------------------

class InferenceThread(threading.Thread):
    """Reads SHM → rolling history → forward pass → zarr.
    Pauses automatically when NN_CTRL[1] is cleared by the coordinator."""

    def __init__(self, model, mcfg, dev, ctrl_r, zarr_path, loop_hz, flush_every, out_dim):
        super().__init__(daemon=True)
        self.model       = model    # CPython attribute swap is GIL-atomic
        self.mcfg        = mcfg
        self.dev         = dev
        self.ctrl_r      = ctrl_r
        self.zarr_path   = zarr_path
        self.loop_hz     = loop_hz
        self.flush_every = flush_every
        self.out_dim     = out_dim
        self.stream      = torch.cuda.Stream(priority=0)
        self.stop_event  = threading.Event()

    def swap(self, new_model):
        """Atomically replace model reference (safe under CPython GIL)."""
        self.model = new_model

    def run(self):
        input_len = self.mcfg["input_len"]

        sr  = ShmReader(channels.ESTIMATED_STATE)
        ar  = ShmReader(channels.ACTUATION)
        spr = ShmReader(channels.GENERAL_SETPOINT)

        store = zarr.open(self.zarr_path, mode="a")
        for name, dim in [("obs", _N_DIM), ("output", self.out_dim)]:
            if name not in store:
                store.create_dataset(name, shape=(0, dim), chunks=(1024, dim), dtype="f4")

        hist   = torch.zeros(1, input_len, _N_DIM, device=self.dev)
        filled = 0
        obs_buf, out_buf = [], []
        period = 1.0 / self.loop_hz
        tnext  = time.perf_counter()

        while not self.stop_event.is_set():
            if not self.ctrl_r.data[1]:
                tnext = time.perf_counter()
                time.sleep(0.05)
                continue

            obs = _obs_norm(sr.data, ar.data, spr.data)
            hist = torch.roll(hist, -1, 1)
            hist[0, -1] = torch.from_numpy(obs).to(self.dev)
            filled = min(filled + 1, input_len)

            if filled >= input_len:
                model = self.model
                with torch.cuda.stream(self.stream), torch.inference_mode():
                    out = model(hist)[0, 0, :self.out_dim].cpu().numpy()
                obs_buf.append(obs)
                out_buf.append(out)

            if len(out_buf) >= self.flush_every:
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

        if out_buf:
            store["obs"].append(np.stack(obs_buf))
            store["output"].append(np.stack(out_buf))
        for r in (sr, ar, spr):
            r.close()


# ---------------------------------------------------------------------------
# Data collection thread
# ---------------------------------------------------------------------------

class CollectorThread(threading.Thread):
    """Reads SHM + Vicon; builds vicon-supervised episodes → online zarr.
    Pauses when NN_CTRL[1] is cleared."""

    def __init__(self, ctrl_r, online_path, episode_len, collect_hz, min_episodes):
        super().__init__(daemon=True)
        self.ctrl_r       = ctrl_r
        self.online_path  = Path(online_path)
        self.episode_len  = episode_len
        self.collect_hz   = collect_hz
        self.min_episodes = min_episodes
        self.stop_event   = threading.Event()
        self.ready_event  = threading.Event()
        self._buf         = []
        self._lock        = threading.Lock()

    def n_episodes(self):
        try:
            return zarr.open(str(self.online_path), mode="r")["episodes"].shape[0]
        except Exception:
            return 0

    def _flush(self):
        ep = np.stack(self._buf).reshape(1, self.episode_len, _N_DIM).astype(np.float32)
        store = zarr.open(str(self.online_path), mode="a")
        if "episodes" not in store:
            store.create_dataset(
                "episodes",
                shape=(0, self.episode_len, _N_DIM),
                chunks=(1, self.episode_len, _N_DIM),
                dtype="f4",
            )
        store["episodes"].append(ep)
        self._buf.clear()
        n = self.n_episodes()
        print(f"[collector] episode saved ({n} total)")
        if n >= self.min_episodes:
            self.ready_event.set()

    def run(self):
        sr  = ShmReader(channels.ESTIMATED_STATE)
        ar  = ShmReader(channels.ACTUATION)
        spr = ShmReader(channels.GENERAL_SETPOINT)
        vr  = ShmReader(channels.VICON_STATE)

        period   = 1.0 / self.collect_hz
        tnext    = time.perf_counter()
        prev_vic = vr.data[0, :3].copy()

        while not self.stop_event.is_set():
            if not self.ctrl_r.data[1]:
                tnext    = time.perf_counter()
                prev_vic = vr.data[0, :3].copy()   # reset so velocity doesn't spike on resume
                time.sleep(0.05)
                continue

            vic_pos = vr.data[0, :3].copy()
            vic_vel = (vic_pos - prev_vic) / period
            prev_vic = vic_pos

            frame = _frame_vicon(sr.data, ar.data, spr.data, vic_pos, vic_vel)

            with self._lock:
                self._buf.append(frame)
                if len(self._buf) >= self.episode_len:
                    self._flush()

            tnext += period
            sl = tnext - time.perf_counter()
            if sl > 0:
                time.sleep(sl)
            else:
                tnext = time.perf_counter()

        for r in (sr, ar, spr, vr):
            r.close()


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

@hydra.main(version_base=None, config_path="config", config_name="online_learner")
def main(cfg: DictConfig):
    gc.collect()
    gc.disable()

    dev = torch.device(cfg.device)
    run = Path(cfg.model_run_dir)

    # Wait for coordinator, then attach to NN_CTRL
    print("[nn.learner] waiting for coordinator…")
    ctrl_r = await_ctrl()
    print("[nn.learner] coordinator ready — starting (disabled until 'u' is pressed)")

    # Load base model (generation 0)
    mcfg  = _load_hydra_cfg(run)
    model = _build_model(mcfg)
    model.load_state_dict(
        _strip_compile_prefix(torch.load(_find_checkpoint(run), map_location=dev))
    )
    model.to(dev).eval()
    out_dim = mcfg["models"]["out_dim"]

    gen_dir = Path(cfg.gen_dir)
    gen_dir.mkdir(parents=True, exist_ok=True)
    _save_ckpt(model, gen_dir / "gen_initial.pth")

    train_stream = torch.cuda.Stream(priority=-1)
    eval_stream  = torch.cuda.Stream(priority=-1)

    # Start persistent threads
    infer_thread = InferenceThread(
        model=model, mcfg=mcfg, dev=dev, ctrl_r=ctrl_r,
        zarr_path=cfg.infer_zarr, loop_hz=cfg.loop_hz,
        flush_every=cfg.flush_every, out_dim=out_dim,
    )
    collector = CollectorThread(
        ctrl_r=ctrl_r, online_path=cfg.online_zarr,
        episode_len=cfg.episode_len, collect_hz=cfg.collect_hz,
        min_episodes=cfg.min_online_episodes,
    )
    infer_thread.start()
    collector.start()

    running = True
    def _stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT,  _stop)

    mode = cfg.training_mode
    gen  = torch.Generator(device="cuda").manual_seed(cfg.seed)

    for cycle in range(cfg.p_cycles):
        if not running:
            break

        # Wait for learner to be enabled and enough data to exist
        print(f"[nn.learner] cycle {cycle} — waiting for enable + data…")
        while running and (not ctrl_r.data[1] or not collector.ready_event.is_set()):
            time.sleep(1.0)
        if not running:
            break

        print(f"\n[nn.learner] ===== Cycle {cycle}/{cfg.p_cycles} "
              f"(sched={_sched_prob(cycle, cfg.p_cycles, cfg.max_schedule_prob):.3f}) =====")

        # Build challenger from current inference model
        challenger = _make_model(mcfg, mode, dev)
        challenger.load_state_dict(_state_dict(infer_thread.model))
        optimizer = torch.optim.AdamW(
            challenger.parameters(), lr=cfg.lr,
            weight_decay=cfg.weight_decay, eps=1e-12,
        )

        # Train
        window = mcfg["input_len"] + mcfg["output_len"] + cfg.rollout_steps
        online_ds = QuickDataset2(
            path=str(cfg.online_zarr), training_size=cfg.m_train_steps, window_size=window,
        )
        train_loader = DataLoader(
            online_ds, batch_size=cfg.batch_size, num_workers=2,
            pin_memory=True, multiprocessing_context="fork",
            persistent_workers=True, prefetch_factor=2,
        )
        print(f"[nn.learner] training challenger…")
        with torch.cuda.stream(train_stream):
            _run_train(mode, train_loader, challenger, optimizer, gen, cfg, mcfg, cycle)
        train_stream.synchronize()
        challenger.eval()

        # Evaluate (snapshot current gen to avoid racing with inference thread)
        print(f"[nn.learner] evaluating…")
        current_snap = _make_model(mcfg, mode, dev)
        current_snap.load_state_dict(_state_dict(infer_thread.model))
        current_snap.eval()

        with torch.cuda.stream(eval_stream):
            cur_score = _eval_score(current_snap, cfg.sim_dataset_path,
                                    cfg.online_zarr, cfg.n_eval_traj, mcfg, dev)
            new_score = _eval_score(challenger, cfg.sim_dataset_path,
                                    cfg.online_zarr, cfg.n_eval_traj, mcfg, dev)
        eval_stream.synchronize()
        del current_snap

        print(f"[nn.learner] cur_ATE={cur_score:.4f}  new_ATE={new_score:.4f}")

        if new_score < cur_score:
            ckpt = gen_dir / f"gen_cycle_{cycle:03d}.pth"
            _save_ckpt(challenger, ckpt)
            infer_thread.swap(challenger)   # GIL-atomic reference swap
            print(f"[nn.learner] new gen wins → swapped. Saved: {ckpt}")
        else:
            print(f"[nn.learner] current gen retained.")

    # Shutdown
    infer_thread.stop_event.set()
    collector.stop_event.set()
    infer_thread.join(timeout=5)
    collector.join(timeout=5)
    _save_ckpt(infer_thread.model, gen_dir / "gen_final.pth")
    ctrl_r.close()
    print("[nn.learner] done.")


if __name__ == "__main__":
    main()
