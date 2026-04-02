#!/usr/bin/env python3
"""RL Inference Server — runs with conda Python (has torch/skrl).

Reads 139-float32 observations from stdin, writes 2-int32 actions to stdout.
Launched as subprocess by rl_policy_node.py.

Protocol (binary, little-endian):
  stdin:  139 × float32 = 556 bytes per obs
  stdout: 2 × int32     = 8 bytes per action [accel_idx, omega_idx]
"""

import sys
import os
import struct
from pathlib import Path

import numpy as np

# IsaacLab scripts path
sys.path.insert(0, '/home/aa/IsaacLab/scripts/reinforcement_learning/skrl')

import torch

OBS_DIM = 139
OBS_BYTES = OBS_DIM * 4    # float32
ACT_BYTES = 2 * 4           # 2 × int32

CHECKPOINT_DEFAULT = (
    '/home/aa/IsaacLab/logs/skrl/'
    'Isaac-Navigation-Charge-VLP16-Curriculum-NavRL-AC/'
    'rw_groundv8_openendedv1__seed1_nowalls_diag_v9/'
    'checkpoints/agent_23436.pt'
)


def load_model(checkpoint_path: str, device: str, deterministic: bool):
    from vlp16_models import VLP16DiscretePolicy
    from gymnasium.spaces import Box, MultiDiscrete

    obs_space = Box(low=-np.inf, high=np.inf, shape=(OBS_DIM,))
    act_space = MultiDiscrete([19, 19])

    policy = VLP16DiscretePolicy(
        observation_space=obs_space,
        action_space=act_space,
        device=torch.device(device),
    )

    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=False)
    policy.load_state_dict(ckpt['policy'])
    policy.eval()
    policy.to(torch.device(device))

    # State preprocessor (RunningStandardScaler or CADN)
    normalizer = None
    if 'state_preprocessor' in ckpt:
        sp = ckpt['state_preprocessor']
        if 'running_mean' in sp:
            # RunningStandardScaler — simple (x - mean) / sqrt(var + eps)
            mean = sp['running_mean'].float().to(device)
            var = sp['running_variance'].float().to(device)
            normalizer = ('rss', mean, var)
            print(f'[INFERENCE SERVER] RunningStandardScaler loaded (count={sp.get("current_count", "?")})',
                  file=sys.stderr)
        elif 'branch_state.mu_f' in sp:
            # CADN
            try:
                from cadn_preprocessor import CurriculumAwareDualRateNormalizer
                cadn = CurriculumAwareDualRateNormalizer(size=OBS_DIM, device=device)
                cadn.load_state_dict(sp)
                cadn.eval()
                normalizer = ('cadn', cadn)
                print(f'[INFERENCE SERVER] CADN loaded OK', file=sys.stderr)
            except Exception as e:
                print(f'[WARN] CADN load failed: {e}', file=sys.stderr)
        else:
            print(f'[WARN] Unknown state_preprocessor format: {list(sp.keys())[:5]}', file=sys.stderr)
    else:
        print('[WARN] No state_preprocessor in checkpoint', file=sys.stderr)

    return policy, normalizer, torch.device(device), deterministic


def run_server(checkpoint_path: str, device: str, deterministic: bool, skip_cadn: bool = False):
    policy, normalizer, dev, det = load_model(checkpoint_path, device, deterministic)
    if skip_cadn:
        normalizer = None
        print('[INFERENCE SERVER] Normalizer skipped (--no-cadn)', file=sys.stderr)
    print(f'[INFERENCE SERVER] Ready (device={device}, deterministic={det}, normalizer={normalizer[0] if normalizer else None})',
          file=sys.stderr, flush=True)

    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer

    while True:
        # Read exactly OBS_BYTES
        data = b''
        while len(data) < OBS_BYTES:
            chunk = stdin.read(OBS_BYTES - len(data))
            if not chunk:
                return  # parent closed stdin → exit
            data += chunk

        obs = np.frombuffer(data, dtype=np.float32).copy()
        obs_t = torch.from_numpy(obs).unsqueeze(0).to(dev)

        if not hasattr(run_server, '_cnt'):
            run_server._cnt = 0
        run_server._cnt += 1

        with torch.inference_mode():
            if normalizer is not None:
                if normalizer[0] == 'rss':
                    _, mean, var = normalizer
                    eps = 1e-8
                    obs_t = (obs_t - mean) / torch.sqrt(var + eps)
                    obs_t = obs_t.clamp(-5.0, 5.0)
                elif normalizer[0] == 'cadn':
                    obs_t = normalizer[1](obs_t)
                if run_server._cnt % 25 == 1:
                    print(f'[NORM] obs_mean={obs_t.mean():.3f} obs_std={obs_t.std():.3f}',
                          file=sys.stderr, flush=True)
            logits, _ = policy.compute({'states': obs_t}, role='policy')

            if det:
                accel_idx = int(logits[0, :19].argmax().item())
                omega_idx = int(logits[0, 19:].argmax().item())
            else:
                accel_idx = int(
                    torch.distributions.Categorical(logits=logits[0, :19]).sample().item()
                )
                omega_idx = int(
                    torch.distributions.Categorical(logits=logits[0, 19:]).sample().item()
                )

        # Write 2 ints
        out = struct.pack('<ii', accel_idx, omega_idx)
        stdout.write(out)
        stdout.flush()


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', default=CHECKPOINT_DEFAULT)
    p.add_argument('--device', default='cuda:0')
    p.add_argument('--deterministic', action='store_true', default=True)
    p.add_argument('--no-cadn', action='store_true', default=False, help='Skip CADN normalization')
    args = p.parse_args()

    run_server(args.checkpoint, args.device, args.deterministic, skip_cadn=args.no_cadn)
