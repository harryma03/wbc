"""Offline USD joint-chain IK audit (requires pxr, numpy and scipy).

This checks pose reachability, not collisions or dynamic stability.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from pxr import Usd, UsdPhysics


def transform(pos, quat):
    t = np.eye(4)
    t[:3, :3] = R.from_quat([*quat.GetImaginary(), quat.GetReal()]).as_matrix()
    t[:3, 3] = pos
    return t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--radius', type=float, default=.60)
    parser.add_argument('--full-limits', action='store_true', help='Use 90 percent of USD limits instead of legacy action limits')
    parser.add_argument('--bounds', type=float, nargs=6, default=[.25, .55, -.30, .30, .50, .82])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    stage = Usd.Stage.Open(str(root / 'source/LeggedManip_Lab/LeggedManip_Lab/assets/titatit_piper/titatit_piper.usd'))
    joints = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(prim)
            parents, children = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
            if parents and children:
                joints[str(children[0])] = (str(parents[0]), j)
    end = next(str(p.GetPath()) for p in stage.Traverse() if p.GetName() == 'end_effector')
    chain = []
    path = end
    while path.split('/')[-1] != 'link0':
        parent, joint = joints[path]
        chain.append(joint)
        path = parent
    chain.reverse()
    # Mount height relative to the chassis, using authored joint frames.
    mount = np.eye(4)
    mount_path = path
    while mount_path in joints:
        parent, j = joints[mount_path]
        mount = transform(j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get()) @ np.linalg.inv(transform(j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get())) @ mount
        mount_path = parent
    height = .30 + mount[2, 3]
    specs = []
    hard = []
    for j in chain:
        a = transform(j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get())
        b = np.linalg.inv(transform(j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get()))
        axis = None
        if j.GetPrim().IsA(UsdPhysics.RevoluteJoint):
            rev = UsdPhysics.RevoluteJoint(j.GetPrim())
            axis = np.eye(3)['XYZ'.index(rev.GetAxisAttr().Get())]
            hard.append(np.deg2rad([rev.GetLowerLimitAttr().Get(), rev.GetUpperLimitAttr().Get()]))
        specs.append((a, b, axis))
    # Legacy action ranges for comparison; --full-limits uses the new ranges.
    low = np.array([-.75, 0., -1.5, -1., -1., -1.])
    high = np.array([.75, 1.5, 0., 1., 1., 1.])
    if args.full_limits:
        low, high = (np.array(hard) * .90).T
    assert len(hard) == 6
    assert np.all(low >= np.array(hard)[:, 0]) and np.all(high <= np.array(hard)[:, 1])

    def fk(q):
        t = np.eye(4)
        k = 0
        for a, b, axis in specs:
            rot = np.eye(4)
            if axis is not None:
                rot[:3, :3] = R.from_rotvec(axis * q[k]).as_matrix()
                k += 1
            t = t @ a @ rot @ b
        return t

    rng = np.random.default_rng(42)
    bounds = np.array(args.bounds).reshape(3, 2)
    errors = []
    for p in rng.uniform(bounds[:, 0], bounds[:, 1], (args.samples, 3)):
        p[2] -= height
        p *= min(1., args.radius / max(np.linalg.norm(p), 1e-9))
        pitch = -np.arctan2(p[2], np.linalg.norm(p[:2]))
        yaw = np.arctan2(p[1], p[0])
        angles = np.clip([0., pitch, yaw], -3.14 / 4, 3.14 / 3)
        target = R.from_euler('xyz', angles) * R.from_euler('y', np.pi / 2)

        def residual(q):
            t = fk(q)
            return np.r_[t[:3, 3] - p, .2 * (target.inv() * R.from_matrix(t[:3, :3])).as_rotvec()]

        best = None
        for start in [(low + high) / 2, rng.uniform(low, high), rng.uniform(low, high)]:
            sol = least_squares(residual, start, bounds=(low, high), max_nfev=100)
            e = residual(sol.x)
            score = np.linalg.norm(e)
            if best is None or score < best[0]:
                best = (score, np.linalg.norm(e[:3]), np.linalg.norm(e[3:]) / .2)
        errors.append(best[1:])
    e = np.array(errors)
    print(json.dumps({'samples': args.samples, 'bounds': args.bounds, 'mount_world_height': height,
                      'zero_pose': fk(np.zeros(6))[:3, 3].tolist(),
                      'hard_limits_rad': np.array(hard).tolist(),
                      'success_2cm_10deg': float(np.mean((e[:, 0] < .02) & (e[:, 1] < np.deg2rad(10)))),
                      'max_position_error': float(e[:, 0].max()),
                      'p95_position_error': float(np.quantile(e[:, 0], .95)),
                      'max_orientation_error_rad': float(e[:, 1].max())}, indent=2))


if __name__ == '__main__':
    main()
