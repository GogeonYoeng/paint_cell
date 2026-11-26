#!/usr/bin/env python3
"""8축 플래닝 결과(seq_*_joint.csv)를 실행기용 1 ms 단일 타임라인으로 합친다.

입력: 3_transition.py 가 만든 세그먼트 CSV 들 (8축 SI 형식)
    t,s,gantry_x,gantry_y,joint_1..joint_6,qd_gantry_x,...,qd_joint_6,qdd_...
    - 회전 rad, 직동 m. row 는 1 ms, 전이(move)는 10 ms 간격이라 그대로 이어 붙일 수 없다.
    - 6축 deg 형식(q1..q6)은 단위가 달라 거부한다.

출력:
    <out>.csv   k,t,seg,gantry_x_m,gantry_y_m,joint_1_rad..joint_6_rad,
                gantry_x_mps,gantry_y_mps,joint_1_radps..joint_6_radps
                (k = 0..N-1, t = k*dt 정확히, 컬럼 이름에 단위를 박아 rad/deg 혼동을 막는다)
    <out>.yaml  단위, 축 순서, 세그먼트 경계(k), 시작/끝 자세, 한계 점검, sha256

보간 (--interp):
    spline  (기본) 세그먼트마다 위치 q 의 C2 3차 스플라인, 양 끝 속도만 파일 qd 로 고정.
            출력 속도는 위치의 해석적 미분이라 위치·속도 지령이 서로 일관된다.
    hermite 위치·속도 노드를 모두 지키는 3차 Hermite. 파일의 q 와 qd 가 서로 안 맞으면 가속도 스파이크가 생긴다
            (2026-11-23 도어 t90 전이 구간: q 차분 2.8 m/s^2 vs qdd 컬럼 1.5 m/s^2 → Hermite 5 m/s^2).
세그먼트 경계는 다음 세그먼트 첫 점과 같아야 하며(이음새 검사), 어긋나면 실패한다.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import math
import sys
from pathlib import Path

import numpy as np

AXES = ["gantry_x", "gantry_y", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
PRISMATIC = np.array([True, True, False, False, False, False, False, False])
POS_COLS = [f"{a}_m" if p else f"{a}_rad" for a, p in zip(AXES, PRISMATIC)]
VEL_COLS = [f"{a}_mps" if p else f"{a}_radps" for a, p in zip(AXES, PRISMATIC)]


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_segment(path: Path):
    with path.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader if r and any(c.strip() for c in r)]
    col = {name: i for i, name in enumerate(header)}
    if "q1" in col or "q1_deg" in col:
        raise ValueError(f"{path}: 6축 deg 형식(q1..)이다. 8축 SI 형식(gantry_x,..,joint_6)만 받는다")
    missing = [n for n in ["t"] + AXES + [f"qd_{a}" for a in AXES] if n not in col]
    if missing:
        raise ValueError(f"{path}: 컬럼 없음 {missing}")
    data = np.array([[float(r[col[n]]) for n in ["t"] + AXES + [f"qd_{a}" for a in AXES]]
                     for r in rows], dtype=float)
    t, q, qd = data[:, 0], data[:, 1:9], data[:, 9:17]
    if len(t) < 2:
        raise ValueError(f"{path}: 점이 2개 미만")
    if not np.all(np.diff(t) > 0):
        raise ValueError(f"{path}: t 가 증가하지 않는다")
    if not np.all(np.isfinite(data)):
        raise ValueError(f"{path}: NaN/Inf")
    # 회전축이 deg 로 들어왔는지 휴리스틱 검사 (|q| > 2.5π 이면 rad 일 수 없다)
    if np.max(np.abs(q[:, ~PRISMATIC])) > 2.5 * math.pi:
        raise ValueError(f"{path}: 회전축 값이 2.5π 를 넘는다. deg 로 저장된 파일 아닌가?")
    return t - t[0], q, qd


def hermite(t_nodes, q, qd, t_eval):
    """구간별 3차 Hermite. t_eval 은 [t0, tN] 안."""
    idx = np.clip(np.searchsorted(t_nodes, t_eval, side="right") - 1, 0, len(t_nodes) - 2)
    t0, t1 = t_nodes[idx], t_nodes[idx + 1]
    h = (t1 - t0)[:, None]
    s = ((t_eval - t0) / (t1 - t0))[:, None]
    s2, s3 = s * s, s * s * s
    h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s, -2 * s3 + 3 * s2, s3 - s2
    p0, p1, m0, m1 = q[idx], q[idx + 1], qd[idx], qd[idx + 1]
    pos = h00 * p0 + h10 * h * m0 + h01 * p1 + h11 * h * m1
    dh00, dh10, dh01, dh11 = 6 * s2 - 6 * s, 3 * s2 - 4 * s + 1, -6 * s2 + 6 * s, 3 * s2 - 2 * s
    vel = (dh00 * p0 + dh01 * p1) / h + dh10 * m0 + dh11 * m1
    return pos, vel


def spline(t_nodes, q, qd, t_eval):
    """위치 C2 스플라인 (끝점 속도 고정). 출력 속도 = 스플라인 미분."""
    from scipy.interpolate import CubicSpline
    cs = CubicSpline(t_nodes, q, axis=0, bc_type=((1, qd[0]), (1, qd[-1])))
    return cs(t_eval), cs(t_eval, 1)


def velocity_consistency(t, q, qd):
    """노드 qd 와 위치 차분의 최대 불일치 (Hermite 가 과도하게 휘는지 확인용)."""
    fd = np.gradient(q, t, axis=0)
    return np.max(np.abs(fd - qd), axis=0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", type=Path, help="세그먼트 CSV 들 또는 디렉터리 하나")
    ap.add_argument("--pattern", default="seq_*_joint.csv", help="디렉터리 입력 시 glob")
    ap.add_argument("-o", "--output", type=Path, required=True, help="출력 경로 (확장자 없이 또는 .csv)")
    ap.add_argument("--dt", type=float, default=0.001)
    ap.add_argument("--interp", choices=["spline", "hermite"], default="spline")
    ap.add_argument("--node-vel-tol", type=float, default=1e-3,
                    help="파일 qd 와 위치 차분 불일치 경고 기준 [rad/s|m/s]")
    ap.add_argument("--seam-pos-tol", type=float, default=1e-5, help="이음새 위치 허용 [rad|m]")
    ap.add_argument("--seam-vel-tol", type=float, default=0.01,
                    help="이음새 속도 점프 허용 [rad/s|m/s]. 넘으면 실패 (갠트리는 실행기에서 가속도 클램프가 없다)")
    ap.add_argument("--gantry-vmax", type=float, default=0.5, help="갠트리 속도 한계 [m/s] (env motor_limits)")
    ap.add_argument("--gantry-amax", type=float, default=1.6, help="갠트리 가속도 한계 [m/s^2]")
    ap.add_argument("--arm-vmax-deg", type=float, default=70.0, help="팔 속도 한계 [deg/s] (RT set_velj_rt)")
    ap.add_argument("--arm-amax-deg", type=float, default=500.0, help="팔 가속도 한계 [deg/s^2] (실행기 클램프)")
    args = ap.parse_args(argv)

    if len(args.inputs) == 1 and args.inputs[0].is_dir():
        files = sorted(args.inputs[0].glob(args.pattern))
    else:
        files = list(args.inputs)
    if not files:
        print("[compile] 입력 파일 없음", file=sys.stderr)
        return 2

    out = args.output
    out_csv = out if out.suffix == ".csv" else out.with_suffix(".csv")
    out_yaml = out_csv.with_suffix(".yaml")
    dt = args.dt

    segs = []
    for f in files:
        t, q, qd = load_segment(f)
        segs.append({"path": f, "t": t, "q": q, "qd": qd, "T": float(t[-1]),
                     "dt_src": float(np.median(np.diff(t))),
                     "vel_mismatch": velocity_consistency(t, q, qd)})

    # 이음새 검사
    errors, warnings = [], []
    for a, b in zip(segs[:-1], segs[1:]):
        dp = np.abs(a["q"][-1] - b["q"][0])
        dv = np.abs(a["qd"][-1] - b["qd"][0])
        if np.max(dp) > args.seam_pos_tol:
            errors.append(f"이음새 위치 불연속 {a['path'].name} -> {b['path'].name}: "
                          f"{AXES[int(np.argmax(dp))]} {np.max(dp):.3e}")
        if np.max(dv) > args.seam_vel_tol:
            errors.append(f"이음새 속도 차 {a['path'].name} -> {b['path'].name}: "
                            f"{AXES[int(np.argmax(dv))]} {np.max(dv):.3e}")
    if errors:
        for e in errors:
            print("[compile][error]", e, file=sys.stderr)
        return 1

    # 전역 1 ms 격자
    starts = np.cumsum([0.0] + [s["T"] for s in segs[:-1]])
    total = float(starts[-1] + segs[-1]["T"])
    n = int(math.floor(total / dt + 1e-9)) + 1
    t_glob = np.arange(n) * dt
    seg_of = np.clip(np.searchsorted(starts, t_glob, side="right") - 1, 0, len(segs) - 1)
    pos = np.empty((n, 8))
    vel = np.empty((n, 8))
    for i, s in enumerate(segs):
        mask = seg_of == i
        if not np.any(mask):
            warnings.append(f"{s['path'].name}: 1 ms 샘플이 하나도 없다 (T={s['T']:.4f}s)")
            continue
        local = np.clip(t_glob[mask] - starts[i], 0.0, s["T"])
        interp = spline if args.interp == "spline" else hermite
        pos[mask], vel[mask] = interp(s["t"], s["q"], s["qd"], local)
        if np.max(s["vel_mismatch"]) > args.node_vel_tol:
            j = int(np.argmax(s["vel_mismatch"]))
            warnings.append(f"{s['path'].name}: 파일 qd 와 위치 차분 불일치 {AXES[j]} "
                            f"{s['vel_mismatch'][j]:.2e} (계획 q·qd 비일관)")

    # 총 시간이 dt 배수가 아니면 마지막 tick 은 계획 끝점보다 end_gap 만큼 앞이다 (끝점 위치를 따로 넣지 않음)
    end_gap = total - t_glob[-1]

    acc = np.gradient(vel, dt, axis=0)
    vmax = np.max(np.abs(vel), axis=0)
    amax = np.max(np.abs(acc), axis=0)
    limit_v = np.where(PRISMATIC, args.gantry_vmax, math.radians(args.arm_vmax_deg))
    limit_a = np.where(PRISMATIC, args.gantry_amax, math.radians(args.arm_amax_deg))
    step = np.max(np.abs(np.diff(pos, axis=0)), axis=0)
    for i, a in enumerate(AXES):
        if vmax[i] > limit_v[i]:
            warnings.append(f"{a} 속도 {vmax[i]:.4f} > 한계 {limit_v[i]:.4f} ({vmax[i] / limit_v[i]:.2f}x)")
        if amax[i] > limit_a[i]:
            warnings.append(f"{a} 가속도 {amax[i]:.4f} > 한계 {limit_a[i]:.4f} ({amax[i] / limit_a[i]:.2f}x)")

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["k", "t", "seg"] + POS_COLS + VEL_COLS)
        for k in range(n):
            w.writerow([k, f"{t_glob[k]:.6f}", int(seg_of[k])]
                       + [f"{v:.9f}" for v in pos[k]] + [f"{v:.9f}" for v in vel[k]])

    def axis_dict(arr, deg=False):
        return {a: float(math.degrees(v) if (deg and not p) else v) for a, v, p in zip(AXES, arr, PRISMATIC)}

    seg_meta = []
    for i, s in enumerate(segs):
        ks = np.flatnonzero(seg_of == i)
        seg_meta.append({"index": i, "name": s["path"].stem,
                         "k_start": int(ks[0]) if len(ks) else None,
                         "k_end": int(ks[-1]) if len(ks) else None,
                         "t_start": float(starts[i]), "duration": s["T"], "dt_source": s["dt_src"],
                         "source_sha256": sha256_of(s["path"]),
                         "node_velocity_mismatch_max": float(np.max(s["vel_mismatch"]))})

    meta = {
        "version": 1,
        "dt": dt,
        "n_ticks": n,
        "duration": total,
        "end_gap": float(end_gap),
        "axes": AXES,
        "position_columns": POS_COLS,
        "velocity_columns": VEL_COLS,
        "units": {"revolute": "rad, rad/s", "prismatic": "m, m/s"},
        "interpolation": args.interp,
        "start_q": axis_dict(pos[0]),
        "end_q": axis_dict(pos[-1]),
        "start_q_deg": axis_dict(pos[0], deg=True),
        "max_abs_velocity": axis_dict(vmax),
        "max_abs_acceleration": axis_dict(amax),
        "max_abs_step_per_tick": axis_dict(step),
        "limits": {"gantry_vmax_mps": args.gantry_vmax, "gantry_amax_mps2": args.gantry_amax,
                   "arm_vmax_deg_s": args.arm_vmax_deg, "arm_amax_deg_s2": args.arm_amax_deg},
        "warnings": warnings,
        "segments": seg_meta,
        "csv": out_csv.name,
        "csv_sha256": sha256_of(out_csv),
    }
    try:
        import yaml  # type: ignore
        out_yaml.write_text(yaml.safe_dump(meta, sort_keys=False, allow_unicode=True), encoding="utf-8")
    except ImportError:
        import json
        out_yaml.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"[compile] 세그먼트 {len(segs)}개, {n} tick ({total:.3f} s) -> {out_csv}")
    print(f"[compile] 최대 속도  : " + ", ".join(
        f"{a}={vmax[i]:.3f}" + ("m/s" if PRISMATIC[i] else f"({math.degrees(vmax[i]):.1f}°/s)")
        for i, a in enumerate(AXES)))
    print(f"[compile] 최대 가속도: " + ", ".join(
        f"{a}={amax[i]:.3f}" + ("m/s²" if PRISMATIC[i] else f"({math.degrees(amax[i]):.0f}°/s²)")
        for i, a in enumerate(AXES)))
    for wmsg in warnings:
        print("[compile][warn]", wmsg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
