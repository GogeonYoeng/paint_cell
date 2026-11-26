#!/usr/bin/env python3
"""PLC 모드(--gantry plc) 동기 검증. plc_sim.py 의 정답 로그(truth.csv)와 실행기 로그를 CLOCK_MONOTONIC 으로 맞춘다.

검사
  1. PLC 시계 추정   실행기 master_est vs 실제 PLC 마스터 시간 [ms]
  2. 캠 충실도       PLC 지령 X/Y vs 계획(마스터 시간 기준) [mm]  — 캠 보간·세그먼트 연결이 맞는가
  3. 장비 지연       갠트리 실측 / 로봇 실측이 계획을 몇 ms 늦게 따라가나 → 차이 = 동기 오차
  4. TCP 오차        FK(갠트리 실측, 로봇 실측) vs FK(계획(마스터 - 갠트리 지연)) [mm]

실기에서는 truth 대신 PLC 가 기록한 (마스터, 실측 X/Y) 로그와 PC 시각 대응이 필요하다 (README 참고).
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

AXES = ["gantry_x", "gantry_y", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def read_meta(path: Path) -> dict:
    out = {}
    for line in path.read_text().splitlines():
        if "=" in line and not line.startswith("#"):
            k, v = line.split("=", 1)
            out[k] = float(v)
    return out


def best_delay(fn, lo_ms=-20.0, hi_ms=30.0, step_ms=0.1):
    best = (math.inf, 0.0)
    for d in np.arange(lo_ms, hi_ms + 1e-9, step_ms):
        e = fn(d * 1e-3)
        if e < best[0]:
            best = (e, d)
    return best[1], best[0]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motion", type=Path, required=True)
    ap.add_argument("--cam-meta", type=Path, required=True)
    ap.add_argument("--truth", type=Path, required=True)
    ap.add_argument("--log", type=Path, required=True)
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--tcp-frame", default="tcp")
    ap.add_argument("--fk-stride", type=int, default=5)
    ap.add_argument("--tol-clock-ms", type=float, default=0.5)
    ap.add_argument("--tol-sync-ms", type=float, default=1.0)
    ap.add_argument("--tol-tcp-mm", type=float, default=1.0)
    ap.add_argument("--tol-cam-mm", type=float, default=0.05)
    ap.add_argument("--plot", type=Path)
    args = ap.parse_args(argv)

    meta = read_meta(args.cam_meta)
    lead = meta["lead_in"]
    sx, ox, sy, oy = meta["x_sign"], meta["x_offset_mm"], meta["y_sign"], meta["y_offset_mm"]
    M = np.loadtxt(args.motion, delimiter=",", skiprows=1)
    dt = M[1, 1] - M[0, 1]
    tp, qp = M[:, 1], M[:, 3:11]

    def plan(t):  # 계획 시간 t [s] → (N,8)
        t = np.clip(t, tp[0], tp[-1])
        return np.stack([np.interp(t, tp, qp[:, j]) for j in range(8)], axis=1)

    T = np.loadtxt(args.truth, delimiter=",", skiprows=1)
    t_mono, master, mvel = T[:, 0], T[:, 1], T[:, 2]
    xc, yc = (T[:, 3] - ox) / (1000 * sx), (T[:, 4] - oy) / (1000 * sy)
    xa, ya = (T[:, 5] - ox) / (1000 * sx), (T[:, 6] - oy) / (1000 * sy)
    state = T[:, 7].astype(int)
    run_t = (state == 3) & (master > lead)

    with args.log.open() as f:
        r = csv.reader(f)
        hdr = next(r)
        rows = [x for x in r if x[2] == "run"]
    col = {n: i for i, n in enumerate(hdr)}
    L = np.array([[float(x[col[n]]) for n in
                   ["mono_ns", "master_est", "r_fb_q1", "r_fb_q2", "r_fb_q3", "r_fb_q4", "r_fb_q5", "r_fb_q6"]]
                  for x in rows])
    l_mono, m_est, r_fb = L[:, 0], L[:, 1], np.radians(L[:, 2:8])
    m_true_at_log = np.interp(l_mono, t_mono, master)
    sel_log = (m_est >= 0) & (m_true_at_log > lead) & (m_true_at_log < master.max() - 0.05)

    res, verdict = {}, {}
    # 1. 시계 추정
    ce = (m_est[sel_log] - m_true_at_log[sel_log]) * 1e3
    # 일정한 치우침(상태값이 스캔 끝에 갱신돼 평균 반 주기 늦음 + 통신 지연)은 지연 보정값에 흡수되므로 따로 본다
    jit = ce - ce.mean()
    res["clock_err_ms"] = {"mean": float(ce.mean()), "p99_abs": float(np.percentile(np.abs(ce), 99)),
                           "max_abs": float(np.abs(ce).max()), "jitter_p99": float(np.percentile(np.abs(jit), 99)),
                           "drift_ms": float(np.polyfit(tl_all := (m_true_at_log[sel_log] - lead), ce, 1)[0] * (tl_all.max() - tl_all.min()))}
    verdict[f"PLC 시계 추정 흔들림(치우침 제외) p99 < {args.tol_clock_ms} ms"] = res["clock_err_ms"]["jitter_p99"] < args.tol_clock_ms

    # 2. 캠 충실도
    pc = plan(master[run_t] - lead)
    cam_err = np.hypot(xc[run_t] - pc[:, 0], yc[run_t] - pc[:, 1]) * 1e3
    res["cam_cmd_err_mm_max"] = float(cam_err.max())
    verdict[f"PLC 캠 지령 = 계획 (< {args.tol_cam_mm} mm)"] = res["cam_cmd_err_mm_max"] < args.tol_cam_mm

    # 3. 장비 지연
    tm_r = master[run_t] - lead

    def g_cost(d):
        p = plan(tm_r - d)
        return float(np.sqrt(np.mean((xa[run_t] - p[:, 0]) ** 2 + (ya[run_t] - p[:, 1]) ** 2)))

    tl = m_true_at_log[sel_log] - lead
    rf = r_fb[sel_log]

    def r_cost(d):
        p = plan(tl - d)
        return float(np.sqrt(np.mean((rf - p[:, 2:]) ** 2)))

    d_g, g_res = best_delay(g_cost)
    d_r, r_res = best_delay(r_cost)
    res["gantry_delay_ms"], res["robot_delay_ms"] = float(d_g), float(d_r)
    res["gantry_residual_mm"], res["robot_residual_deg"] = g_res * 1e3, math.degrees(r_res)
    res["sync_offset_ms"] = float(abs(d_g - d_r))
    verdict[f"갠트리-로봇 지연 차 < {args.tol_sync_ms} ms"] = res["sync_offset_ms"] < args.tol_sync_ms

    # 4. TCP
    import pinocchio as pin
    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    fid = model.getFrameId(args.tcp_frame)
    qidx = [model.joints[model.getJointId(a)].idx_q for a in AXES]

    def fk(q8):
        q = pin.neutral(model)
        for i, j in enumerate(qidx):
            q[j] = q8[i]
        pin.framesForwardKinematics(model, data, q)
        return data.oMf[fid].translation.copy()

    idx = np.flatnonzero(sel_log)[:: max(args.fk_stride, 1)]
    g_at = np.c_[np.interp(l_mono[idx], t_mono, xa), np.interp(l_mono[idx], t_mono, ya)]
    act8 = np.c_[g_at, r_fb[idx]]
    ref8 = plan(m_true_at_log[idx] - lead - d_g * 1e-3)
    err = np.array([np.linalg.norm(fk(a) - fk(b)) * 1e3 for a, b in zip(act8, ref8)])
    res["tcp_err_mm"] = {"rms": float(np.sqrt(np.mean(err ** 2))), "p99": float(np.percentile(err, 99)),
                         "max": float(err.max())}
    verdict[f"TCP 위치오차 p99 < {args.tol_tcp_mm} mm"] = res["tcp_err_mm"]["p99"] < args.tol_tcp_mm

    c = res["clock_err_ms"]
    print(f"[verify-plc] PLC 시계 추정: 치우침 {c['mean']:+.3f} ms, 흔들림 p99 {c['jitter_p99']:.3f} ms, "
          f"|오차| 최대 {c['max_abs']:.3f} ms, 전 구간 누적 경향 {c['drift_ms']:+.3f} ms")
    print(f"[verify-plc] PLC 캠 지령 vs 계획: 최대 {res['cam_cmd_err_mm_max']:.4f} mm")
    print(f"[verify-plc] 지연: 갠트리 {d_g:.1f} ms (잔차 {res['gantry_residual_mm']:.3f} mm), "
          f"로봇 {d_r:.1f} ms (잔차 {res['robot_residual_deg']:.4f} deg) → 동기 오차 {res['sync_offset_ms']:.1f} ms")
    t = res["tcp_err_mm"]
    print(f"[verify-plc] TCP 오차: rms {t['rms']:.3f} / p99 {t['p99']:.3f} / max {t['max']:.3f} mm")
    for k, ok in verdict.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        ax[0].plot(tl, ce, lw=0.6)
        ax[0].set_ylabel("clock est err [ms]")
        ax[1].plot(m_true_at_log[idx] - lead, err, lw=0.6)
        ax[1].set_ylabel("TCP err [mm]")
        ax[2].plot(tm_r, (xa[run_t] - plan(tm_r - d_g * 1e-3)[:, 0]) * 1e3, lw=0.6, label="gantry x (delay-comp)")
        ax[2].plot(tm_r, (ya[run_t] - plan(tm_r - d_g * 1e-3)[:, 1]) * 1e3, lw=0.6, label="gantry y")
        ax[2].set_ylabel("gantry act-plan [mm]")
        ax[2].set_xlabel("plan time [s]")
        ax[2].legend(fontsize=7)
        fig.suptitle(f"{args.log.parent.name}: delay g/r {d_g:.1f}/{d_r:.1f} ms")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
    return 0 if all(verdict.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
