#!/usr/bin/env python3
"""sync_executor 로그 검증: 갠트리·로봇 명령이 같은 tick 에 계획대로 나갔는지, 실측이 얼마나 어긋났는지.

검사 항목
  1. tick 무결성   RUN 구간의 k 가 빠짐/중복 없이 연속인가 (갠트리·로봇이 같은 k 를 받았다 = 한 행에 기록)
  2. 명령 충실도   갠트리 목표 = 계획 q[k+Lg], 로봇 기준 = 계획 q[k+Lr] (단위 변환 오류 검출)
  3. 실시간 주기   tick 간격 지터, 지연 p99/최대, 0.5 ms 초과 횟수, 같은 tick 안 갠트리→로봇 송신 간격
  4. 장비 지연     실측이 계획을 몇 tick 늦게 따라가는가 (갠트리 / 로봇 각각) → 둘의 차 = 동기 오차
  5. TCP 오차      FK(계획 8축) vs FK(갠트리 실측, 로봇 실측) [mm, deg]

사용
  verify_sync.py --motion motion.csv --log synclog.csv --urdf paint_cell_8axis_x7y4z3_t90.urdf
                 [--robot-lead-ticks 0 --gantry-lead-ticks 0] [--plot out.png] [--json out.json]
실기 로그에도 그대로 쓴다 (mock 은 장비 모델이 단순할 뿐 로그 형식은 같다).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

AXES = ["gantry_x", "gantry_y", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def load_csv(path: Path):
    with path.open() as f:
        header = f.readline().strip().split(",")
    return header, path


def load_motion(path: Path):
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    return d[:, 3:11], d[:, 11:19]  # q, qd


def load_log(path: Path):
    import csv
    with path.open() as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)
    col = {n: i for i, n in enumerate(header)}
    phase = np.array([r[col["phase"]] for r in rows])
    num = np.array([[float(r[i]) if j != col["phase"] else 0.0 for j, i in enumerate(range(len(header)))]
                    for r in rows])
    get = lambda n: num[:, col[n]]
    return {
        "phase": phase,
        "tick": get("tick").astype(np.int64),
        "k": get("k").astype(np.int64),
        "wake_ns": get("wake_ns"),
        "late_ns": get("late_ns"),
        "skew_ns": get("robot_write_ns") - get("gantry_write_ns"),
        "g_cmd": np.c_[get("g_cmd_x"), get("g_cmd_y")],
        "g_fb": np.c_[get("g_fb_x"), get("g_fb_y")],
        "r_ref": np.c_[[get(f"r_ref{i}") for i in range(1, 7)]].T,
        "r_cmd": np.c_[[get(f"r_cmd_qd{i}") for i in range(1, 7)]].T,
        "r_fb": np.c_[[get(f"r_fb_q{i}") for i in range(1, 7)]].T,
    }


def estimate_delay(ref: np.ndarray, act: np.ndarray, max_shift: int):
    """act[k] ≈ ref[k - s] 가 되는 s (tick). 0.1 tick 해상도 (선형 보간). 음수 = 실측이 계획보다 앞섬(lead 과보상)."""
    best = (math.inf, 0.0)
    n = len(ref)
    idx = np.arange(n, dtype=float)
    for s10 in range(-max_shift * 10, max_shift * 10 + 1):
        s = s10 / 10.0
        src = idx - s
        valid = (src >= 0) & (src <= n - 1)
        shifted = np.empty_like(ref[valid])
        for j in range(ref.shape[1]):
            shifted[:, j] = np.interp(src[valid], idx, ref[:, j])
        err = np.sqrt(np.mean((act[valid] - shifted) ** 2))
        if err < best[0]:
            best = (err, s)
    return best[1], best[0]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motion", type=Path, required=True)
    ap.add_argument("--log", type=Path, required=True)
    ap.add_argument("--urdf", type=Path, required=True)
    ap.add_argument("--tcp-frame", default="tcp")
    ap.add_argument("--robot-lead-ticks", type=int, default=0)
    ap.add_argument("--gantry-lead-ticks", type=int, default=0)
    ap.add_argument("--fk-stride", type=int, default=1, help="TCP FK 계산 간격 (tick)")
    ap.add_argument("--max-delay-ticks", type=int, default=30)
    ap.add_argument("--tol-cmd", type=float, default=1e-4, help="명령 충실도 허용 [deg|mm]")
    ap.add_argument("--tol-sync-ms", type=float, default=1.0, help="갠트리-로봇 지연 차 허용 [ms]")
    ap.add_argument("--tol-tcp-mm", type=float, default=1.0, help="TCP 위치 오차 허용 [mm]")
    ap.add_argument("--tol-late-us", type=float, default=200.0, help="실시간 지연 p99.9 허용 [us]")
    ap.add_argument("--plot", type=Path)
    ap.add_argument("--json", type=Path)
    args = ap.parse_args(argv)

    q_plan, _ = load_motion(args.motion)
    L = load_log(args.log)
    run = L["phase"] == "run"
    n_plan = len(q_plan)
    k_raw = L["k"][run]
    # RUN 은 k_begin - max(lead) 에서 시작한다 (그 사이 계획은 시작점 정지) → 계획 인덱스로 쓸 때는 자른다
    k_begin = int(k_raw[0]) + max(args.robot_lead_ticks, args.gantry_lead_ticks) if len(k_raw) else 0
    k = np.clip(k_raw, k_begin, n_plan - 1)
    dt = 1e-3
    results, verdict = {}, {}

    # ---- 1. tick 무결성
    k_ok = len(k_raw) > 1 and np.all(np.diff(k_raw) == 1)
    tick_ok = np.all(np.diff(L["tick"][run]) == 1)
    results["run_ticks"] = int(len(k))
    results["k_range"] = [int(k_raw[0]), int(k_raw[-1])] if len(k_raw) else None
    verdict["tick 연속 (k 빠짐·중복 없음)"] = bool(k_ok and tick_ok)

    # ---- 2. 명령 충실도
    kg = np.clip(k_raw + args.gantry_lead_ticks, k_begin, n_plan - 1)
    kr = np.clip(k_raw + args.robot_lead_ticks, k_begin, n_plan - 1)
    g_cmd_err_mm = np.max(np.abs(L["g_cmd"][run] - q_plan[kg, :2])) * 1e3
    r_ref_err_deg = np.max(np.abs(L["r_ref"][run] - np.degrees(q_plan[kr, 2:])))
    results["gantry_cmd_vs_plan_max_mm"] = float(g_cmd_err_mm)
    results["robot_ref_vs_plan_max_deg"] = float(r_ref_err_deg)
    # float32 로그 정밀도 (deg ~200 에서 1.5e-5) 감안
    verdict["갠트리 명령 = 계획"] = bool(g_cmd_err_mm < max(args.tol_cmd, 1e-3))
    verdict["로봇 기준 = 계획 (rad→deg)"] = bool(r_ref_err_deg < max(args.tol_cmd, 1e-4))

    # ---- 3. 실시간 주기
    realtime = np.any(L["late_ns"] != 0)
    if realtime:
        per = np.diff(L["wake_ns"]) / 1e3
        late = L["late_ns"] / 1e3
        skew = L["skew_ns"] / 1e3
        results["period_us"] = {"mean": float(per.mean()), "std": float(per.std()), "min": float(per.min()),
                                "max": float(per.max())}
        results["late_us"] = {"p99": float(np.percentile(late, 99)), "p99_9": float(np.percentile(late, 99.9)),
                              "max": float(late.max()), "over_500us": int(np.sum(late > 500))}
        results["same_tick_skew_us"] = {"p99": float(np.percentile(skew, 99)), "max": float(skew.max())}
        verdict[f"실시간 지연 p99.9 < {args.tol_late_us:.0f} us"] = bool(
            results["late_us"]["p99_9"] < args.tol_late_us)
    else:
        results["period_us"] = "no-sleep 시뮬레이션 (주기 검사 생략)"

    # ---- 4. 장비 지연 (실측이 계획 대비 몇 tick 늦나)
    plan_run = q_plan[k]
    g_delay, g_rms = estimate_delay(plan_run[:, :2] * 1e3, L["g_fb"][run] * 1e3, args.max_delay_ticks)
    r_delay, r_rms = estimate_delay(np.degrees(plan_run[:, 2:]), L["r_fb"][run], args.max_delay_ticks)
    results["gantry_delay_ticks"] = g_delay
    results["robot_delay_ticks"] = r_delay
    results["gantry_residual_rms_mm"] = float(g_rms)
    results["robot_residual_rms_deg"] = float(r_rms)
    sync_ms = abs(g_delay - r_delay) * dt * 1e3
    results["sync_offset_ms"] = sync_ms
    verdict[f"갠트리-로봇 지연 차 < {args.tol_sync_ms} ms"] = bool(sync_ms < args.tol_sync_ms)

    # ---- 5. TCP 오차
    import pinocchio as pin
    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    fid = model.getFrameId(args.tcp_frame)
    if fid >= model.nframes:
        raise SystemExit(f"프레임 없음: {args.tcp_frame}")
    qidx = [model.joints[model.getJointId(a)].idx_q for a in AXES]

    def fk(q8):
        q = pin.neutral(model)
        for i, j in enumerate(qidx):
            q[j] = q8[i]
        pin.framesForwardKinematics(model, data, q)
        M = data.oMf[fid]
        return M.translation.copy(), M.rotation.copy()

    sel = np.arange(0, len(k), max(args.fk_stride, 1))
    act8 = np.c_[L["g_fb"][run], np.radians(L["r_fb"][run])]
    pos_err, ang_err = np.empty(len(sel)), np.empty(len(sel))
    for n, i in enumerate(sel):
        p0, R0 = fk(plan_run[i])
        p1, R1 = fk(act8[i])
        pos_err[n] = np.linalg.norm(p1 - p0) * 1e3
        ang_err[n] = np.degrees(np.linalg.norm(pin.log3(R0.T @ R1)))
    results["tcp_pos_err_mm"] = {"rms": float(np.sqrt(np.mean(pos_err ** 2))),
                                 "p99": float(np.percentile(pos_err, 99)), "max": float(pos_err.max()),
                                 "max_at_k": int(k[sel[int(np.argmax(pos_err))]])}
    results["tcp_ang_err_deg"] = {"rms": float(np.sqrt(np.mean(ang_err ** 2))), "max": float(ang_err.max())}
    verdict[f"TCP 위치오차 p99 < {args.tol_tcp_mm} mm"] = bool(results["tcp_pos_err_mm"]["p99"] < args.tol_tcp_mm)

    # ---- 출력
    print(f"[verify] log {args.log.name}: RUN {results['run_ticks']} tick, k {results['k_range']}")
    print(f"[verify] 명령 충실도: 갠트리 {g_cmd_err_mm:.2e} mm, 로봇 {r_ref_err_deg:.2e} deg")
    if realtime:
        p, l, s = results["period_us"], results["late_us"], results["same_tick_skew_us"]
        print(f"[verify] 주기 {p['mean']:.1f}±{p['std']:.1f} us (min {p['min']:.0f}, max {p['max']:.0f}) | "
              f"지연 p99 {l['p99']:.0f} / p99.9 {l['p99_9']:.0f} / max {l['max']:.0f} us, >0.5ms {l['over_500us']}회 | "
              f"같은 tick 송신 간격 max {s['max']:.1f} us")
    print(f"[verify] 장비 지연: 갠트리 {g_delay:.1f} tick (잔차 {g_rms:.3f} mm), 로봇 {r_delay:.1f} tick "
          f"(잔차 {r_rms:.4f} deg) → 동기 오차 {sync_ms:.2f} ms")
    t = results["tcp_pos_err_mm"]
    print(f"[verify] TCP 오차: rms {t['rms']:.3f} / p99 {t['p99']:.3f} / max {t['max']:.3f} mm (k={t['max_at_k']}), "
          f"자세 max {results['tcp_ang_err_deg']['max']:.3f} deg")
    for name, ok in verdict.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    results["verdict"] = verdict

    if args.json:
        args.json.write_text(json.dumps(results, indent=2, ensure_ascii=False))
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        tk = k[sel] * dt
        fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        ax[0].plot(tk, pos_err, lw=0.7)
        ax[0].set_ylabel("TCP pos err [mm]")
        ax[1].plot(k * dt, (L["g_fb"][run] - plan_run[:, :2]) * 1e3, lw=0.7)
        ax[1].set_ylabel("gantry fb-plan [mm]")
        ax[1].legend(["x", "y"], loc="upper right")
        ax[2].plot(k * dt, L["r_fb"][run] - np.degrees(plan_run[:, 2:]), lw=0.7)
        ax[2].set_ylabel("robot fb-plan [deg]")
        ax[2].set_xlabel("t [s]")
        ax[2].legend([f"j{i}" for i in range(1, 7)], loc="upper right", ncol=6, fontsize=7)
        fig.suptitle(f"{args.log.name}  delay g/r = {g_delay:.1f}/{r_delay:.1f} tick")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
    return 0 if all(verdict.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
