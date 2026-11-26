#!/usr/bin/env python3
"""1 ms 8축 타임라인(compile_motion.py 출력)에서 갠트리 X/Y 를 XMC PLC 캠 데이터로 내보낸다.

출력 (-o <prefix>):
  <prefix>_cam.csv    seg,i,master_s,x_mm,y_mm,vx_mmps,vy_mmps   ← sync_executor --cam 이 읽어 PLC 로 전송
  <prefix>_cam.meta   lead_in, t_plan0, 축 매핑, 세그먼트 수 (key=value)
  <prefix>_txy.csv    T,X,Y,Vx,Vy,Ax,Ay (mm, mm/s, mm/s^2) — 발표 자료의 PC→PLC 데이터 형식, 참고/디버그용
  <prefix>_report.yaml 포인트 예산, 3차 보간 오차, 한계 점검

캠 구성
  - 마스터 = PLC 가상축 위치 = 시간 [s]. 0 ~ lead_in 은 시작점 정지 구간 (PC 가 PLC 시계를 잡는 시간).
  - 세그먼트 i (row/전이 하나) = 캠 테이블 하나. 마스터 점은 1 ms 격자에서 cam_dt 마다 고른다.
    이웃 테이블은 경계 점을 공유한다 (앞 테이블 끝 = 뒤 테이블 시작).
  - 축 매핑: PLC_mm = sign * (계획 joint [m] * 1000) + offset_mm.  PLC 원점·방향이 계획(URDF joint)과 다르면 지정.
  - 펄스 환산(K = 32766.361682 pulse/mm)은 PLC 축 파라미터(단위 mm)에서 처리한다고 가정. 참고용으로 report 에만 적는다.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

K_PULSE_PER_MM = 524288 / (6.667 * 24 / 10)  # 모터 1회전 524,288 pulse / 16.0008 mm


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("motion", type=Path, help="compile_motion.py 출력 csv")
    ap.add_argument("-o", "--output", type=Path, required=True, help="출력 접두어")
    ap.add_argument("--cam-dt", type=float, default=0.010, help="캠 점 간격 [s] (1 ms 의 배수)")
    ap.add_argument("--lead-in", type=float, default=0.5, help="마스터 시작 후 정지 구간 [s]")
    ap.add_argument("--x-sign", type=float, default=1.0)
    ap.add_argument("--x-offset-mm", type=float, default=0.0)
    ap.add_argument("--y-sign", type=float, default=1.0)
    ap.add_argument("--y-offset-mm", type=float, default=0.0)
    ap.add_argument("--nmax", type=int, default=1024, help="PLC 캠 배열 길이 (슬롯당)")
    ap.add_argument("--total-points", type=int, default=8192, help="XMC-E08A 캠 총 포인트")
    ap.add_argument("--tables", type=int, default=4, help="쓰는 캠 테이블 수 (X/Y × 슬롯 2)")
    args = ap.parse_args(argv)

    d = np.loadtxt(args.motion, delimiter=",", skiprows=1)
    dt = float(d[1, 1] - d[0, 1])
    t, seg = d[:, 1], d[:, 2].astype(int)
    gx, gy = d[:, 3], d[:, 4]          # [m]
    vx, vy = d[:, 11], d[:, 12]        # [m/s]
    X = args.x_sign * gx * 1000 + args.x_offset_mm
    Y = args.y_sign * gy * 1000 + args.y_offset_mm
    VX, VY = args.x_sign * vx * 1000, args.y_sign * vy * 1000
    step = int(round(args.cam_dt / dt))
    if abs(step * dt - args.cam_dt) > 1e-9 or step < 1:
        raise SystemExit("--cam-dt 는 motion dt 의 정수배여야 한다")

    segs = np.unique(seg)
    starts = [int(np.flatnonzero(seg == s)[0]) for s in segs]
    bounds = starts + [len(t) - 1]  # 세그먼트 s 는 tick [bounds[s], bounds[s+1]] (끝점 공유)
    lead_in = args.lead_in
    rows, sizes, err_x, err_y = [], [], [], []
    from scipy.interpolate import CubicSpline

    for si, s in enumerate(segs):
        k0, k1 = bounds[si], bounds[si + 1]
        ks = list(range(k0, k1, step))
        if k1 - ks[-1] < max(step // 2, 1) and len(ks) > 1:
            ks[-1] = k1  # 마지막 간격이 너무 짧으면 직전 점을 끝점으로 대체
        else:
            ks.append(k1)
        master = lead_in + t[ks]
        xs, ys, vxs, vys = X[ks], Y[ks], VX[ks], VY[ks]
        if si == 0 and lead_in > 0:  # 정지 구간 앞에 붙이기
            pre = np.arange(0.0, lead_in - 1e-9, args.cam_dt)
            master = np.r_[pre, master]
            xs = np.r_[np.full(len(pre), X[k0]), xs]
            ys = np.r_[np.full(len(pre), Y[k0]), ys]
            vxs = np.r_[np.zeros(len(pre)), vxs]
            vys = np.r_[np.zeros(len(pre)), vys]
        sizes.append(len(master))
        for i in range(len(master)):
            rows.append((int(si), i, master[i], xs[i], ys[i], vxs[i], vys[i]))
        # 3차 보간(끝 기울기 고정) 을 1 ms 계획과 비교 — PLC 캠 곡선(Cubic) 근사
        sel = slice(k0, k1 + 1)
        tm = lead_in + t[sel]
        csx = CubicSpline(master, xs, bc_type=((1, vxs[0]), (1, vxs[-1])))
        csy = CubicSpline(master, ys, bc_type=((1, vys[0]), (1, vys[-1])))
        err_x.append(float(np.max(np.abs(csx(tm) - X[sel]))))
        err_y.append(float(np.max(np.abs(csy(tm) - Y[sel]))))

    out = args.output
    out.parent.mkdir(parents=True, exist_ok=True)
    cam_csv = Path(str(out) + "_cam.csv")
    with cam_csv.open("w") as f:
        f.write("seg,i,master_s,x_mm,y_mm,vx_mmps,vy_mmps\n")
        for r in rows:
            f.write(f"{r[0]},{r[1]},{r[2]:.6f},{r[3]:.6f},{r[4]:.6f},{r[5]:.6f},{r[6]:.6f}\n")
    meta = Path(str(out) + "_cam.meta")
    meta.write_text(
        "# export_gantry_plc.py\n"
        f"lead_in={lead_in}\nt_plan0={t[0]}\ncam_dt={args.cam_dt}\n"
        f"x_sign={args.x_sign}\nx_offset_mm={args.x_offset_mm}\ny_sign={args.y_sign}\ny_offset_mm={args.y_offset_mm}\n"
        f"n_segments={len(segs)}\nnmax={args.nmax}\n")

    # 발표 자료 형식 T,X,Y,Vx,Vy,Ax,Ay (cam_dt 간격, 전체 연속)
    kk = np.arange(0, len(t), step)
    AX = np.gradient(VX, dt)
    AY = np.gradient(VY, dt)
    txy = Path(str(out) + "_txy.csv")
    with txy.open("w") as f:
        f.write("T,X,Y,Vx,Vy,Ax,Ay\n")
        for k in kk:
            f.write(f"{t[k]:.4f},{X[k]:.4f},{Y[k]:.4f},{VX[k]:.4f},{VY[k]:.4f},{AX[k]:.2f},{AY[k]:.2f}\n")

    budget_need = 2 * 2 * max(sizes)  # X/Y × 슬롯 2, 가장 긴 세그먼트 기준
    report = {
        "segments": int(len(segs)),
        "duration_s": float(t[-1] - t[0]),
        "lead_in_s": lead_in,
        "master_end_s": float(rows[-1][2]),
        "cam_dt_s": args.cam_dt,
        "points_per_segment_max": int(max(sizes)),
        "points_per_segment_min": int(min(sizes)),
        "nmax": args.nmax,
        "cam_point_budget": {"needed_worst": budget_need, "available": args.total_points,
                             "ok": bool(max(sizes) <= args.nmax and args.tables * args.nmax <= args.total_points)},
        "cubic_interp_err_mm": {"x_max": max(err_x), "y_max": max(err_y)},
        "x_range_mm": [float(X.min()), float(X.max())],
        "y_range_mm": [float(Y.min()), float(Y.max())],
        "v_max_mmps": [float(np.abs(VX).max()), float(np.abs(VY).max())],
        "a_max_mmps2": [float(np.abs(AX).max()), float(np.abs(AY).max())],
        "pulse_per_mm": K_PULSE_PER_MM,
        "v_max_pulse_per_s": [float(np.abs(VX).max() * K_PULSE_PER_MM), float(np.abs(VY).max() * K_PULSE_PER_MM)],
    }
    import yaml
    Path(str(out) + "_report.yaml").write_text(yaml.safe_dump(report, sort_keys=False, allow_unicode=True))
    print(f"[export] 세그먼트 {len(segs)}, 마스터 0 ~ {report['master_end_s']:.3f} s (lead-in {lead_in} s), "
          f"세그먼트당 포인트 {min(sizes)}~{max(sizes)} (nmax {args.nmax})")
    print(f"[export] 10 ms 캠 3차 보간 vs 1 ms 계획: X {max(err_x):.4f} mm, Y {max(err_y):.4f} mm")
    print(f"[export] X {report['x_range_mm'][0]:.1f}~{report['x_range_mm'][1]:.1f} mm, "
          f"Y {report['y_range_mm'][0]:.1f}~{report['y_range_mm'][1]:.1f} mm, "
          f"v_max {report['v_max_mmps'][0]:.0f}/{report['v_max_mmps'][1]:.0f} mm/s, "
          f"a_max {report['a_max_mmps2'][0]:.0f}/{report['a_max_mmps2'][1]:.0f} mm/s²")
    if not report["cam_point_budget"]["ok"]:
        print("[export][error] 캠 포인트 예산 초과")
        return 1
    print(f"[export] -> {cam_csv}, {meta}, {txy}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
