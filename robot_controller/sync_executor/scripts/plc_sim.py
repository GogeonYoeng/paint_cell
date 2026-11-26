#!/usr/bin/env python3
"""XMC PLC 시뮬레이터: XGT 전용 프로토콜 서버 + PLC_INTERFACE.md 의 PLC 프로그램 동작을 흉내 낸다.

실제 XMC 없이 sync_executor --gantry plc 를 TCP 로 끝까지 돌려 보기 위한 것.
  - XGT 서버: 연속 읽기(0x54)/쓰기(0x58), 변수명 %MB<주소>
  - PLC 스캔 1 ms: 명령 처리(PREPOS/LOAD/START/STOP/RESET), 가상 마스터축(시간), 캠 3차 보간, Buffered 연결,
    DONE/공급지연(ERROR)/PC 워치독
  - 고장 주입: --drift-ppm (PLC 시계가 PC 보다 빠르거나 느림), --servo-delay-ms (서보 추종 지연)
  - 정답 로그 (--truth-log): CLOCK_MONOTONIC 기준 mono_ns, master, x/y 지령·실측 → verify_plc_sync.py 가 사용

사용: plc_sim.py --port 12004 --truth-log truth.csv --drift-ppm 100 --servo-delay-ms 4
"""
from __future__ import annotations

import argparse
import math
import signal
import socketserver
import struct
import sys
import threading
import time

import numpy as np
from scipy.interpolate import CubicSpline

# ---- PLC_INTERFACE (plc_interface.hpp 와 같게)
CMD_SEQ, CMD_CODE, CMD_SLOT, CMD_SEG, CMD_NPTS, CMD_NSEG, PC_ALIVE = 0, 2, 4, 6, 8, 10, 14
CMD_SLOPE_X0, CMD_SLOPE_X1, CMD_SLOPE_Y0, CMD_SLOPE_Y1, CMD_PREPOS_VEL, CMD_HALT_DECEL = 16, 24, 32, 40, 48, 56
CAM_BASE = 64
ST_ACK_SEQ, ST_ACK_CODE, ST_ACK_ERR, ST_STATE, ST_ACTIVE_SEG, ST_QUEUED_SEG, ST_PLC_ALIVE, ST_AXIS_ERR = \
    0, 2, 4, 6, 8, 10, 12, 14
ST_MASTER_POS, ST_MASTER_VEL, ST_X_ACT, ST_Y_ACT, ST_X_CMD, ST_Y_CMD = 16, 24, 32, 40, 48, 56
IDLE, PREPOS, READY, RUNNING, STOPPING, DONE, ERROR = 0, 1, 2, 3, 4, 5, 9
C_PREPOS, C_LOAD, C_START, C_STOP, C_RESET = 1, 2, 3, 4, 5


class Plc:
    def __init__(self, args):
        self.a = args
        self.base = args.base_mb
        self.nmax = args.nmax
        self.st_base = self.base + CAM_BASE + 2 * 3 * self.nmax * 8
        self.mem = bytearray(self.st_base + 64 + 16)
        self.lock = threading.Lock()
        self.last_seq = 0
        self.state = IDLE
        self.tables = [None, None]  # (seg, master, csx, csy)
        self.active = 0
        self.queued = 0xFFFF
        self.nseg = 0
        self.master = 0.0
        self.mvel = 0.0
        self.mtarget = 0.0
        self.mdecel = 2.0
        self.x_cmd = self.y_cmd = 0.0
        self.x0, self.y0 = args.x0, args.y0
        self.x_cmd, self.y_cmd = self.x0, self.y0
        self.hist = []  # (mono_ns, x_cmd, y_cmd)
        self.alive = 0
        self.pc_alive_last = None
        self.pc_alive_t = time.monotonic()
        self.prepos = None
        self.truth = []
        self.done_t = None
        self.err = 0

    # -- 메모리 접근 (XGT 서버 스레드와 공유)
    def u16(self, off):
        return struct.unpack_from("<H", self.mem, self.base + off)[0]

    def f64(self, off):
        return struct.unpack_from("<d", self.mem, self.base + off)[0]

    def ack(self, seq, code, err=0):
        struct.pack_into("<HHH", self.mem, self.st_base + ST_ACK_SEQ, seq, code, err)

    def cam_array(self, slot, arr, n):
        off = self.base + CAM_BASE + ((slot * 3 + arr) * self.nmax) * 8
        return np.frombuffer(bytes(self.mem[off:off + 8 * n]), dtype="<f8").copy()

    # -- 명령
    def command(self):
        seq = self.u16(CMD_SEQ)
        if seq == self.last_seq:
            return
        self.last_seq = seq
        code = self.u16(CMD_CODE)
        err = 0
        if code == C_RESET:
            self.state, self.err = IDLE, 0
            self.mvel = 0.0
        elif code == C_LOAD:
            slot, seg, n = self.u16(CMD_SLOT), self.u16(CMD_SEG), self.u16(CMD_NPTS)
            if slot > 1 or n < 2 or n > self.nmax:
                err = 0x000B
            else:
                m, x, y = (self.cam_array(slot, i, n) for i in range(3))
                sx = ((1, self.f64(CMD_SLOPE_X0)), (1, self.f64(CMD_SLOPE_X1)))
                sy = ((1, self.f64(CMD_SLOPE_Y0)), (1, self.f64(CMD_SLOPE_Y1)))
                if not np.all(np.diff(m) > 0):
                    err = 0x111C
                else:
                    self.tables[slot] = (seg, m, CubicSpline(m, x, bc_type=sx), CubicSpline(m, y, bc_type=sy))
                    if self.state == RUNNING and seg == self.active + 1:
                        self.queued = seg  # MC_CamIn BufferMode=Buffered 예약
        elif code == C_PREPOS:
            if self.tables[0] is None:
                err = 0x0001
            else:
                _, m, csx, csy = self.tables[0]
                tx, ty = float(csx(m[0])), float(csy(m[0]))
                v = max(self.f64(CMD_PREPOS_VEL), 1.0)
                self.prepos = (self.x_cmd, self.y_cmd, tx, ty, v, time.monotonic())
                self.state = PREPOS
        elif code == C_START:
            if self.state != READY or self.tables[0] is None:
                err = 0x0002
            else:
                self.nseg = self.u16(CMD_NSEG)
                self.active = 0
                self.queued = 1 if (self.tables[1] is not None and self.tables[1][0] == 1) else 0xFFFF
                self.master, self.mvel, self.mtarget = 0.0, 0.0, 1.0
                self.state = RUNNING
        elif code == C_STOP:
            self.mdecel = max(self.f64(CMD_HALT_DECEL), 0.1)
            self.mtarget = 0.0
            if self.state == RUNNING:
                self.state = STOPPING
        else:
            err = 0x0003
        self.ack(seq, code, err)

    # -- 스캔
    def scan(self, dt_pc):
        dt = dt_pc * (1.0 + self.a.drift_ppm * 1e-6)  # PLC 시계로 잰 경과 시간
        with self.lock:
            self.command()
            alive = self.u16(PC_ALIVE)
            now = time.monotonic()
            if alive != self.pc_alive_last:
                self.pc_alive_last, self.pc_alive_t = alive, now
            if self.state in (RUNNING,) and now - self.pc_alive_t > self.a.watchdog_s:
                self.state, self.err = STOPPING, 0x0F01
                self.mtarget = 0.0
            if self.state == PREPOS:
                x0, y0, tx, ty, v, t0 = self.prepos
                dist = math.hypot(tx - x0, ty - y0)
                s = min(v * (now - t0), dist)
                f = s / dist if dist > 0 else 1.0
                self.x_cmd, self.y_cmd = x0 + f * (tx - x0), y0 + f * (ty - y0)
                if s >= dist:
                    self.state = READY
            if self.state in (RUNNING, STOPPING):
                # 가상 마스터축: 가속 1000 s/s^2 로 1.0 까지, 정지는 mdecel
                acc = 1000.0 if self.mtarget > self.mvel else self.mdecel
                dv = max(min(self.mtarget - self.mvel, acc * dt), -acc * dt)
                self.mvel += dv
                self.master += self.mvel * dt
                tab = self.tables[self.active % 2]
                if tab is None or tab[0] != self.active:
                    self.state, self.err = ERROR, 0x2001
                else:
                    seg, m, csx, csy = tab
                    if self.master >= m[-1]:
                        if self.active + 1 >= self.nseg:
                            self.master, self.mvel = m[-1], 0.0
                            self.x_cmd, self.y_cmd = float(csx(m[-1])), float(csy(m[-1]))
                            self.state = DONE
                        elif self.queued == self.active + 1:
                            self.active += 1
                            self.queued = 0xFFFF
                            seg, m, csx, csy = self.tables[self.active % 2]
                        else:
                            self.state, self.err = ERROR, 0x2002  # 공급 지연
                    if self.state in (RUNNING, STOPPING):
                        mm = min(self.master, m[-1])
                        self.x_cmd, self.y_cmd = float(csx(mm)), float(csy(mm))
                if self.state == STOPPING and self.mvel <= 0.0:
                    self.state = IDLE if self.err == 0 else ERROR
            # 서보: 지령을 servo_delay 만큼 늦게 따라감
            mono = time.monotonic_ns()
            self.hist.append((mono, self.x_cmd, self.y_cmd))
            target = mono - int(self.a.servo_delay_ms * 1e6)
            while len(self.hist) > 2 and self.hist[1][0] <= target:
                self.hist.pop(0)
            h0 = self.hist[0]
            if len(self.hist) > 1 and h0[0] <= target:
                h1 = self.hist[1]
                f = (target - h0[0]) / max(h1[0] - h0[0], 1)
                xa, ya = h0[1] + f * (h1[1] - h0[1]), h0[2] + f * (h1[2] - h0[2])
            else:
                xa, ya = h0[1], h0[2]
            self.alive = (self.alive + 1) & 0xFFFF
            struct.pack_into("<HHHHH", self.mem, self.st_base + ST_STATE, self.state, self.active, self.queued,
                             self.alive, self.err)
            struct.pack_into("<dddddd", self.mem, self.st_base + ST_MASTER_POS, self.master, self.mvel, xa, ya,
                             self.x_cmd, self.y_cmd)
            if self.a.truth_log:
                self.truth.append((mono, self.master, self.mvel, self.x_cmd, self.y_cmd, xa, ya, self.state,
                                   self.active))
            if self.state == DONE and self.done_t is None:
                self.done_t = now


def make_handler(plc: Plc):
    class H(socketserver.BaseRequestHandler):
        def recv_n(self, n):
            b = b""
            while len(b) < n:
                c = self.request.recv(n - len(b))
                if not c:
                    raise ConnectionError
                b += c
            return b

        def handle(self):
            try:
                while True:
                    h = self.recv_n(20)
                    if h[:8] != b"LSIS-XGT" or h[13] != 0x33:
                        return
                    inv, ln = struct.unpack_from("<HH", h, 14)
                    body = self.recv_n(ln)
                    cmd, typ, _, nblk, nlen = struct.unpack_from("<HHHHH", body, 0)
                    name = body[10:10 + nlen].decode()
                    p = 10 + nlen
                    size = struct.unpack_from("<H", body, p)[0]
                    if not name.startswith("%MB") or typ != 0x14 or nblk != 1:
                        resp = struct.pack("<HHHHH", cmd + 1, typ, 0, 0xFFFF, 0x0010)
                    else:
                        addr = int(name[3:])
                        with plc.lock:
                            if addr + size > len(plc.mem):
                                resp = struct.pack("<HHHHH", cmd + 1, typ, 0, 0xFFFF, 0x0011)
                            elif cmd == 0x54:
                                resp = struct.pack("<HHHHHH", 0x55, typ, 0, 0, 1, size) + bytes(plc.mem[addr:addr + size])
                            elif cmd == 0x58:
                                plc.mem[addr:addr + size] = body[p + 2:p + 2 + size]
                                resp = struct.pack("<HHHHH", 0x59, typ, 0, 0, 1)
                            else:
                                resp = struct.pack("<HHHHH", cmd + 1, typ, 0, 0xFFFF, 0x0012)
                    hdr = bytearray(b"LSIS-XGT\x00\x00" + struct.pack("<HBBHHB", 0, 0xA0, 0x11, inv, len(resp), 0))
                    hdr.append(sum(hdr) & 0xFF)
                    self.request.sendall(bytes(hdr) + resp)
            except (ConnectionError, OSError, struct.error):
                return

    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=12004)
    ap.add_argument("--base-mb", type=int, default=20000)
    ap.add_argument("--nmax", type=int, default=1024)
    ap.add_argument("--drift-ppm", type=float, default=0.0)
    ap.add_argument("--servo-delay-ms", type=float, default=0.0)
    ap.add_argument("--watchdog-s", type=float, default=0.3)
    ap.add_argument("--x0", type=float, default=0.0, help="초기 X [mm]")
    ap.add_argument("--y0", type=float, default=0.0)
    ap.add_argument("--truth-log", default="")
    ap.add_argument("--exit-after-done", type=float, default=2.0)
    ap.add_argument("--max-runtime", type=float, default=600.0)
    args = ap.parse_args(argv)

    plc = Plc(args)
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", args.port), make_handler(plc))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[plc_sim] XGT 서버 127.0.0.1:{args.port}, drift {args.drift_ppm} ppm, servo delay {args.servo_delay_ms} ms",
          flush=True)

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    period = 0.001
    t_prev = time.monotonic()
    t_next = t_prev
    t_start = t_prev
    while not stop["flag"]:
        t_next += period
        d = t_next - time.monotonic()
        if d > 0:
            time.sleep(d)
        now = time.monotonic()
        plc.scan(now - t_prev)
        t_prev = now
        if plc.done_t is not None and now - plc.done_t > args.exit_after_done:
            break
        if now - t_start > args.max_runtime:
            break
    srv.shutdown()
    if args.truth_log:
        with open(args.truth_log, "w") as f:
            f.write("mono_ns,master,master_vel,x_cmd,y_cmd,x_act,y_act,state,active_seg\n")
            for r in plc.truth:
                f.write(f"{r[0]},{r[1]:.9f},{r[2]:.6f},{r[3]:.6f},{r[4]:.6f},{r[5]:.6f},{r[6]:.6f},{r[7]},{r[8]}\n")
    print(f"[plc_sim] 종료 state={plc.state} active_seg={plc.active} err=0x{plc.err:04X} master={plc.master:.3f}",
          flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
