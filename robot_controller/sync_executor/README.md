# sync_executor — 갠트리 + 로봇 시간 동기 실행기

플래닝한 8축 궤적(갠트리 X/Y + 두산 6축)을 **하나의 1 ms 루프**에서 같은 tick 으로 두 장비에 내보낸다.
별도 상위 제어기 없이 이 PC 가 직접 구동하는 구조다.

```
seq_*_joint.csv ──compile_motion.py──▶ motion.csv(1 ms, SI 단위) ──sync_executor──▶ 갠트리(CSP 목표+속도FF)
 (row 1 ms, 전이 10 ms)                 + motion.yaml(메타)                     └─▶ 로봇(speedj_rt)
                                                                                └─▶ synclog.csv ──verify_sync.py──▶ PASS/FAIL
```

| 파일 | 역할 |
|---|---|
| `scripts/compile_motion.py` | 세그먼트 CSV 들 → 1 ms 단일 타임라인. 이음새·단위·한계 점검 |
| `src/main.cpp` | 실행기. 사전위치 → 동시 시작 → 재생 → 정지 / 비상정지 |
| `include/sync_executor/backends.hpp` | 갠트리·로봇 인터페이스 + 모의(mock) 장비 |
| `src/drfl_robot_backend.cpp` | 두산 실기 백엔드 (DRFL RT). **컴파일만 확인, 실기 미검증** |
| `scripts/verify_sync.py` | 로그 검증: tick 연속, 명령=계획, 주기 지터, 장비별 지연, TCP 오차 |
| `scripts/export_gantry_plc.py` | motion → XMC 캠 데이터 (`_cam.csv/.meta`), 발표 형식 `_txy.csv`(T,X,Y,Vx,Vy,Ax,Ay), 보간오차·예산 보고 |
| `src/xgt_client.cpp`, `src/plc_gantry.cpp` | 갠트리 = XMC-E08A (XGT 전용 프로토콜). 캠 더블버퍼 공급, 상태 폴링, **PLC 시계 추정** |
| `PLC_INTERFACE.md` | **PLC 프로그래머용**: 메모리 맵, 명령 동작, XG5000 설정, 지연 보정, 실기 시험 순서 |
| `scripts/plc_sim.py`, `scripts/run_plc_sim_test.sh` | XMC 시뮬레이터 (XGT 서버 + 캠/가상축/버퍼/워치독, 시계 드리프트·서보 지연 주입) |
| `scripts/verify_plc_sync.py` | PLC 모드 검증: 시계 추정, 캠 지령=계획, 장비 지연, TCP 오차 |

## PLC 모드 (`--gantry plc`, 갠트리 = XMC-E08A)

갠트리 궤적은 PLC 가 **가상 마스터축(= 시간) + 캠**으로 만들고, 로봇이 PLC 마스터 시간을 따라간다 (구조·메모리 맵: `PLC_INTERFACE.md`).
PC 와 PLC 는 시계가 달라 (수정 발진기 ±수십 ppm → 100 s 에 수 ms) PC 가 상태 폴링(200 Hz)으로 마스터 시간 ↔ PC 시계를 계속 추정한다.

```bash
python scripts/export_gantry_plc.py motion.csv -o out/door          # → out/door_cam.csv/.meta/_txy.csv/_report.yaml
build/sync_executor --motion motion.csv --gantry plc --cam out/door_cam.csv --plc-ip <XMC IP> \
    --robot drfl --gantry-delay-ms <dg> --robot-delay-ms <dr>
# 시뮬레이터로 (로봇 mock):
EXE=build/sync_executor scripts/run_plc_sim_test.sh out/t motion.csv out/door_cam.csv \
    --x0 2450 --y0 2420 --drift-ppm 100 --servo-delay-ms 4 -- --gantry-delay-ms 3.1 --robot-delay-ms 5.7
python scripts/verify_plc_sync.py --motion motion.csv --cam-meta out/door_cam.meta \
    --truth out/t/truth.csv --log out/t/exec_log.csv --urdf <urdf>
```

검증 (2026-11-23, 시뮬레이터, 도어 t90 98.5 s, PLC 시계 +100 ppm, 서보 지연 4 ms):

| | 시계 추적 (기본) | `--plc-clock-fixed` (시작 때만 맞춤) |
|---|---|---|
| PLC 시간 추정 흔들림 p99 | 0.03 ms (일정 치우침 −0.68 ms) | 4.9 ms (끝에서 9.8 ms 누적) |
| 동기 오차 / TCP p99 | 0.9 ms / 0.42 mm → 보정 후 **0.1 ms / 0.15 mm** | 6.6 ms / 3.6 mm |
| 캠 지령 vs 계획 (10 ms 캠, 3차) | 0.006 mm | 0.006 mm |

- 45 세그먼트 버퍼 연결·DONE, Ctrl+C 시 PLC 마스터 감속 정지(IDLE) 확인. 1 ms 루프 지터 p99.9 6 µs.
- XGT 프레임은 FEnet 규격으로 구현했고 **실기 XMC 와는 아직 통신해 보지 않았다** (`PLC_INTERFACE.md` 6장 1단계).
- 시뮬레이터(Python) 스캔이 가끔 수 ms 멈춰 그 순간 시계 오차가 튄다 (전체 6 tick). 실기 PLC 와 무관.

## 빌드 (새 PC 에서)

필요한 것: CMake ≥ 3.16, C++17 컴파일러(g++), Python 3 + `numpy scipy pyyaml` (검증 스크립트는 추가로 `pinocchio matplotlib`).
외부 C++ 의존성은 API-DRFL 서브모듈뿐이다.

```bash
# 저장소 루트에서
git submodule update --init -- robot_controller/doosan_a0912_controller/API-DRFL   # DRFL 백엔드용
cmake -S robot_controller/sync_executor -B robot_controller/sync_executor/build
cmake --build robot_controller/sync_executor/build -j
```
- DRFL 라이브러리가 없으면 자동으로 mock 전용으로 빌드된다 (`SYNC_WITH_DRFL=OFF`). cmake 출력의 `DRFL backend ON/OFF` 로 확인.
  라이브러리는 `API-DRFL/library/Linux/64bits/<amd64|arm64>/<우분투 버전>/` 에서 고르므로 우분투 버전이 다르면 OFF 가 된다.
  제어기 펌웨어가 M3.x 면 `-DDOOSAN_DRCF_VERSION=3`.
- 실시간 권한: 1 ms 루프는 `SCHED_FIFO` + `mlockall` 을 쓴다. 권한이 없으면 경고만 내고 일반 스케줄로 돈다 (지터 커짐).
  `/etc/security/limits.conf` 에 `<user> - rtprio 99`, `<user> - memlock unlimited` 를 넣고 재로그인하거나 `sudo` 로 실행.
- 네트워크: 로봇 제어기 기본 `192.168.137.100` (`--robot-ip`), XMC 는 `--plc-ip`. 같은 서브넷에 있어야 한다.
- 입력 궤적: 플래너 결과 `motion_planning/log/<run>/transition/seq_*_joint.csv` 와 검증용 URDF(`tcp` 프레임이 있는 t90 8축 URDF)가 필요하다.

## 사용

```bash
# 1) 1 ms 타임라인 만들기 (paintcell env, PyYAML 은 시스템 dist-packages)
PYTHONPATH=/usr/lib/python3/dist-packages python scripts/compile_motion.py \
    ../../motion_planning/log/20261117_door_flat_z32_t90/transition -o /tmp/door_motion.csv

# 2) 모의 장비로 실행 (--no-sleep: 실시간 대기 없이 빠르게, 생략하면 실제 1 ms 주기)
build/sync_executor --motion /tmp/door_motion.csv --no-sleep --log /tmp/log.csv

# 3) 검증
python scripts/verify_sync.py --motion /tmp/door_motion.csv --log /tmp/log.csv \
    --urdf <paint_cell_8axis_x7y4z3_t90.urdf> --plot /tmp/log.png
```

## 제어식

- 갠트리: `pos = q[k+Lg]`, `vel_ff = qd[k+Lg]` (CSP)
- 로봇: `qd_cmd = qd[k+Lr] + Kp·(q[k] − q_act)` → 속도 클램프 → 가속도 클램프 → `speedj_rt`
  - lead(`Lr`, `Lg`)는 **피드포워드에만** 건다. 피드백까지 `q[k+L]` 과 비교하면 `Kp·L·dt·v` 만큼 치우친다.
  - RUN 은 `k = k_begin − max(Lr, Lg)` 부터 돌고 장비별 기준 인덱스는 `[k_begin, k_last]` 로 자른다.
    지연이 큰 장비가 먼저 출발해 실제 움직임이 동시에 시작된다 (첫 tick 에 lead 만큼 건너뛰는 점프 없음).
  - 시작·끝 tick 은 정지 상태여야 한다 (row 경계). 갠트리는 실행기에서 가속도 클램프가 없기 때문.

## 지연 보정 절차 (실기)

1. `--kp 0 --end-tick <row 하나 끝>` 으로 짧게 재생 → `verify_sync.py` 의 "장비 지연" 이 순수 지연이다.
   (Kp 를 켠 상태에서 재면 피드백이 지연을 일부 가려 과소 추정된다. mock: Kp=4 에서 1.5 tick, Kp=0 에서 6.5 tick)
2. `--robot-lead-ticks`, `--gantry-lead-ticks` 를 측정값(반올림)으로 주고 Kp 를 켜서 재생.
3. 검증에서 지연 차가 ±0.5 tick 안이면 동기 완료. 음수면 과보상.

## 안전 장치

- 사전위치: 로봇 10 °/s, 갠트리 0.05 m/s 로 첫 점까지 (허용 0.01°, 0.2 mm) → 둘 다 200 ms 정지 확인 후 같은 tick 에 RUN.
  **충돌 검사가 없으므로** 시작 자세에서 10° / 100 mm 넘게 떨어져 있으면 거부한다 (`--pre-max-deg/--pre-max-mm`).
- 비상정지(ABORT): Ctrl+C, 로봇 SAFE_STOP/EMERGENCY/SAFE_OFF/연결끊김, 로봇 추종오차 > 3°, 갠트리 > 5 mm,
  피드백 10 tick 연속 끊김(DRFL `time_stamp` 정지 포함), 송신 실패 10 tick →
  로봇은 직전 명령에서 `--max-qdd-deg` 로 0 까지, 갠트리는 **직전 송신 명령**에서 1.6 m/s² 로 감속.
- 종료(TAIL/ABORT): 명령 0 + **실측 속도 0** 이 100 ms 지속되어야 끝낸다 (최대 5 s). 그 뒤 stop_rt → servo_off → disconnect.
- `set_accj_rt` 는 `--max-qdd-deg`(기본 500) 와 같게 맞춘다. 기존 실기 코드는 70 이었으니 첫 시험에서 추종 확인.
- 2 주기 이상 늦으면 밀린 명령을 몰아 보내지 않고 기준 시각을 다시 잡는다.
- 로봇 첫 실기 시험은 갠트리 mock + 로봇 drfl 로, 짧은 row 하나부터.

## 검증 결과 (2026-11-23, mock, 도어 t90 전체 98.5 s)

| 항목 | 결과 |
|---|---|
| 실시간 주기 (SCHED_FIFO 80, 일반 커널 6.8 generic) | 1000.0 ± 1.2 µs, 지연 p99.9 8 µs, 0.5 ms 초과 0 회 |
| 같은 tick 안 갠트리→로봇 송신 간격 | 최대 0.4 µs |
| 명령 = 계획 | 갠트리 0 mm, 로봇 1.5e-5° (float32 로그 정밀도) |
| 지연 보상 전 (mock 갠트리 3 tick, 로봇 6.5 tick) | 동기 오차 1.5 ms, TCP p99 1.9 mm |
| 보상 후 (lead g3 / r6) | 동기 오차 0.1 ms, TCP p99 0.13 mm |
| 비상정지 (Ctrl+C / 추종오차 주입) | 갠트리 명령 감속 1.6 m/s², 로봇 500 °/s², 0 까지 정지 |

## 알려진 계획 데이터 문제 (실행기와 무관)

- 갠트리 가속도가 env 한계 1.6 m/s² 를 넘는다: row 안 1.8~3.7 m/s² (NLP 결과 qdd 그대로), 1 ms 재표본 후 최대 4.5 m/s².
- 전이(move, 10 ms) 구간의 `q` 와 `qd/qdd` 컬럼이 서로 안 맞는 곳이 있다 (예: q 차분 2.8 m/s² vs qdd 1.5 m/s²).
  그래서 `compile_motion.py` 는 위치 기준 C2 스플라인(`--interp spline`)으로 1 ms 를 만든다.
