# PC ↔ XMC-E08A 인터페이스 (캠 동기 방식)

PC(`sync_executor --gantry plc`)가 갠트리 궤적을 **캠 테이블**로 PLC 에 넣고, PLC 는 **가상 마스터축(= 시간)** 에
X/Y 를 캠으로 묶어 돌린다. 로봇은 PC 가 PLC 마스터 시간을 읽어 그 시간에 맞춰 움직인다.
즉 **셀 전체의 시계 = PLC 가상 마스터축 위치 [s]**.

```
PC (sync_executor)                         XMC-E08A
 ├ XGT 전용 프로토콜 TCP 2004 ───────────▶ XGT 서버 (%MB 영역 읽기/쓰기)
 │   · 캠 배열(슬롯 0/1) 쓰기                 · LS_WriteCamData → 캠 테이블 X_s, Y_s
 │   · 명령 블록 쓰기 (seq 핸드셰이크)         · MC_CamIn(Master=가상축, Slave=X/Y, Buffered 연결)
 │   · 상태 블록 200 Hz 읽기                   · 가상축 MC_MoveVelocity(1.0 s/s) = 시간
 │       └ 마스터 시간 → PC 시계 대응 추정      · X1/X2 CCC (LS_CrossCoupledControlOn)
 └ DRFL RT 1 kHz ─────────▶ 두산 로봇
```

## 1. XG5000 설정

| 항목 | 값 |
|---|---|
| 로컬 이더넷 | XGT 서버 사용 (TCP 2004). PC 와 같은 서브넷 |
| 메인 태스크 / EtherCAT 주기 | 1 ms |
| 실축 단위 | mm. 펄스 환산 K = 524288 / 16.0008 = 32,766.36 pulse/mm (랙 피치 6.667 × 잇수 24 / 감속 10) |
| 가상 마스터축 | 전용 가상축 1개(E08A). 단위 "s" 로 쓴다 (1 unit = 1 s). 속도 1.0 = 실시간 |
| 캠 프로파일 | 4개: X_A, Y_A (슬롯 0), X_B, Y_B (슬롯 1). 각 1024 포인트 (E08A 총 8,192 이내), 곡선 Cubic |
| 좌표 | PLC mm = sign × 계획 joint[m] × 1000 + offset. 기본 sign=+1, offset=0 → **PLC 원점과 방향이 URDF gantry_x/gantry_y 조인트와 같아야 한다** (다르면 `export_gantry_plc.py --x-sign/--x-offset-mm` 으로 맞춤) |

## 2. 메모리 맵

기준 `base` = `%MB20000` (= `%MW10000`). 정수 UINT/WORD 2 B, 실수 LREAL 8 B (리틀엔디언). `NMAX` = 1024.
코드 정의: `include/sync_executor/plc_interface.hpp`.

### 2.1 명령 블록 (PC → PLC), base + 0

| 오프셋 | 타입 | 이름 | 내용 |
|---|---|---|---|
| 0 | UINT | CMD_SEQ | PC 가 명령마다 +1. **다른 필드를 모두 쓴 뒤 마지막에 쓴다** → PLC 는 이 값이 바뀌면 명령 1회 처리 |
| 2 | UINT | CMD_CODE | 1 PREPOS, 2 LOAD, 3 START, 4 STOP, 5 RESET |
| 4 | UINT | CMD_SLOT | LOAD: 슬롯 0/1 |
| 6 | UINT | CMD_SEG | LOAD: 세그먼트 번호 (0부터) |
| 8 | UINT | CMD_NPTS | LOAD: 포인트 수 (≤ NMAX) |
| 10 | UINT | CMD_NSEG | START: 전체 세그먼트 수 |
| 14 | UINT | PC_ALIVE | PC 가 20 Hz 로 +1. RUNNING 중 300 ms 이상 안 바뀌면 PLC 가 STOP (워치독) |
| 16/24 | LREAL | SLOPE_X0/X1 | LOAD: X 시작/끝 기울기 [mm/s] → LS_WriteCamData StartSlope/EndSlope |
| 32/40 | LREAL | SLOPE_Y0/Y1 | LOAD: Y 시작/끝 기울기 |
| 48 | LREAL | PREPOS_VEL | PREPOS: 이동 속도 [mm/s] |
| 56 | LREAL | HALT_DECEL | STOP: 마스터 감속 [s/s²] |

### 2.2 캠 배열 (PC → PLC), base + 64

`LREAL CAM[slot 0..1][array 0..2][0..NMAX-1]`, array 0 = MasterPoint [s], 1 = X [mm], 2 = Y [mm].
바이트 오프셋 = `64 + ((slot*3 + array)*NMAX + i) * 8`. 슬롯 하나 = 24,576 B.

### 2.3 상태 블록 (PLC → PC), base + 64 + 2·3·NMAX·8 (= base + 49216)

| 오프셋 | 타입 | 이름 | 내용 |
|---|---|---|---|
| 0 | UINT | ACK_SEQ | 처리 끝난 CMD_SEQ (처리 후 마지막에 씀) |
| 2 | UINT | ACK_CODE | 처리한 명령 코드 |
| 4 | WORD | ACK_ERR | 0 정상, 아니면 FB ErrorID |
| 6 | UINT | STATE | 0 IDLE, 1 PREPOS, 2 READY, 3 RUNNING, 4 STOPPING, 5 DONE, 9 ERROR |
| 8 | UINT | ACTIVE_SEG | 지금 캠이 걸린 세그먼트 |
| 10 | UINT | QUEUED_SEG | Buffered 로 예약된 다음 세그먼트 (없으면 16#FFFF) |
| 12 | UINT | PLC_ALIVE | 스캔마다 +1 |
| 14 | WORD | AXIS_ERR | 축 에러 |
| 16 | LREAL | MASTER_POS | 가상축 **지령** 위치 [s] (MC_ReadCommandedPosition) |
| 24 | LREAL | MASTER_VEL | 가상축 지령 속도 [s/s] |
| 32/40 | LREAL | X_ACT/Y_ACT | 실측 위치 [mm] (X 는 X1 기준) |
| 48/56 | LREAL | X_CMD/Y_CMD | 지령 위치 [mm] |

**같은 스캔에서** MASTER_POS 와 X/Y 를 함께 갱신할 것 (PC 가 이 둘의 대응으로 갠트리 지연을 잰다).

## 3. 명령 동작

| 명령 | PLC 동작 | ACK 시점 | STATE |
|---|---|---|---|
| RESET | MC_Reset, 내부 변수 초기화 | 즉시 | → IDLE |
| LOAD | 슬롯 s 배열로 `LS_WriteCamData`(X_s, Y_s, CamPointNum=NPTS, Slope, CamCurveSel 전부 1=Cubic, ExecutionMode 0). RUNNING 이고 SEG = ACTIVE_SEG+1 이면 `MC_CamIn(X/Y, CamTable=슬롯 s, BufferMode=Buffered)` 예약 → QUEUED_SEG=SEG | WriteCamData Done (+CamIn 예약) 후 | 유지 |
| PREPOS | X/Y 를 슬롯 0 첫 점(CAM[0][1][0], CAM[0][2][0])으로 MC_MoveAbsolute(PREPOS_VEL) | 명령 접수 즉시 | PREPOS → 도착 시 READY |
| START | (READY 에서만) 가상축 위치 0, `MC_CamIn(X/Y, 슬롯 0, Master=가상축, StartMode=0 절대, MasterValueSource=0 지령)`, 슬롯 1 에 SEG 1 이 있으면 Buffered 예약, 가상축 `MC_MoveVelocity(1.0, Acc 1000)` | 접수 즉시 | → RUNNING |
| STOP | 가상축 `MC_Halt(Decel=HALT_DECEL)` → X/Y 는 캠을 따라 **경로 위에서** 같이 감속 | 접수 즉시 | → STOPPING → 정지 후 IDLE |

RUNNING 중 PLC 가 할 일:
- 캠 하나가 끝나면 (EndOfProfile) 예약된 다음 캠이 이어진다 → `ACTIVE_SEG := ACTIVE_SEG + 1`, `QUEUED_SEG := 16#FFFF`.
- 마지막 세그먼트(NSEG-1) 가 끝나면 가상축 정지 → DONE.
- 캠이 끝났는데 다음이 예약 안 됐으면 → 가상축 Halt, ERROR (AXIS_ERR=16#2002). PC 도 끝 100 ms 전에 미리 감지해 비상정지한다.
- PC_ALIVE 워치독 → STOP, ERROR.

세그먼트 n 은 슬롯 n mod 2 를 쓴다. PC 는 ACTIVE_SEG 가 n−1 이 된 뒤(= 슬롯의 이전 사용자 n−2 가 끝난 뒤)에 n 을 LOAD 한다.
가장 짧은 세그먼트가 0.56 s 라 LOAD(배열 ~12 KB 전송 + WriteCamData)가 그 안에 끝나야 한다.

## 4. X 이중 구동 (CCC)

X1/X2 가 한 축을 같이 미는 구조라면 X2 도 같은 캠 테이블을 따르게 하는 것이 가장 단순하다
(MC_CamIn 을 X1, X2 에 같은 테이블로 각각 실행, 또는 X2 를 X1 에 MC_GearIn 1:1). `LS_CrossCoupledControlOn(X1, X2)` 는 축이 정지한 상태에서
미리 켜 둔다 (실행 중에는 켤 수 없음, 매뉴얼 6.6.37). 상태 블록의 X_ACT/X_CMD 는 X1 기준으로 준다.

## 5. 지연 보정 (로봇 ↔ 갠트리)

- 갠트리 실측은 마스터 지령보다 `Dg` 늦고 (EtherCAT 2~3 주기 + 서보 추종), 로봇 실측은 명령보다 `Dr` 늦다.
- 실행기: 로봇 ff = plan(τ − Dg + Dr), fb 목표 = plan(τ − Dg), τ = 마스터 시간 − lead_in.
- PC 의 마스터 시간 추정에는 일정한 치우침이 있다 (상태값이 스캔 끝에 갱신돼 평균 반 주기 + 통신). 이것도 Dr 에 흡수된다.
- 절차:
  1. `--gantry-delay-ms 0 --robot-delay-ms 0` 으로 한 번 돌림 → `verify_plc_sync.py` 가 갠트리 지연 `dg`, 로봇 지연 `dr` 을 준다.
  2. `--robot-delay-ms` = 로봇 명령→실측 지연 (Kp=0 로 잰 값, mock 6.5 ms), `--gantry-delay-ms` = `dg`.
  3. 다시 돌려 `dr − dg` (동기 오차)가 남으면 **`--gantry-delay-ms` 를 그만큼 줄인다** (로봇이 늦으면 줄이고 빠르면 늘림).
     피드백 목표가 `τ − Dg` 라서 로봇 실측 위치를 정하는 것은 이 값이다. 시뮬레이터: 4.0 → 3.1 ms 로 동기 오차 0.9 → 0.1 ms.

## 6. 실기 첫 시험 순서

1. **통신**: PC 에서 `%MW10000` 근처를 읽고/쓰기 (XG5000 모니터로 확인). XGT 프레임 규격 검증이 목적.
2. **캠 적재만**: 서보 OFF 상태에서 LOAD 2회 → XG5000 캠 프로파일 뷰로 데이터 확인.
3. **PREPOS**: 저속(10 mm/s)으로 시작점 이동.
4. **갠트리 단독 RUN**: 로봇 없이 (`--robot mock`) 짧은 계획(세그먼트 2~3 개)으로 START → 상태 로그로 캠 연결·DONE 확인.
5. **로봇 + 갠트리**: 지연 보정 절차 (5장).
