#pragma once
// PC ↔ XMC 공유 메모리 맵 (PLC_INTERFACE.md 와 반드시 같게 유지). 모든 오프셋은 base(%MB) 기준 바이트.
// 정수는 UINT/WORD(2 B), 실수는 LREAL(8 B, IEEE754, 리틀엔디언).

#include <cstdint>

namespace sync_exec::plc
{

// ---- 명령 블록 (PC → PLC)
constexpr uint32_t CMD_SEQ = 0;         // UINT  PC 가 명령마다 +1 (다른 필드를 먼저 쓰고 마지막에 씀)
constexpr uint32_t CMD_CODE = 2;        // UINT  명령 코드 (아래)
constexpr uint32_t CMD_SLOT = 4;        // UINT  LOAD: 캠 슬롯 0/1
constexpr uint32_t CMD_SEG = 6;         // UINT  LOAD: 세그먼트 번호 (0부터)
constexpr uint32_t CMD_NPTS = 8;        // UINT  LOAD: 포인트 수
constexpr uint32_t CMD_NSEG = 10;       // UINT  START: 전체 세그먼트 수
constexpr uint32_t PC_ALIVE = 14;       // UINT  PC 가 주기적으로 +1 (PLC 워치독)
constexpr uint32_t CMD_SLOPE_X0 = 16;   // LREAL LOAD: X 시작 기울기 [mm/s]
constexpr uint32_t CMD_SLOPE_X1 = 24;   // LREAL LOAD: X 끝 기울기
constexpr uint32_t CMD_SLOPE_Y0 = 32;   // LREAL
constexpr uint32_t CMD_SLOPE_Y1 = 40;   // LREAL
constexpr uint32_t CMD_PREPOS_VEL = 48; // LREAL PREPOS: 이동 속도 [mm/s]
constexpr uint32_t CMD_HALT_DECEL = 56; // LREAL STOP: 마스터 감속 [s/s^2]
constexpr uint32_t CMD_BLOCK_SIZE = 64;

enum CmdCode : uint16_t
{
    CMD_NONE = 0,
    CMD_PREPOS = 1,  // X/Y 를 슬롯 0 첫 점으로 절대 이동 → 완료 시 ack
    CMD_LOAD = 2,    // CAM 배열 슬롯을 LS_WriteCamData 로 X/Y 캠 테이블에 씀 → (RUN 중이면) CamIn Buffered 예약 → ack
    CMD_START = 3,   // 슬롯0 CamIn(X,Y) + 마스터 MC_MoveVelocity(1.0 s/s) → ack
    CMD_STOP = 4,    // 마스터 MC_Halt(감속 CMD_HALT_DECEL) → 경로를 따라 같이 감속 → ack
    CMD_RESET = 5,   // 에러 해제, IDLE 로
};

// ---- 캠 배열 (PC → PLC): 슬롯 s, 배열 a(0=master,1=X,2=Y), 원소 i
constexpr uint32_t CAM_BASE = 64;
inline uint32_t camOffset(uint32_t nmax, int slot, int array, int i)
{
    return CAM_BASE + ((static_cast<uint32_t>(slot) * 3u + static_cast<uint32_t>(array)) * nmax +
                       static_cast<uint32_t>(i)) * 8u;
}
inline uint32_t statusBase(uint32_t nmax) { return CAM_BASE + 2u * 3u * nmax * 8u; }

// ---- 상태 블록 (PLC → PC), statusBase(nmax) 기준
constexpr uint32_t ST_ACK_SEQ = 0;      // UINT  처리 끝난 CMD_SEQ
constexpr uint32_t ST_ACK_CODE = 2;     // UINT
constexpr uint32_t ST_ACK_ERR = 4;      // WORD  0=정상, 아니면 FB ErrorID
constexpr uint32_t ST_STATE = 6;        // UINT  아래 State
constexpr uint32_t ST_ACTIVE_SEG = 8;   // UINT  현재 캠이 걸린 세그먼트 번호
constexpr uint32_t ST_QUEUED_SEG = 10;  // UINT  Buffered 로 예약된 다음 세그먼트 번호 (없으면 0xFFFF)
constexpr uint32_t ST_PLC_ALIVE = 12;   // UINT  PLC 스캔마다 +1
constexpr uint32_t ST_AXIS_ERR = 14;    // WORD  축 에러 코드
constexpr uint32_t ST_MASTER_POS = 16;  // LREAL 가상 마스터축 지령 위치 = 시간 [s]
constexpr uint32_t ST_MASTER_VEL = 24;  // LREAL [s/s]
constexpr uint32_t ST_X_ACT = 32;       // LREAL [mm] 실측
constexpr uint32_t ST_Y_ACT = 40;
constexpr uint32_t ST_X_CMD = 48;       // LREAL [mm] 지령
constexpr uint32_t ST_Y_CMD = 56;
constexpr uint32_t ST_BLOCK_SIZE = 64;

enum State : uint16_t
{
    ST_IDLE = 0,
    ST_PREPOS = 1,
    ST_READY = 2,    // 시작점 정지, 서보 ON
    ST_RUNNING = 3,
    ST_STOPPING = 4,
    ST_DONE = 5,     // 마지막 세그먼트 끝, 마스터 정지
    ST_ERROR = 9,
};

}  // namespace sync_exec::plc
