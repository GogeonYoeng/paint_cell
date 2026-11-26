#pragma once
// XMC PLC 가 캠 동기로 갠트리를 돌리고, PC 는 데이터 공급 + 상태/시간 감시만 하는 백엔드.
//
//  - 통신 스레드(비실시간): 상태 블록 폴링(기본 200 Hz) → PLC 마스터 시간 추정, 캠 세그먼트 더블 버퍼 전송, 명령 핸드셰이크
//  - RT 루프는 read()/masterTimeAt() 로 최신 값만 가져간다 (seqlock, 블로킹 없음)
//  - write() 는 아무것도 하지 않는다 (궤적은 PLC 가 캠으로 직접 생성)
//
// 시간 기준: PLC 가상 마스터축 위치(= 시간 s) 가 전체 셀의 시계다. 로봇은 masterTimeAt(now) 로 그 시계를 따라간다.

#include "sync_executor/backends.hpp"
#include "sync_executor/xgt_client.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sync_exec
{

struct CamSegment
{
    std::vector<double> master, x, y;  // [s], [mm], [mm]
    double vx0 = 0, vx1 = 0, vy0 = 0, vy1 = 0;  // 끝점 기울기 [mm/s]
};

struct CamPlan
{
    double lead_in = 0.5;       // 마스터 0 ~ lead_in 동안 시작점 정지
    double t_plan0 = 0.0;       // master=lead_in 일 때의 계획 시간 [s]
    double x_sign = 1, x_offset_mm = 0, y_sign = 1, y_offset_mm = 0;  // PLC mm = sign*계획 m*1000 + offset
    std::vector<CamSegment> segs;
    double masterEnd() const { return segs.empty() ? 0 : segs.back().master.back(); }
};

bool loadCamPlan(const std::string &csv_path, const std::string &meta_path, CamPlan *plan, std::string *err);

struct PlcConfig
{
    std::string ip = "192.168.0.10";
    int port = 2004;
    uint32_t base_byte = 20000;  // %MB20000 = %MW10000
    uint32_t nmax = 1024;        // 슬롯당 배열 길이 (PLC 선언과 같아야 함)
    double poll_hz = 200;
    double prepos_vel_mmps = 50;
    double halt_decel = 2.0;     // 마스터 감속 [s/s^2] → 0.5 s 안에 정지
    bool clock_fixed = false;    // true: 시작 직후 오프셋만 잡고 이후 보정 안 함 (비교 시험용)
    double clock_window_s = 2.0;
    int cmd_timeout_ms = 3000;
    double watchdog_alive_hz = 20;
};

struct PlcStatus
{
    uint16_t ack_seq = 0, ack_code = 0, ack_err = 0, state = 0, active_seg = 0, queued_seg = 0xFFFF;
    uint16_t plc_alive = 0, axis_err = 0;
    double master_pos = 0, master_vel = 0, x_act = 0, y_act = 0, x_cmd = 0, y_cmd = 0;
    int64_t t_send_ns = 0, t_recv_ns = 0;  // CLOCK_MONOTONIC
};

class PlcGantry final : public GantryBackend
{
public:
    PlcGantry(PlcConfig cfg, CamPlan plan);
    ~PlcGantry() override;
    std::string name() const override { return "plc-gantry(XGT " + cfg_.ip + ")"; }

    // 연결 + 통신 스레드 시작 + 슬롯 0/1 사전 로드 + PREPOS 요청. 시작점 도착은 ready() 로 확인.
    bool open() override;
    bool read(GantryFeedback *fb) override;              // 계획 좌표(m) 로 변환한 최신 실측
    bool write(const Arr2 &, const Arr2 &) override { return true; }
    void close() override;

    // ---- RT 루프용 (논블로킹)
    bool ready() const;                     // PLC READY + 슬롯 0/1 적재 완료
    void requestStart();
    void requestStop();
    bool masterTimeAt(int64_t mono_ns, double *master) const;  // 시계 추정이 유효하면 true
    bool done() const;                      // PLC DONE
    bool failed(const char **why) const;    // 통신 끊김 / PLC ERROR / 캠 공급 지연
    PlcStatus status() const;
    const CamPlan &plan() const { return plan_; }

    // 통계
    double rttMaxUs() const { return rtt_max_us_.load(); }
    long polls() const { return polls_.load(); }

private:
    void commLoop();
    bool pollOnce();
    bool sendCommand(uint16_t code, const std::vector<std::pair<uint32_t, std::vector<uint8_t>>> &fields);
    bool waitAck(uint16_t seq, int timeout_ms);
    bool loadSegment(int seg);
    void updateClock(const PlcStatus &s);
    double toPlanX(double mm) const { return (mm - plan_.x_offset_mm) / (1000.0 * plan_.x_sign); }
    double toPlanY(double mm) const { return (mm - plan_.y_offset_mm) / (1000.0 * plan_.y_sign); }

    PlcConfig cfg_;
    CamPlan plan_;
    XgtClient xgt_;
    std::thread comm_;
    std::atomic_bool run_{false};
    std::atomic_bool start_req_{false}, stop_req_{false};
    std::atomic_bool started_{false}, loaded01_{false}, comm_fail_{false}, underrun_{false};
    std::atomic<long> polls_{0};
    std::atomic<double> rtt_max_us_{0};
    uint16_t seq_ = 0;
    int next_load_ = 0;  // 다음에 실을 세그먼트

    // 통신 스레드 ↔ RT 루프 공유 상태 (임계구역은 구조체 복사 수준으로 짧다)
    mutable std::mutex mtx_;
    PlcStatus st_{};
    bool clk_valid_ = false;
    double clk_a_ = 0, clk_b_ = 1;  // master = clk_a + clk_b * (t - clk_t0) [s]
    int64_t clk_t0_ = 0;
    double prev_x_cmd_ = 0, prev_y_cmd_ = 0, vel_x_ = 0, vel_y_ = 0;
    int64_t prev_t_ = 0;

    struct ClkSample
    {
        int64_t t_mid;
        double master;
        double rtt_us;
    };
    std::vector<ClkSample> clk_buf_;
};

}  // namespace sync_exec
