#pragma once
// 두산 A0912 실기 백엔드: DRFL RT(UDP) 로 tick 마다 read_data_rt / speedj_rt.
// 연결 순서는 doosan_a0912_controller/test/src/doosan_controller.cpp (실기 검증된 경로) 와 같다.
// ※ 2026-11-23 현재 실기 미검증 (컴파일만 확인). 첫 실행은 로봇 단독·저속으로.

#include "sync_executor/backends.hpp"

#include <atomic>
#include <memory>
#include <string>

namespace sync_exec
{

struct DrflConfig
{
    std::string ip = "192.168.137.100";
    unsigned int port = 12345;
    std::string rt_ip;  // 비우면 ip
    unsigned int rt_port = 12347;
    float rt_period_s = 0.001f;
    int rt_loss_count = 4;
    Arr6f vel_limit_deg_s{70, 70, 70, 70, 70, 70};     // set_velj_rt
    // set_accj_rt. 실행기 가속도 클램프(--max-qdd-deg)와 같게 맞춘다 (main 에서 덮어씀).
    // 기존 실기 코드는 70 이었다. 계획 가속도는 최대 ~530 deg/s^2 라 70 이 실제로 걸리면 추종이 늦는다.
    Arr6f acc_limit_deg_s2{500, 500, 500, 500, 500, 500};
    int access_timeout_s = 15;
    bool servo_off_on_close = true;
};

class DrflRobotBackend final : public RobotBackend
{
public:
    explicit DrflRobotBackend(DrflConfig cfg);
    ~DrflRobotBackend() override;
    std::string name() const override { return "drfl-robot"; }
    bool open() override;
    bool read(RobotFeedback *fb) override;
    bool speedj(const Arr6f &qd_deg_s, const Arr6f &acc_deg_s2, float time_s) override;
    bool healthy(const char **reason) const override;
    void close() override;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace sync_exec
