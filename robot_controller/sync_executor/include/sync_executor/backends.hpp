#pragma once
// 갠트리 / 로봇 출력 인터페이스와 모의(mock) 구현.
// 실행기는 1 ms tick 마다 read() → 제어 계산 → write() 를 두 백엔드에 같은 tick 안에서 호출한다.
// 실제 장비 백엔드: 로봇 = DrflRobotBackend (drfl_robot_backend.hpp), 갠트리 = 드라이브 기종 확정 후 EtherCAT 구현.

#include <array>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace sync_exec
{

using Arr2 = std::array<double, 2>;
using Arr6f = std::array<float, 6>;

struct GantryFeedback
{
    Arr2 pos_m{};
    Arr2 vel_mps{};
    bool valid = false;
};

class GantryBackend
{
public:
    virtual ~GantryBackend() = default;
    virtual std::string name() const = 0;
    virtual bool open() = 0;
    virtual bool read(GantryFeedback *fb) = 0;
    // CSP 목표 위치 + 속도 피드포워드. 이 tick 에 드라이브로 나갈 값.
    virtual bool write(const Arr2 &pos_m, const Arr2 &vel_ff_mps) = 0;
    virtual void close() = 0;
};

struct RobotFeedback
{
    Arr6f q_deg{};
    Arr6f qd_deg_s{};
    double time_stamp = 0.0;  // 로봇 컨트롤러 RT 데이터 time_stamp (mock 은 시뮬레이션 시간)
    bool valid = false;
};

class RobotBackend
{
public:
    virtual ~RobotBackend() = default;
    virtual std::string name() const = 0;
    virtual bool open() = 0;      // 연결·서보온·RT 시작까지
    virtual bool read(RobotFeedback *fb) = 0;
    virtual bool speedj(const Arr6f &qd_deg_s, const Arr6f &acc_deg_s2, float time_s) = 0;
    // 보호정지/비상정지/연결끊김이면 false. reason 은 고정 문자열 (RT 루프에서 할당 없음).
    virtual bool healthy(const char **reason) const
    {
        (void)reason;
        return true;
    }
    virtual void close() = 0;     // RT 정지·서보오프·연결 해제
};

// ---------------------------------------------------------------- mock

// EtherCAT CSP 드라이브 근사: 목표 위치가 delay tick 뒤에 반영되고 1차 지연 tau 로 따라간다.
class MockGantry final : public GantryBackend
{
public:
    MockGantry(Arr2 initial, int delay_ticks, double tau_s, double dt)
        : pos_(initial), delay_(std::max(delay_ticks, 0)), tau_(tau_s), dt_(dt),
          buffer_(static_cast<std::size_t>(delay_ + 1), initial)
    {
    }
    std::string name() const override { return "mock-gantry"; }
    bool open() override { return true; }
    bool read(GantryFeedback *fb) override
    {
        fb->pos_m = pos_;
        fb->vel_mps = vel_;
        fb->valid = true;
        return true;
    }
    bool write(const Arr2 &pos_m, const Arr2 &) override
    {
        buffer_[head_] = pos_m;
        head_ = (head_ + 1) % buffer_.size();
        const Arr2 &target = buffer_[head_];  // delay_ tick 전 명령
        for (int i = 0; i < 2; ++i)
        {
            const double prev = pos_[i];
            if (tau_ <= 0.0)
                pos_[i] = target[i];
            else
                pos_[i] += (target[i] - pos_[i]) * std::min(dt_ / tau_, 1.0);
            vel_[i] = (pos_[i] - prev) / dt_;
        }
        return true;
    }
    void close() override {}

private:
    Arr2 pos_{}, vel_{};
    int delay_;
    double tau_, dt_;
    std::vector<Arr2> buffer_;
    std::size_t head_ = 0;
};

// 두산 speedj_rt 근사: 속도 명령이 delay tick 뒤 반영, 1차 지연 tau, 적분해 위치. 선택적 측정 잡음.
class MockRobot final : public RobotBackend
{
public:
    MockRobot(Arr6f initial_deg, int delay_ticks, double tau_s, double noise_deg, double dt)
        : q_(initial_deg), delay_(std::max(delay_ticks, 0)), tau_(tau_s), noise_(noise_deg), dt_(dt),
          buffer_(static_cast<std::size_t>(delay_ + 1), Arr6f{}), rng_(12345), gauss_(0.0, 1.0)
    {
    }
    std::string name() const override { return "mock-robot"; }
    bool open() override { return true; }
    bool read(RobotFeedback *fb) override
    {
        for (int i = 0; i < 6; ++i)
        {
            fb->q_deg[i] = static_cast<float>(q_[i] + (noise_ > 0 ? noise_ * gauss_(rng_) : 0.0));
            fb->qd_deg_s[i] = static_cast<float>(v_[i]);
        }
        fb->time_stamp = sim_time_;
        fb->valid = true;
        return true;
    }
    bool speedj(const Arr6f &qd_deg_s, const Arr6f &, float) override
    {
        buffer_[head_] = qd_deg_s;
        head_ = (head_ + 1) % buffer_.size();
        const Arr6f &cmd = buffer_[head_];
        for (int i = 0; i < 6; ++i)
        {
            if (tau_ <= 0.0)
                v_[i] = cmd[i];
            else
                v_[i] += (cmd[i] - v_[i]) * std::min(dt_ / tau_, 1.0);
            q_[i] = static_cast<float>(q_[i] + v_[i] * dt_);
        }
        sim_time_ += dt_;
        return true;
    }
    void close() override {}

private:
    Arr6f q_{};
    std::array<double, 6> v_{};
    int delay_;
    double tau_, noise_, dt_;
    std::vector<Arr6f> buffer_;
    std::size_t head_ = 0;
    double sim_time_ = 0.0;
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_;
};

}  // namespace sync_exec
