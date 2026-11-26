#include "sync_executor/drfl_robot_backend.hpp"

#include "DRFLEx.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

namespace sync_exec
{

struct DrflRobotBackend::Impl
{
    DRAFramework::CDRFLEx drfl;
    DrflConfig cfg;
    std::atomic_bool connected{false};
    std::atomic_bool access{false};
    std::atomic_bool standby{false};
    std::atomic_bool disconnected{false};
    std::atomic_int state{-1};
    double last_ts = -1.0;
    int stale = 0;
    std::atomic_bool rt_connected{false};
    std::atomic_bool rt_started{false};
    bool servo_on = false;
    static Impl *active;

    static void onAccess(MONITORING_ACCESS_CONTROL a)
    {
        if (!active)
            return;
        if (a == MONITORING_ACCESS_CONTROL_GRANT)
        {
            active->access = true;
            active->standby = false;
        }
        else if (a == MONITORING_ACCESS_CONTROL_DENY || a == MONITORING_ACCESS_CONTROL_LOSS)
        {
            active->access = false;
            active->standby = false;
        }
    }
    static void onState(ROBOT_STATE s)
    {
        if (active)
        {
            active->standby = (s == STATE_STANDBY);
            active->state = static_cast<int>(s);
        }
    }
    static void onDisconnected()
    {
        if (active)
        {
            std::cerr << "[drfl] disconnected\n";
            active->disconnected = true;
            active->connected = false;
            active->rt_started = false;      // 끊긴 링크에 RT API 를 부르지 않는다
            active->rt_connected = false;
        }
    }
};

DrflRobotBackend::Impl *DrflRobotBackend::Impl::active = nullptr;

DrflRobotBackend::DrflRobotBackend(DrflConfig cfg) : impl_(std::make_unique<Impl>())
{
    impl_->cfg = std::move(cfg);
    if (impl_->cfg.rt_ip.empty())
        impl_->cfg.rt_ip = impl_->cfg.ip;
}

DrflRobotBackend::~DrflRobotBackend() { close(); }

bool DrflRobotBackend::open()
{
    auto &d = impl_->drfl;
    auto &c = impl_->cfg;
    Impl::active = impl_.get();
    d.set_on_monitoring_access_control(&Impl::onAccess);
    d.set_on_monitoring_state(&Impl::onState);
    d.set_on_disconnected(&Impl::onDisconnected);

    std::cout << "[drfl] open_connection " << c.ip << ":" << c.port << "\n";
    if (!d.open_connection(c.ip, c.port))
    {
        std::cerr << "[drfl] open_connection 실패\n";
        return false;
    }
    impl_->connected = true;
    SYSTEM_VERSION ver;
    std::memset(&ver, 0, sizeof(ver));
    if (d.get_system_version(&ver))
        std::cout << "[drfl] DRCF " << ver._szController << " / DRFL " << d.get_library_version() << "\n";
    if (!d.setup_monitoring_version(1))
    {
        std::cerr << "[drfl] setup_monitoring_version(1) 실패\n";
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(c.access_timeout_s);
    while (!(impl_->access && impl_->standby))
    {
        if (impl_->disconnected || std::chrono::steady_clock::now() > deadline)
        {
            std::cerr << "[drfl] 제어권/STANDBY 대기 시간 초과\n";
            return false;
        }
        if (!impl_->access)
            d.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        else
            d.set_robot_control(CONTROL_SERVO_ON);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!d.set_robot_mode(ROBOT_MODE_AUTONOMOUS) || !d.set_robot_system(ROBOT_SYSTEM_REAL))
    {
        std::cerr << "[drfl] robot mode/system 설정 실패\n";
        return false;
    }
    d.set_auto_servo_off(false, 5.0f);
    d.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    impl_->servo_on = true;

    std::cout << "[drfl] connect_rt_control " << c.rt_ip << ":" << c.rt_port << "\n";
    if (!d.connect_rt_control(c.rt_ip, c.rt_port))
    {
        std::cerr << "[drfl] connect_rt_control 실패\n";
        return false;
    }
    impl_->rt_connected = true;
    if (!d.set_rt_control_output("v1.0", c.rt_period_s, c.rt_loss_count))
    {
        std::cerr << "[drfl] set_rt_control_output 실패\n";
        return false;
    }
    auto vel = c.vel_limit_deg_s;
    auto acc = c.acc_limit_deg_s2;
    if (!d.set_velj_rt(vel.data()) || !d.set_accj_rt(acc.data()))
    {
        std::cerr << "[drfl] set_velj_rt/set_accj_rt 실패\n";
        return false;
    }
    if (!d.start_rt_control())
    {
        std::cerr << "[drfl] start_rt_control 실패\n";
        return false;
    }
    impl_->rt_started = true;
    return true;
}

bool DrflRobotBackend::read(RobotFeedback *fb)
{
    fb->valid = false;
    if (!impl_->rt_started || impl_->disconnected)
        return false;
    auto *data = impl_->drfl.read_data_rt();
    if (!data || !std::isfinite(data->time_stamp) || data->time_stamp <= 0.0)
        return false;
    for (int i = 0; i < 6; ++i)
    {
        if (!std::isfinite(data->actual_joint_position[i]) || !std::isfinite(data->actual_joint_velocity[i]))
            return false;
        fb->q_deg[i] = data->actual_joint_position[i];
        fb->qd_deg_s[i] = data->actual_joint_velocity[i];
    }
    // 패킷 손실 시 read_data_rt() 는 마지막 버퍼를 그대로 돌려준다 → time_stamp 가 안 바뀌면 무효로 본다.
    if (data->time_stamp == impl_->last_ts)
    {
        ++impl_->stale;
        return false;
    }
    impl_->stale = 0;
    impl_->last_ts = data->time_stamp;
    fb->time_stamp = data->time_stamp;
    fb->valid = true;
    return true;
}

bool DrflRobotBackend::speedj(const Arr6f &qd_deg_s, const Arr6f &acc_deg_s2, float time_s)
{
    if (!impl_->rt_started || impl_->disconnected)
        return false;
    auto v = qd_deg_s;
    auto a = acc_deg_s2;
    return impl_->drfl.speedj_rt(v.data(), a.data(), time_s);
}

bool DrflRobotBackend::healthy(const char **reason) const
{
    if (impl_->disconnected)
    {
        *reason = "로봇 연결 끊김";
        return false;
    }
    switch (impl_->state.load())
    {
    case STATE_SAFE_STOP:
    case STATE_SAFE_STOP2:
        *reason = "로봇 SAFE_STOP (보호정지)";
        return false;
    case STATE_SAFE_OFF:
    case STATE_SAFE_OFF2:
        *reason = "로봇 SAFE_OFF (서보 꺼짐)";
        return false;
    case STATE_EMERGENCY_STOP:
        *reason = "로봇 EMERGENCY_STOP";
        return false;
    case STATE_RECOVERY:
        *reason = "로봇 RECOVERY 모드";
        return false;
    default:
        return true;
    }
}

void DrflRobotBackend::close()
{
    if (!impl_)
        return;
    auto &d = impl_->drfl;
    // 순서는 기준 코드(doosan_controller.cpp servoOff/shutdown)와 같다: stop_rt → servo_off → disconnect_rt → close
    if (impl_->rt_started.exchange(false) && !impl_->disconnected)
        d.stop_rt_control();
    if (impl_->servo_on && impl_->cfg.servo_off_on_close && impl_->connected && !impl_->disconnected)
    {
        std::cout << "[drfl] servo_off\n";
        d.servo_off(STOP_TYPE_QUICK);
    }
    impl_->servo_on = false;
    if (impl_->rt_connected.exchange(false) && !impl_->disconnected)
        d.disconnect_rt_control();
    if (impl_->connected)
    {
        d.close_connection();
        impl_->connected = false;
    }
    if (Impl::active == impl_.get())
        Impl::active = nullptr;
}

}  // namespace sync_exec
