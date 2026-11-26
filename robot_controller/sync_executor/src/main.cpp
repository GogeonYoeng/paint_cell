// sync_executor: 갠트리 2축 + 두산 6축을 하나의 1 ms 루프에서 같은 tick 으로 구동한다.
//
//   tick 마다:  read(갠트리, 로봇) → 기준 k 선택 → 갠트리 CSP 목표(+속도 FF) write → 로봇 speedj_rt write → 로그
//   단계:      PREPOSITION(각자 시작 자세로, 저속) → 둘 다 settle 되면 같은 tick 에 k=0 시작 → RUN → TAIL(정지)
//              비상: SIGINT / 추종오차 / 피드백 끊김 → ABORT(가속도 한계로 감속 후 정지)
//
// 로봇 명령:   qd_cmd = qd_ref[k+Lr] + Kp (q_ref[k] - q_act) → 속도 클램프 → 가속도 클램프 → speedj_rt
// 갠트리 명령: pos = q_ref[k+Lg], vel_ff = qd_ref[k+Lg]
// 종료:        TAIL/ABORT 는 명령 0 + 실측 속도 0 이 100 ms 지속되어야 끝낸다 (그 뒤 stop_rt → servo_off)
//
// --gantry plc (XMC 캠 동기): 갠트리 궤적은 PLC 가 가상 마스터축(=시간) 캠으로 직접 만든다. PC 는 캠 데이터만 공급하고
//   로봇이 PLC 마스터 시간을 따라간다:  τ = master_est(now) - lead_in
//   로봇 ff = plan(τ - Dg + Dr), fb 목표 = plan(τ - Dg)   (Dg/Dr = 갠트리/로봇 실측 지연, --gantry-delay-ms/--robot-delay-ms)
// Lr/Lg(lead tick) 는 장비별 지연 보상. 실측 지연은 verify_sync.py 가 로그에서 추정한다.
// RUN 은 k = k_begin - max(Lr,Lg) 부터 시작하고 장비별 기준 인덱스는 [k_begin, k_last] 로 자른다
// → 지연이 큰 장비가 먼저 출발해 실제 움직임이 동시에 시작된다 (첫 tick 에 lead 만큼 건너뛰는 점프 없음).

#include "sync_executor/backends.hpp"
#include "sync_executor/motion.hpp"
#include "sync_executor/plc_gantry.hpp"
#ifdef SYNC_WITH_DRFL
#include "sync_executor/drfl_robot_backend.hpp"
#endif

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <sys/mman.h>
#include <vector>

using namespace sync_exec;

namespace
{

std::atomic_bool g_stop{false};
void onSignal(int) { g_stop = true; }

constexpr double kPi = 3.14159265358979323846;
inline double deg(double r) { return r * 180.0 / kPi; }

int64_t nowNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

enum Phase : uint8_t { PRE = 0, RUN = 1, TAIL = 2, ABORT = 3 };
const char *phaseName(int p)
{
    static const char *n[] = {"pre", "run", "tail", "abort"};
    return n[p];
}

struct LogRow
{
    int64_t tick;
    int32_t k;  // 모션 인덱스 (RUN 외에는 -1)
    uint8_t phase;
    int64_t wake_ns, late_ns, gantry_write_ns, robot_write_ns;
    Arr2 g_cmd, g_vff, g_fb;
    Arr6f r_ref_deg, r_cmd_qd, r_fb_q;
    double r_ts;
    int64_t mono_ns;    // CLOCK_MONOTONIC 절대값 (외부 로그와 맞출 때)
    double t_plan;      // 로봇 fb 목표의 계획 시간 [s]
    double master_est;  // PLC 모드: 추정 마스터 시간 (아니면 -1)
};

struct Args
{
    std::map<std::string, std::string> kv;
    std::vector<std::string> flags;
    bool has(const std::string &k) const
    {
        return kv.count(k) || std::find(flags.begin(), flags.end(), k) != flags.end();
    }
    std::string str(const std::string &k, const std::string &d) const
    {
        auto it = kv.find(k);
        return it == kv.end() ? d : it->second;
    }
    double num(const std::string &k, double d) const
    {
        auto it = kv.find(k);
        return it == kv.end() ? d : std::stod(it->second);
    }
};

Args parseArgs(int argc, char **argv)
{
    static const std::vector<std::string> kFlags = {"--no-sleep", "--check-only", "--keep-servo-on", "--help",
                                                     "--allow-moving-endpoints", "--plc-clock-fixed"};
    Args a;
    for (int i = 1; i < argc; ++i)
    {
        std::string k = argv[i];
        if (std::find(kFlags.begin(), kFlags.end(), k) != kFlags.end())
            a.flags.push_back(k);
        else if (k.rfind("--", 0) == 0 && i + 1 < argc)
            a.kv[k] = argv[++i];
        else
            throw std::runtime_error("알 수 없는 인자: " + k);
    }
    return a;
}

void usage()
{
    std::cout << R"(사용: sync_executor --motion <motion.csv> [옵션]
  --robot mock|drfl          (기본 mock)      --gantry mock            (EtherCAT 은 드라이브 확정 후)
  --log <out.csv>            (기본 <motion>_synclog.csv)
  --start-tick N --end-tick N  모션 일부만 재생 (row 하나 테스트용)
  --no-sleep                 실시간 대기 없이 시뮬레이션 (mock 전용, 빠른 검증)
  --check-only               파일 검증만
  --rt-priority 80 (0=끔)    --cpu <코어번호>
  --kp 4.0  --max-qd-deg 70  --max-qdd-deg 500  --speedj-time 0.001
  --robot-lead-ticks 0  --gantry-lead-ticks 0      지연 보상
  --track-err-deg 3.0  --gantry-err-mm 5.0  --fb-loss-ticks 10   비상정지 기준
  --pre-vel-deg 10 --pre-acc-deg 50 --pre-gantry-vel 0.05 --pre-gantry-acc 0.2
  --pre-tol-deg 0.01 --pre-tol-mm 0.2 --settle-ms 200 --pre-timeout 120 --tail-ms 300
  --pre-max-deg 10 --pre-max-mm 100   사전위치 허용 최대 거리 (사전위치는 충돌 검사 없음)
  --gantry-amax-abort 1.6   --allow-moving-endpoints (시작/끝 점 속도≠0 허용, 비권장)
  mock: --mock-robot-delay 2 --mock-robot-tau 0.005 --mock-robot-noise 0
        --mock-gantry-delay 2 --mock-gantry-tau 0 --mock-robot-offset-deg 0 --mock-gantry-offset-mm 0
  plc:  --gantry plc --cam <prefix>_cam.csv (meta 는 <prefix>_cam.meta) --plc-ip 192.168.0.10 --plc-port 2004
        --plc-base-mb 20000 --plc-nmax 1024 --plc-poll-hz 200 --plc-prepos-vel 50 --plc-halt-decel 2.0
        --gantry-delay-ms 0 --robot-delay-ms 0 --plc-clock-fixed (드리프트 보정 끔, 비교용)
  drfl: --robot-ip 192.168.137.100 --robot-port 12345 --rt-port 12347 --velj-rt 70 --accj-rt <max-qdd> --keep-servo-on
)";
}

void setupRealtime(int priority, int cpu)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        std::cerr << "[rt] mlockall 실패: " << std::strerror(errno) << "\n";
    if (cpu >= 0)
    {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0)
            std::cerr << "[rt] CPU " << cpu << " 고정 실패\n";
    }
    if (priority > 0)
    {
        sched_param sp{};
        sp.sched_priority = priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
            std::cerr << "[rt] SCHED_FIFO " << priority << " 실패 (ulimit -r / 권한 확인)\n";
        else
            std::cout << "[rt] SCHED_FIFO priority " << priority << (cpu >= 0 ? ", cpu " + std::to_string(cpu) : "")
                      << "\n";
    }
}

// 1D 사다리꼴: 거리 D 를 vmax, amax 로. t 에서의 진행 거리/속도.
struct Trapezoid
{
    double D = 0, v = 0, a = 1, ta = 0, tc = 0, T = 0;
    void plan(double dist, double vmax, double amax)
    {
        D = dist;
        a = amax;
        v = vmax;
        ta = v / a;
        if (a * ta * ta > D)  // 삼각형
        {
            ta = std::sqrt(D / a);
            v = a * ta;
        }
        tc = D > 0 ? (D - a * ta * ta) / v : 0.0;
        T = 2 * ta + tc;
    }
    double s(double t) const
    {
        if (D <= 0)
            return 0;
        if (t <= 0)
            return 0;
        if (t < ta)
            return 0.5 * a * t * t;
        if (t < ta + tc)
            return 0.5 * a * ta * ta + v * (t - ta);
        if (t < T)
        {
            const double r = T - t;
            return D - 0.5 * a * r * r;
        }
        return D;
    }
    double sd(double t) const
    {
        if (D <= 0 || t <= 0 || t >= T)
            return 0;
        if (t < ta)
            return a * t;
        if (t < ta + tc)
            return v;
        return a * (T - t);
    }
};

template <typename T>
double percentile(std::vector<T> v, double p)
{
    if (v.empty())
        return 0;
    std::sort(v.begin(), v.end());
    const std::size_t i = std::min(v.size() - 1, static_cast<std::size_t>(p * (v.size() - 1)));
    return static_cast<double>(v[i]);
}

}  // namespace

int main(int argc, char **argv)
{
    Args args;
    try
    {
        args = parseArgs(argc, argv);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "\n";
        usage();
        return 2;
    }
    if (args.has("--help") || !args.kv.count("--motion"))
    {
        usage();
        return args.has("--help") ? 0 : 2;
    }

    const double dt = args.num("--dt", 0.001);
    const int64_t period_ns = static_cast<int64_t>(dt * 1e9 + 0.5);
    const std::string motion_path = args.str("--motion", "");
    MotionTable motion;
    try
    {
        motion = loadMotion(motion_path, dt);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[motion] " << e.what() << "\n";
        return 2;
    }
    const long k_begin = static_cast<long>(args.num("--start-tick", 0));
    const long k_last_req = static_cast<long>(args.num("--end-tick", static_cast<double>(motion.size() - 1)));
    const long k_last = std::min<long>(k_last_req, static_cast<long>(motion.size()) - 1);
    if (k_begin < 0 || k_begin >= k_last)
    {
        std::cerr << "[motion] start/end tick 범위 오류\n";
        return 2;
    }
    const long n_run = k_last - k_begin + 1;
    std::cout << "[motion] " << motion_path << " : " << motion.size() << " tick, 재생 " << k_begin << ".." << k_last
              << " (" << n_run * dt << " s)\n";
    if (args.has("--check-only"))
        return 0;

    // ------------------------------------------------------------- 파라미터
    const double kp = args.num("--kp", 4.0);
    const double max_qd = args.num("--max-qd-deg", 70.0);
    const double max_qdd = args.num("--max-qdd-deg", 500.0);
    const float speedj_time = static_cast<float>(args.num("--speedj-time", 0.001));
    const long lead_r = static_cast<long>(args.num("--robot-lead-ticks", 0));
    const long lead_g = static_cast<long>(args.num("--gantry-lead-ticks", 0));
    const double track_err = args.num("--track-err-deg", 3.0);
    const double gantry_err = args.num("--gantry-err-mm", 5.0) * 1e-3;
    const int fb_loss_max = static_cast<int>(args.num("--fb-loss-ticks", 10));
    const double pre_vel = args.num("--pre-vel-deg", 10.0), pre_acc = args.num("--pre-acc-deg", 50.0);
    const double pre_gvel = args.num("--pre-gantry-vel", 0.05), pre_gacc = args.num("--pre-gantry-acc", 0.2);
    const double pre_tol = args.num("--pre-tol-deg", 0.01), pre_tol_g = args.num("--pre-tol-mm", 0.2) * 1e-3;
    const double pre_max_deg = args.num("--pre-max-deg", 10.0), pre_max_g = args.num("--pre-max-mm", 100.0) * 1e-3;
    const long settle_ticks = static_cast<long>(args.num("--settle-ms", 200) * 1e-3 / dt);
    const long pre_timeout_ticks = static_cast<long>(args.num("--pre-timeout", 120) / dt);
    const long tail_ticks = static_cast<long>(args.num("--tail-ms", 300) * 1e-3 / dt);
    const bool no_sleep = args.has("--no-sleep");
    const double gantry_amax_abort = args.num("--gantry-amax-abort", 1.6);  // 감속용 [m/s^2]
    const double stop_vel_deg = 0.5, stop_vel_g = 1e-3;  // 정지 판정 (실측)
    const long stop_ticks = static_cast<long>(0.1 / dt);  // 100 ms 연속 정지해야 종료
    const long stop_timeout_ticks = static_cast<long>(5.0 / dt);
    if (lead_r < 0 || lead_g < 0)
    {
        std::cerr << "lead tick 은 0 이상이어야 한다\n";
        return 2;
    }
    // 시작/끝 점은 정지해 있어야 한다 (갠트리는 가속도 클램프가 없으므로 이동 중 시작·종료 금지)
    for (long kk : {k_begin, k_last})
    {
        const Vec8 &v = motion.qd[kk];
        double gv = std::max(std::fabs(v[0]), std::fabs(v[1])), av = 0;
        for (int i = 2; i < 8; ++i)
            av = std::max(av, std::fabs(deg(v[i])));
        if ((gv > 2e-3 || av > 0.5) && !args.has("--allow-moving-endpoints"))
        {
            std::cerr << "[motion] k=" << kk << " 에서 정지 상태가 아니다 (갠트리 " << gv << " m/s, 팔 " << av
                      << " deg/s). row 경계(속도 0) 를 --start-tick/--end-tick 으로 고르라\n";
            return 2;
        }
    }
    const Vec8 &q0 = motion.q[k_begin];
    Arr6f q0_deg{};
    for (int i = 0; i < 6; ++i)
        q0_deg[i] = static_cast<float>(deg(q0[2 + i]));
    const Arr2 g0{q0[0], q0[1]};

    // ------------------------------------------------------------- 백엔드
    std::unique_ptr<RobotBackend> robot;
    std::unique_ptr<GantryBackend> gantry;
    const std::string robot_kind = args.str("--robot", "mock");
    const std::string gantry_kind = args.str("--gantry", "mock");
    if (robot_kind == "mock")
    {
        Arr6f init = q0_deg;
        const double off = args.num("--mock-robot-offset-deg", 0.0);
        for (auto &v : init)
            v = static_cast<float>(v + off);
        robot = std::make_unique<MockRobot>(init, static_cast<int>(args.num("--mock-robot-delay", 2)),
                                            args.num("--mock-robot-tau", 0.005), args.num("--mock-robot-noise", 0.0), dt);
    }
    else if (robot_kind == "drfl")
    {
#ifdef SYNC_WITH_DRFL
        if (no_sleep)
        {
            std::cerr << "--no-sleep 는 mock 전용이다\n";
            return 2;
        }
        DrflConfig c;
        c.ip = args.str("--robot-ip", c.ip);
        c.port = static_cast<unsigned>(args.num("--robot-port", c.port));
        c.rt_port = static_cast<unsigned>(args.num("--rt-port", c.rt_port));
        c.rt_period_s = static_cast<float>(dt);
        c.vel_limit_deg_s.fill(static_cast<float>(args.num("--velj-rt", 70)));
        c.acc_limit_deg_s2.fill(static_cast<float>(args.num("--accj-rt", max_qdd)));
        c.servo_off_on_close = !args.has("--keep-servo-on");
        robot = std::make_unique<DrflRobotBackend>(c);
#else
        std::cerr << "DRFL 없이 빌드됨 (SYNC_WITH_DRFL=OFF)\n";
        return 2;
#endif
    }
    else
    {
        std::cerr << "알 수 없는 --robot " << robot_kind << "\n";
        return 2;
    }
    if (gantry_kind == "mock")
    {
        const double off = args.num("--mock-gantry-offset-mm", 0.0) * 1e-3;
        gantry = std::make_unique<MockGantry>(Arr2{g0[0] + off, g0[1] + off},
                                              static_cast<int>(args.num("--mock-gantry-delay", 2)),
                                              args.num("--mock-gantry-tau", 0.0), dt);
    }
    else if (gantry_kind == "plc")
    {
        if (no_sleep)
        {
            std::cerr << "--no-sleep 는 PLC 모드에서 쓸 수 없다\n";
            return 2;
        }
        const std::string cam = args.str("--cam", "");
        CamPlan cp;
        std::string err;
        const std::string meta = cam.size() > 4 ? cam.substr(0, cam.size() - 4) + ".meta" : cam + ".meta";
        if (cam.empty() || !loadCamPlan(cam, meta, &cp, &err))
        {
            std::cerr << "[plc] 캠 데이터: " << err << " (--cam 필요)\n";
            return 2;
        }
        const double t_motion_end = (motion.size() - 1) * dt;
        if (k_begin != 0 || k_last != static_cast<long>(motion.size()) - 1 || std::fabs(cp.t_plan0) > 1e-9 ||
            std::fabs(cp.masterEnd() - cp.lead_in - t_motion_end) > 2e-3)
        {
            std::cerr << "[plc] 캠 데이터와 motion 이 맞지 않는다 (PLC 모드는 motion 전체를 재생; 캠 끝 "
                      << cp.masterEnd() - cp.lead_in << " s vs motion " << t_motion_end << " s)\n";
            return 2;
        }
        PlcConfig pc;
        pc.ip = args.str("--plc-ip", pc.ip);
        pc.port = static_cast<int>(args.num("--plc-port", pc.port));
        pc.base_byte = static_cast<uint32_t>(args.num("--plc-base-mb", pc.base_byte));
        pc.nmax = static_cast<uint32_t>(args.num("--plc-nmax", pc.nmax));
        pc.poll_hz = args.num("--plc-poll-hz", pc.poll_hz);
        pc.prepos_vel_mmps = args.num("--plc-prepos-vel", pc.prepos_vel_mmps);
        pc.halt_decel = args.num("--plc-halt-decel", pc.halt_decel);
        pc.clock_fixed = args.has("--plc-clock-fixed");
        std::cout << "[plc] 캠 세그먼트 " << cp.segs.size() << ", lead-in " << cp.lead_in << " s, 마스터 끝 "
                  << cp.masterEnd() << " s\n";
        gantry = std::make_unique<PlcGantry>(pc, std::move(cp));
    }
    else
    {
        std::cerr << "--gantry " << gantry_kind << " 미구현 (mock | plc)\n";
        return 2;
    }
    PlcGantry *plc = dynamic_cast<PlcGantry *>(gantry.get());
    const double d_g = args.num("--gantry-delay-ms", 0.0) * 1e-3, d_r = args.num("--robot-delay-ms", 0.0) * 1e-3;
    const double t_end = (k_last - k_begin) * dt;
    long clk_wait = 0;
    bool plc_stop_sent = false;
    // 계획을 연속 시간 t [s] (k_begin 기준) 에서 선형 보간
    auto planAt = [&](double t, Vec8 *q, Vec8 *qd) {
        const double u = std::clamp(t / dt, 0.0, static_cast<double>(k_last - k_begin));
        const long i0 = std::min(static_cast<long>(u), k_last - k_begin - 1);
        const double a = u - static_cast<double>(i0);
        const Vec8 &q0v = motion.q[k_begin + i0], &q1v = motion.q[k_begin + i0 + 1];
        const Vec8 &d0v = motion.qd[k_begin + i0], &d1v = motion.qd[k_begin + i0 + 1];
        for (int j = 0; j < kAxes; ++j)
        {
            (*q)[j] = q0v[j] + a * (q1v[j] - q0v[j]);
            (*qd)[j] = d0v[j] + a * (d1v[j] - d0v[j]);
        }
    };

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::cout << "[exec] robot=" << robot->name() << " gantry=" << gantry->name() << (no_sleep ? " (no-sleep)" : "")
              << " kp=" << kp << " lead r/g=" << lead_r << "/" << lead_g << "\n";
    if (!gantry->open())
    {
        std::cerr << "[exec] 갠트리 open 실패\n";
        return 1;
    }
    if (!robot->open())
    {
        std::cerr << "[exec] 로봇 open 실패\n";
        robot->close();
        gantry->close();
        return 1;
    }

    std::vector<LogRow> log;
    log.reserve(static_cast<std::size_t>(n_run + pre_timeout_ticks + tail_ticks + 20000));

    if (!no_sleep)
        setupRealtime(static_cast<int>(args.num("--rt-priority", 80)), static_cast<int>(args.num("--cpu", -1)));

    // ------------------------------------------------------------- 루프 상태
    int64_t tick = 0;
    Phase phase = PRE;
    std::string abort_reason;
    long k = -1;
    long pre_ticks = 0, settled = 0, tail = 0, fb_loss = 0;
    Arr6f prev_cmd{};
    RobotFeedback rfb, rfb_last;
    GantryFeedback gfb, gfb_last;
    Arr2 g_cmd{}, g_vff{};
    Arr2 g_cmd_sent{}, g_vff_sent{};  // 직전 tick 에 실제로 송신한 갠트리 명령
    bool g_pre_planned = false;
    Trapezoid g_traj;
    Arr2 g_pre_from{}, g_dir{};
    Arr2 g_abort_v{};
    bool abort_init = false, ever_valid = false;
    long stopped_ticks = 0, end_ticks = 0, write_fail = 0;
    std::vector<std::string> events;  // RT 루프 안에서는 출력하지 않고 모아 두었다가 끝나고 출력
    events.reserve(64);
    auto event = [&](std::string msg) {
        if (events.size() < events.capacity())
            events.push_back(std::to_string(tick * dt) + "s " + std::move(msg));
    };
    auto enterAbort = [&](std::string reason) {
        if (phase != ABORT)
        {
            phase = ABORT;
            abort_reason = std::move(reason);
        }
    };


    auto clampArm = [&](Arr6f qd, double vmax, double amax) {
        Arr6f acc{};
        for (int i = 0; i < 6; ++i)
        {
            double v = std::clamp(static_cast<double>(qd[i]), -vmax, vmax);
            const double a = std::clamp((v - prev_cmd[i]) / dt, -amax, amax);
            v = prev_cmd[i] + a * dt;
            qd[i] = static_cast<float>(v);
            acc[i] = static_cast<float>(a);
        }
        return std::make_pair(qd, acc);
    };

    const int64_t t0 = no_sleep ? 0 : nowNs();
    int64_t next = t0;
    long resyncs = 0;
    for (;; ++tick)
    {
        // ---- 주기 대기
        int64_t wake;
        if (no_sleep)
        {
            wake = tick * period_ns;
        }
        else
        {
            next += period_ns;
            timespec ts{static_cast<time_t>(next / 1000000000LL), static_cast<long>(next % 1000000000LL)};
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR)
            {
            }
            wake = nowNs() - t0;
            if (wake - (next - t0) > 2 * period_ns)
            {
                // 2 주기 이상 늦었으면 밀린 tick 을 몰아 보내지 않고 기준 시각을 현재로 다시 잡는다
                next = t0 + wake;
                ++resyncs;
            }
        }
        const int64_t late = no_sleep ? 0 : wake - (next - t0);

        if (g_stop)
            enterAbort("signal (Ctrl+C)");
        const char *why = nullptr;
        if (!robot->healthy(&why))
            enterAbort(why);
        if (plc && plc->failed(&why))
            enterAbort(why);

        // ---- 피드백
        const bool r_ok = robot->read(&rfb);
        const bool g_ok = gantry->read(&gfb);
        if (r_ok && g_ok)
        {
            fb_loss = 0;
            rfb_last = rfb;
            gfb_last = gfb;
            ever_valid = true;
        }
        else if (ever_valid && ++fb_loss > fb_loss_max)  // 첫 피드백 전에는 세지 않는다
        {
            enterAbort(std::string("피드백 끊김/정지 ") + (r_ok ? "" : "robot ") + (g_ok ? "" : "gantry"));
        }
        if (!rfb_last.valid || !gfb_last.valid)
        {
            // 첫 피드백 전에는 아무 명령도 보내지 않는다 (0 속도만)
            Arr6f zero{};
            robot->speedj(zero, zero, speedj_time);
            if (tick > 1000)
            {
                abort_reason = "초기 피드백 없음 (1 s)";
                break;
            }
            continue;
        }

        LogRow row{};
        row.tick = tick;
        row.k = -1;
        row.wake_ns = wake;
        row.late_ns = late;
        Arr6f qd_cmd{}, acc_cmd{};
        Arr6f ref_deg{};
        Phase row_phase = phase;  // 이 tick 의 명령을 만든 단계 (전이는 다음 tick 부터 반영)

        if (phase == PRE)
        {
            // 로봇: 시작 자세로 P 제어 (저속)
            double r_err = 0;
            for (int i = 0; i < 6; ++i)
            {
                const double e = q0_deg[i] - rfb_last.q_deg[i];
                r_err = std::max(r_err, std::fabs(e));
                qd_cmd[i] = static_cast<float>(2.0 * e);
            }
            std::tie(qd_cmd, acc_cmd) = clampArm(qd_cmd, pre_vel, pre_acc);
            ref_deg = q0_deg;
            if (plc)
            {
                // 갠트리 사전위치는 PLC 가 한다 (PREPOS 명령, open() 에서 요청). PC 는 READY 만 기다린다.
                g_pre_planned = true;
                g_cmd = gfb_last.pos_m;
                g_vff = {0, 0};
                if (pre_ticks == 0)
                    event("[pre] robot 최대오차 " + std::to_string(r_err) + " deg, 갠트리는 PLC PREPOS");
                if (r_err > pre_max_deg)
                    enterAbort("로봇이 시작 자세에서 너무 멀다: " + std::to_string(r_err) + " deg");
            }
            // 갠트리: 현재 위치 → 시작 위치 직선 사다리꼴
            if (!g_pre_planned)
            {
                g_pre_from = gfb_last.pos_m;
                const double dx = g0[0] - g_pre_from[0], dy = g0[1] - g_pre_from[1];
                const double D = std::hypot(dx, dy);
                g_dir = D > 0 ? Arr2{dx / D, dy / D} : Arr2{0, 0};
                g_traj.plan(D, pre_gvel, pre_gacc);
                g_pre_planned = true;
                event("[pre] robot 최대오차 " + std::to_string(r_err) + " deg, gantry 거리 " +
                      std::to_string(D * 1e3) + " mm (" + std::to_string(g_traj.T) + " s)");
                if (r_err > pre_max_deg || D > pre_max_g)
                    enterAbort("시작 자세에서 너무 멀다 (사전위치는 충돌 검사가 없다): robot " + std::to_string(r_err) +
                               " deg / gantry " + std::to_string(D * 1e3) + " mm. --pre-max-deg/--pre-max-mm");
            }
            const double tp = pre_ticks * dt;
            if (!plc)
                for (int i = 0; i < 2; ++i)
                {
                    g_cmd[i] = g_pre_from[i] + g_dir[i] * g_traj.s(tp);
                    g_vff[i] = g_dir[i] * g_traj.sd(tp);
                }
            const double g_err = std::hypot(g0[0] - gfb_last.pos_m[0], g0[1] - gfb_last.pos_m[1]);
            const bool ready = plc ? (r_err < pre_tol && plc->ready() && g_err < pre_tol_g)
                                   : (r_err < pre_tol && tp >= g_traj.T && g_err < pre_tol_g);
            settled = ready ? settled + 1 : 0;
            ++pre_ticks;
            if (phase == ABORT)
            {
            }
            else if (settled >= settle_ticks)
            {
                // 같은 tick 의 아래 RUN 블록이 k=k_begin 명령을 바로 만든다 (두 장비 동시 시작)
                event("[pre] 시작 자세 도달 (" + std::to_string(pre_ticks * dt) + " s), RUN 시작");
                phase = RUN;
                if (plc)
                    plc->requestStart();
                // 지연이 큰 장비가 먼저 출발하도록 k 를 lead 만큼 앞에서 시작한다.
                // 각 장비 기준은 [k_begin, k_last] 로 잘리므로 lead 가 작은 장비는 그동안 시작점에 머문다.
                k = k_begin - std::max(lead_r, lead_g) - 1;
            }
            else if (pre_ticks > pre_timeout_ticks)
            {
                enterAbort("사전위치 시간 초과");
            }
        }

        if (phase == RUN && plc)
        {
            row_phase = RUN;
            double master = -1.0;
            const bool clk = plc->masterTimeAt(nowNs(), &master);
            double tau = 0.0;  // 계획 시간 (마스터 lead-in 이 끝나는 순간 0)
            if (clk)
                tau = master - plc->plan().lead_in;
            else if (++clk_wait > static_cast<long>(3.0 / dt))
                enterAbort("PLC 시계 동기 실패 (START 후 3 s 동안 마스터 시간 추정 불가)");
            const double tau_fb = tau - d_g, tau_ff = tau - d_g + d_r;
            Vec8 q_ff, qd_ff, q_fb, qd_fb;
            planAt(tau_ff, &q_ff, &qd_ff);
            planAt(tau_fb, &q_fb, &qd_fb);
            row.k = static_cast<int32_t>(k_begin + std::lround(std::clamp(tau_fb, 0.0, t_end) / dt));
            row.t_plan = tau_fb;
            row.master_est = clk ? master : -1.0;
            double r_err = 0;
            for (int i = 0; i < 6; ++i)
            {
                ref_deg[i] = static_cast<float>(deg(q_ff[2 + i]));
                const double e = deg(q_fb[2 + i]) - rfb_last.q_deg[i];
                qd_cmd[i] = static_cast<float>(deg(qd_ff[2 + i]) + kp * e);
                r_err = std::max(r_err, std::fabs(e));
            }
            std::tie(qd_cmd, acc_cmd) = clampArm(qd_cmd, max_qd, max_qdd);
            // 갠트리: 로그용으로 계획 지령값만 기록 (실제 지령은 PLC 캠)
            Vec8 q_g, qd_g;
            planAt(tau, &q_g, &qd_g);
            g_cmd = {q_g[0], q_g[1]};
            g_vff = {qd_g[0], qd_g[1]};
            // 갠트리 추종 감시: PLC 표본 자신의 마스터 시간으로 계획과 비교
            const PlcStatus ps = plc->status();
            Vec8 q_s, qd_s;
            planAt(ps.master_pos - plc->plan().lead_in - d_g, &q_s, &qd_s);
            const double g_err = std::hypot(q_s[0] - gfb_last.pos_m[0], q_s[1] - gfb_last.pos_m[1]);
            if (r_err > track_err)
                enterAbort("로봇 추종오차 " + std::to_string(r_err) + " deg @t=" + std::to_string(tau_fb));
            else if (g_err > gantry_err)
                enterAbort("갠트리 추종오차 " + std::to_string(g_err * 1e3) + " mm @t=" + std::to_string(tau_fb));
            else if (tau_fb >= t_end && plc->done())
                phase = TAIL;
            else if (tau_fb > t_end + 3.0)
                enterAbort("계획 끝 3 s 후에도 PLC DONE 이 아니다");
        }
        else if (phase == RUN)
        {
            row_phase = RUN;
            ++k;
            row.k = static_cast<int32_t>(k);
            const long kr = std::clamp(k + lead_r, k_begin, k_last);
            const long kg = std::clamp(k + lead_g, k_begin, k_last);
            const long kc = std::clamp(k, k_begin, k_last);  // 지금 이 순간 실측이 있어야 할 계획
            const Vec8 &qr = motion.q[kr], &qdr = motion.qd[kr];
            const Vec8 &qg = motion.q[kg], &qdg = motion.qd[kg];
            double r_err = 0;
            for (int i = 0; i < 6; ++i)
            {
                // lead 는 피드포워드에만: 실측 q_act(k) 는 지연 보상 후 계획 q(k) 와 같아야 하므로
                // 피드백은 현재 시점 계획과 비교한다 (q_ref[k+L] 과 비교하면 Kp*L*dt*v 만큼 치우친다).
                ref_deg[i] = static_cast<float>(deg(qr[2 + i]));
                const double e = deg(motion.q[kc][2 + i]) - rfb_last.q_deg[i];
                qd_cmd[i] = static_cast<float>(deg(qdr[2 + i]) + kp * e);
                r_err = std::max(r_err, std::fabs(e));
            }
            std::tie(qd_cmd, acc_cmd) = clampArm(qd_cmd, max_qd, max_qdd);
            g_cmd = {qg[0], qg[1]};
            g_vff = {qdg[0], qdg[1]};
            const double g_err = std::hypot(motion.q[kc][0] - gfb_last.pos_m[0], motion.q[kc][1] - gfb_last.pos_m[1]);
            if (r_err > track_err)
                enterAbort("로봇 추종오차 " + std::to_string(r_err) + " deg @k=" + std::to_string(k));
            else if (g_err > gantry_err)
                enterAbort("갠트리 추종오차 " + std::to_string(g_err * 1e3) + " mm @k=" + std::to_string(k));
            else if (k >= k_last)
            {
                phase = TAIL;
            }
        }
        else if (phase == TAIL)
        {
            row_phase = TAIL;
            Arr6f zero{};
            std::tie(qd_cmd, acc_cmd) = clampArm(zero, max_qd, max_qdd);
            ref_deg = {};
            g_cmd = plc ? gfb_last.pos_m : Arr2{motion.q[k_last][0], motion.q[k_last][1]};
            g_vff = {0, 0};
            ++tail;
        }

        if (phase == ABORT)
        {
            row_phase = ABORT;
            if (!abort_init)
            {
                // 직전 tick 에 내보낸 명령에서 감속을 시작한다 (실측 위치로 바꾸면 지연만큼 뒤로 점프한다)
                abort_init = true;
                event("[abort] " + abort_reason);
                if (plc && !plc_stop_sent)
                {
                    plc->requestStop();  // 마스터 감속 → X/Y 가 경로를 따라 같이 감속
                    plc_stop_sent = true;
                }
                // 같은 tick 의 RUN 블록이 g_cmd 를 이미 한 칸 전진시켰을 수 있으므로 송신된 값에서 다시 시작
                g_cmd = g_cmd_sent;
                g_abort_v = g_vff_sent;
            }
            Arr6f zero{};
            std::tie(qd_cmd, acc_cmd) = clampArm(zero, max_qd, max_qdd);
            for (int i = 0; i < 2; ++i)
            {
                const double dv = std::clamp(-g_abort_v[i], -gantry_amax_abort * dt, gantry_amax_abort * dt);
                g_abort_v[i] += dv;
                g_cmd[i] += g_abort_v[i] * dt;
                g_vff[i] = g_abort_v[i];
            }
        }

        // ---- 종료 판정 (TAIL/ABORT): 명령이 0 이고 "실측" 속도가 100 ms 연속 0 근처여야 끝낸다
        if (phase == TAIL || phase == ABORT)
        {
            const bool cmd_zero = std::all_of(qd_cmd.begin(), qd_cmd.end(), [](float v) { return v == 0.0f; }) &&
                                  g_vff[0] == 0.0 && g_vff[1] == 0.0;
            bool fb_still = std::fabs(gfb_last.vel_mps[0]) < stop_vel_g && std::fabs(gfb_last.vel_mps[1]) < stop_vel_g;
            for (float v : rfb_last.qd_deg_s)
                fb_still = fb_still && std::fabs(v) < stop_vel_deg;
            stopped_ticks = (cmd_zero && fb_still) ? stopped_ticks + 1 : 0;
            ++end_ticks;
            const bool min_tail = phase == ABORT || tail >= tail_ticks;
            if ((stopped_ticks >= stop_ticks && min_tail) || end_ticks > stop_timeout_ticks)
            {
                if (stopped_ticks < stop_ticks)
                    event("[warn] 5 s 안에 실측 정지를 확인하지 못하고 종료");
                // 이 tick 도 0 명령을 보낸 뒤 루프를 나간다
                gantry->write(g_cmd, g_vff);
                robot->speedj(qd_cmd, acc_cmd, speedj_time);
                break;
            }
        }

        // ---- 같은 tick 안에서 두 장비에 송신
        row.gantry_write_ns = (no_sleep ? wake : nowNs() - t0);
        const bool gw = gantry->write(g_cmd, g_vff);
        g_cmd_sent = g_cmd;
        g_vff_sent = g_vff;
        row.robot_write_ns = (no_sleep ? wake : nowNs() - t0);
        const bool rw = robot->speedj(qd_cmd, acc_cmd, speedj_time);
        prev_cmd = qd_cmd;
        write_fail = (gw && rw) ? 0 : write_fail + 1;
        if (write_fail > fb_loss_max)
        {
            enterAbort(std::string("송신 실패 ") + (rw ? "" : "robot ") + (gw ? "" : "gantry"));
            if (write_fail > 1000)  // 송신이 계속 안 되면 루프를 끝낸다 (장비 쪽 timeout 에 맡김)
                break;
        }

        row.phase = row_phase;
        row.g_cmd = g_cmd;
        row.g_vff = g_vff;
        row.g_fb = gfb_last.pos_m;
        row.r_ref_deg = ref_deg;
        row.r_cmd_qd = qd_cmd;
        row.r_fb_q = rfb_last.q_deg;
        row.r_ts = rfb_last.time_stamp;
        row.mono_ns = no_sleep ? wake : t0 + wake;
        if (!(phase == RUN && plc) && row_phase != RUN)
        {
            row.t_plan = -1;
            row.master_est = -1;
        }
        else if (!plc)
        {
            row.t_plan = (row.k - k_begin) * dt;
            row.master_est = -1;
        }
        if (log.size() < log.capacity())
            log.push_back(row);
    }

    robot->close();
    gantry->close();
    for (const auto &e : events)
        std::cout << e << "\n";
    if (resyncs)
        std::cout << "[rt] 2 주기 이상 지연으로 기준 시각 재설정 " << resyncs << " 회\n";

    // ------------------------------------------------------------- 로그
    const std::string log_path = args.str(
        "--log", motion_path.substr(0, motion_path.size() - (motion_path.size() > 4 ? 4 : 0)) + "_synclog.csv");
    {
        std::ofstream f(log_path);
        f << "tick,k,phase,wake_ns,late_ns,gantry_write_ns,robot_write_ns,"
             "g_cmd_x,g_cmd_y,g_vff_x,g_vff_y,g_fb_x,g_fb_y,"
             "r_ref1,r_ref2,r_ref3,r_ref4,r_ref5,r_ref6,"
             "r_cmd_qd1,r_cmd_qd2,r_cmd_qd3,r_cmd_qd4,r_cmd_qd5,r_cmd_qd6,"
             "r_fb_q1,r_fb_q2,r_fb_q3,r_fb_q4,r_fb_q5,r_fb_q6,r_ts,mono_ns,t_plan,master_est\n";
        f << std::setprecision(10);
        for (const auto &r : log)
        {
            f << r.tick << ',' << r.k << ',' << phaseName(r.phase) << ',' << r.wake_ns << ',' << r.late_ns << ','
              << r.gantry_write_ns << ',' << r.robot_write_ns << ',' << r.g_cmd[0] << ',' << r.g_cmd[1] << ','
              << r.g_vff[0] << ',' << r.g_vff[1] << ',' << r.g_fb[0] << ',' << r.g_fb[1];
            for (float v : r.r_ref_deg)
                f << ',' << v;
            for (float v : r.r_cmd_qd)
                f << ',' << v;
            for (float v : r.r_fb_q)
                f << ',' << v;
            f << ',' << r.r_ts << ',' << r.mono_ns << ',' << r.t_plan << ',' << r.master_est << '\n';
        }
    }

    // ------------------------------------------------------------- 요약
    std::vector<int64_t> periods, lates, skews;
    long run_ticks = 0;
    for (std::size_t i = 0; i < log.size(); ++i)
    {
        if (i > 0)
            periods.push_back(log[i].wake_ns - log[i - 1].wake_ns);
        lates.push_back(log[i].late_ns);
        skews.push_back(log[i].robot_write_ns - log[i].gantry_write_ns);
        run_ticks += log[i].phase == RUN;
    }
    std::cout << "[summary] 로그 " << log.size() << " tick (RUN " << run_ticks << "/" << n_run + std::max(lead_r, lead_g)
              << ") -> " << log_path
              << "\n";
    if (!no_sleep && !periods.empty())
    {
        const double mean = std::accumulate(periods.begin(), periods.end(), 0.0) / periods.size();
        long over = std::count_if(lates.begin(), lates.end(), [&](int64_t l) { return l > period_ns / 2; });
        std::cout << std::fixed << std::setprecision(1) << "[summary] 주기 평균 " << mean / 1e3 << " us, 최소 "
                  << *std::min_element(periods.begin(), periods.end()) / 1e3 << ", 최대 "
                  << *std::max_element(periods.begin(), periods.end()) / 1e3 << " us | 지연 p99 "
                  << percentile(lates, 0.99) / 1e3 << " us, p99.9 " << percentile(lates, 0.999) / 1e3 << " us, 최대 "
                  << *std::max_element(lates.begin(), lates.end()) / 1e3 << " us, 0.5 ms 초과 " << over
                  << " 회 | 갠트리→로봇 송신 간격 최대 " << *std::max_element(skews.begin(), skews.end()) / 1e3
                  << " us\n";
    }
    if (plc)
        std::cout << "[plc] 폴링 " << plc->polls() << " 회, RTT 최대 " << plc->rttMaxUs() << " us\n";
    if (!abort_reason.empty())
    {
        std::cerr << "[summary] ABORT: " << abort_reason << "\n";
        return 3;
    }
    if (plc)
        return 0;
    return run_ticks == n_run + std::max(lead_r, lead_g) ? 0 : 4;
}
