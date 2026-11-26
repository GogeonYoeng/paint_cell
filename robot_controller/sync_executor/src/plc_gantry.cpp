#include "sync_executor/plc_gantry.hpp"

#include "sync_executor/motion.hpp"  // splitCsv
#include "sync_executor/plc_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

namespace sync_exec
{

namespace
{
int64_t monoNs()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
std::vector<uint8_t> u16(uint16_t v) { return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>(v >> 8)}; }
std::vector<uint8_t> f64(double v)
{
    std::vector<uint8_t> b(8);
    std::memcpy(b.data(), &v, 8);  // x86/ARM 리틀엔디언 = XGT 바이트 순서
    return b;
}
uint16_t rd16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
double rdf(const uint8_t *p)
{
    double v;
    std::memcpy(&v, p, 8);
    return v;
}
}  // namespace

bool loadCamPlan(const std::string &csv_path, const std::string &meta_path, CamPlan *plan, std::string *err)
{
    std::ifstream m(meta_path);
    if (!m)
    {
        *err = "meta 파일 없음: " + meta_path;
        return false;
    }
    std::map<std::string, double> kv;
    std::string line;
    while (std::getline(m, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        const auto eq = line.find('=');
        if (eq != std::string::npos)
            kv[line.substr(0, eq)] = std::stod(line.substr(eq + 1));
    }
    for (const char *k : {"lead_in", "t_plan0", "x_sign", "x_offset_mm", "y_sign", "y_offset_mm", "n_segments"})
        if (!kv.count(k))
        {
            *err = std::string("meta 키 없음: ") + k;
            return false;
        }
    plan->lead_in = kv["lead_in"];
    plan->t_plan0 = kv["t_plan0"];
    plan->x_sign = kv["x_sign"];
    plan->x_offset_mm = kv["x_offset_mm"];
    plan->y_sign = kv["y_sign"];
    plan->y_offset_mm = kv["y_offset_mm"];
    plan->segs.assign(static_cast<size_t>(kv["n_segments"]), CamSegment{});

    std::ifstream f(csv_path);
    if (!f || !std::getline(f, line))
    {
        *err = "cam csv 없음: " + csv_path;
        return false;
    }
    if (splitCsv(line) != std::vector<std::string>{"seg", "i", "master_s", "x_mm", "y_mm", "vx_mmps", "vy_mmps"})
    {
        *err = "cam csv 헤더가 다르다 (export_gantry_plc.py 출력만)";
        return false;
    }
    while (std::getline(f, line))
    {
        if (line.empty())
            continue;
        const auto c = splitCsv(line);
        const size_t s = static_cast<size_t>(std::stoi(c[0]));
        if (s >= plan->segs.size())
        {
            *err = "seg 번호 범위 밖";
            return false;
        }
        auto &g = plan->segs[s];
        const double vx = std::stod(c[5]), vy = std::stod(c[6]);
        if (g.master.empty())
        {
            g.vx0 = vx;
            g.vy0 = vy;
        }
        g.master.push_back(std::stod(c[2]));
        g.x.push_back(std::stod(c[3]));
        g.y.push_back(std::stod(c[4]));
        g.vx1 = vx;
        g.vy1 = vy;
    }
    for (size_t s = 0; s < plan->segs.size(); ++s)
    {
        const auto &g = plan->segs[s];
        if (g.master.size() < 2)
        {
            *err = "세그먼트 " + std::to_string(s) + " 포인트 부족";
            return false;
        }
        for (size_t i = 1; i < g.master.size(); ++i)
            if (!(g.master[i] > g.master[i - 1]))
            {
                *err = "master 가 증가하지 않음 seg " + std::to_string(s);
                return false;
            }
        if (s > 0 && std::fabs(plan->segs[s - 1].master.back() - g.master.front()) > 1e-9)
        {
            *err = "세그먼트 경계 master 불연속 " + std::to_string(s);
            return false;
        }
    }
    return true;
}

PlcGantry::PlcGantry(PlcConfig cfg, CamPlan plan) : cfg_(std::move(cfg)), plan_(std::move(plan))
{
    clk_buf_.reserve(4096);
}

PlcGantry::~PlcGantry() { close(); }

bool PlcGantry::open()
{
    for (size_t s = 0; s < plan_.segs.size(); ++s)
        if (plan_.segs[s].master.size() > cfg_.nmax)
        {
            std::cerr << "[plc] 세그먼트 " << s << " 포인트 " << plan_.segs[s].master.size() << " > nmax " << cfg_.nmax
                      << "\n";
            return false;
        }
    if (!xgt_.connect(cfg_.ip, cfg_.port))
    {
        std::cerr << "[plc] 연결 실패 " << cfg_.ip << ":" << cfg_.port << " " << xgt_.lastError() << "\n";
        return false;
    }
    if (!pollOnce())
    {
        std::cerr << "[plc] 상태 읽기 실패: " << xgt_.lastError() << "\n";
        return false;
    }
    std::cout << "[plc] 연결됨, state=" << st_.state << " plc_alive=" << st_.plc_alive << "\n";
    if (st_.state == plc::ST_ERROR || st_.state == plc::ST_DONE)
    {
        if (!sendCommand(plc::CMD_RESET, {}))
            return false;
    }
    else if (st_.state == plc::ST_RUNNING || st_.state == plc::ST_STOPPING)
    {
        std::cerr << "[plc] PLC 가 이미 동작 중이다 (state " << st_.state << ")\n";
        return false;
    }
    // 슬롯 0/1 사전 적재
    for (int s = 0; s < std::min<int>(2, static_cast<int>(plan_.segs.size())); ++s)
        if (!loadSegment(s))
            return false;
    next_load_ = std::min<int>(2, static_cast<int>(plan_.segs.size()));
    loaded01_ = true;
    if (!sendCommand(plc::CMD_PREPOS, {{plc::CMD_PREPOS_VEL, f64(cfg_.prepos_vel_mmps)}}))
        return false;
    run_ = true;
    comm_ = std::thread(&PlcGantry::commLoop, this);
    return true;
}

void PlcGantry::close()
{
    run_ = false;
    if (comm_.joinable())
        comm_.join();
    xgt_.close();
}

bool PlcGantry::pollOnce()
{
    uint8_t b[plc::ST_BLOCK_SIZE];
    const int64_t t0 = monoNs();
    if (!xgt_.readBytes(cfg_.base_byte + plc::statusBase(cfg_.nmax), plc::ST_BLOCK_SIZE, b))
        return false;
    const int64_t t1 = monoNs();
    PlcStatus s;
    s.ack_seq = rd16(b + plc::ST_ACK_SEQ);
    s.ack_code = rd16(b + plc::ST_ACK_CODE);
    s.ack_err = rd16(b + plc::ST_ACK_ERR);
    s.state = rd16(b + plc::ST_STATE);
    s.active_seg = rd16(b + plc::ST_ACTIVE_SEG);
    s.queued_seg = rd16(b + plc::ST_QUEUED_SEG);
    s.plc_alive = rd16(b + plc::ST_PLC_ALIVE);
    s.axis_err = rd16(b + plc::ST_AXIS_ERR);
    s.master_pos = rdf(b + plc::ST_MASTER_POS);
    s.master_vel = rdf(b + plc::ST_MASTER_VEL);
    s.x_act = rdf(b + plc::ST_X_ACT);
    s.y_act = rdf(b + plc::ST_Y_ACT);
    s.x_cmd = rdf(b + plc::ST_X_CMD);
    s.y_cmd = rdf(b + plc::ST_Y_CMD);
    s.t_send_ns = t0;
    s.t_recv_ns = t1;
    const double rtt = (t1 - t0) * 1e-3;
    if (rtt > rtt_max_us_.load())
        rtt_max_us_ = rtt;
    ++polls_;
    updateClock(s);
    return true;
}

void PlcGantry::updateClock(const PlcStatus &s)
{
    const int64_t t_mid = (s.t_send_ns + s.t_recv_ns) / 2;
    std::lock_guard<std::mutex> lk(mtx_);
    // 갠트리 속도 (지령 위치 차분, 계획 좌표 m/s)
    if (prev_t_ != 0 && t_mid > prev_t_)
    {
        const double dt = (t_mid - prev_t_) * 1e-9;
        vel_x_ = (toPlanX(s.x_cmd) - toPlanX(prev_x_cmd_)) / dt;
        vel_y_ = (toPlanY(s.y_cmd) - toPlanY(prev_y_cmd_)) / dt;
    }
    prev_t_ = t_mid;
    prev_x_cmd_ = s.x_cmd;
    prev_y_cmd_ = s.y_cmd;
    st_ = s;

    // 시계 모델: 마스터가 1 s/s 로 정속일 때만 표본으로 쓴다
    const bool cruising = s.state == plc::ST_RUNNING && std::fabs(s.master_vel - 1.0) < 1e-3 && s.master_pos > 0;
    if (!cruising)
        return;
    if (cfg_.clock_fixed && clk_valid_)
        return;
    const double rtt_us = (s.t_recv_ns - s.t_send_ns) * 1e-3;
    clk_buf_.push_back({t_mid, s.master_pos, rtt_us});
    const int64_t horizon = t_mid - static_cast<int64_t>(cfg_.clock_window_s * 1e9);
    if (clk_buf_.size() > 4000 || (clk_buf_.size() > 2 && clk_buf_.front().t_mid < horizon))
    {
        auto it = std::find_if(clk_buf_.begin(), clk_buf_.end(), [&](const ClkSample &c) { return c.t_mid >= horizon; });
        clk_buf_.erase(clk_buf_.begin(), it);
    }
    if (clk_buf_.size() < 10)
        return;
    // 가중 최소제곱: master = a + b (t - t0), 가중치 1/(rtt+100us)^2
    const int64_t t0 = clk_buf_.back().t_mid;
    double sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const auto &c : clk_buf_)
    {
        const double w = 1.0 / ((c.rtt_us + 100.0) * (c.rtt_us + 100.0));
        const double x = (c.t_mid - t0) * 1e-9;
        sw += w;
        sx += w * x;
        sy += w * c.master;
        sxx += w * x * x;
        sxy += w * x * c.master;
    }
    const double span = (clk_buf_.back().t_mid - clk_buf_.front().t_mid) * 1e-9;
    double b = 1.0;
    if (!cfg_.clock_fixed && span > 0.2)
    {
        const double den = sw * sxx - sx * sx;
        if (den > 0)
            b = (sw * sxy - sx * sy) / den;
        b = std::clamp(b, 0.999, 1.001);  // 수정 발진기 드리프트는 ±1000 ppm 이내
    }
    const double a = (sy - b * sx) / sw;
    clk_a_ = a;
    clk_b_ = b;
    clk_t0_ = t0;
    if (span > 0.03 || cfg_.clock_fixed)
        clk_valid_ = true;
}

bool PlcGantry::masterTimeAt(int64_t mono_ns, double *master) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!clk_valid_)
        return false;
    *master = clk_a_ + clk_b_ * (mono_ns - clk_t0_) * 1e-9;
    return true;
}

bool PlcGantry::read(GantryFeedback *fb)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (polls_ == 0 || comm_fail_)
    {
        fb->valid = false;
        return false;
    }
    fb->pos_m = {toPlanX(st_.x_act), toPlanY(st_.y_act)};
    fb->vel_mps = {vel_x_, vel_y_};
    fb->valid = true;
    return true;
}

PlcStatus PlcGantry::status() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return st_;
}

bool PlcGantry::ready() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return loaded01_ && st_.state == plc::ST_READY;
}

bool PlcGantry::done() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return st_.state == plc::ST_DONE;
}

bool PlcGantry::failed(const char **why) const
{
    if (comm_fail_)
    {
        *why = "PLC 통신 끊김";
        return true;
    }
    if (underrun_)
    {
        *why = "PLC 캠 공급 지연 (다음 세그먼트 예약 안 됨)";
        return true;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    if (st_.state == plc::ST_ERROR)
    {
        *why = "PLC ERROR 상태";
        return true;
    }
    return false;
}

void PlcGantry::requestStart() { start_req_ = true; }
void PlcGantry::requestStop() { stop_req_ = true; }

bool PlcGantry::sendCommand(uint16_t code, const std::vector<std::pair<uint32_t, std::vector<uint8_t>>> &fields)
{
    for (const auto &f : fields)
        if (!xgt_.writeBytes(cfg_.base_byte + f.first, static_cast<uint16_t>(f.second.size()), f.second.data()))
            return false;
    auto c = u16(code);
    if (!xgt_.writeBytes(cfg_.base_byte + plc::CMD_CODE, 2, c.data()))
        return false;
    ++seq_;
    if (seq_ == 0)
        seq_ = 1;
    auto s = u16(seq_);
    if (!xgt_.writeBytes(cfg_.base_byte + plc::CMD_SEQ, 2, s.data()))
        return false;
    return waitAck(seq_, cfg_.cmd_timeout_ms);
}

bool PlcGantry::waitAck(uint16_t seq, int timeout_ms)
{
    const int64_t deadline = monoNs() + static_cast<int64_t>(timeout_ms) * 1000000LL;
    while (monoNs() < deadline)
    {
        if (!pollOnce())
            return false;
        PlcStatus s = status();
        if (s.ack_seq == seq)
        {
            if (s.ack_err != 0)
            {
                std::cerr << "[plc] 명령 " << s.ack_code << " 오류 ErrorID=0x" << std::hex << s.ack_err << std::dec << "\n";
                return false;
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    std::cerr << "[plc] 명령 ack 시간 초과 (seq " << seq << ")\n";
    return false;
}

bool PlcGantry::loadSegment(int seg)
{
    const auto &g = plan_.segs[static_cast<size_t>(seg)];
    const int slot = seg % 2;
    const uint16_t n = static_cast<uint16_t>(g.master.size());
    const std::vector<double> *arrs[3] = {&g.master, &g.x, &g.y};
    for (int a = 0; a < 3; ++a)
    {
        const uint32_t off = cfg_.base_byte + plc::camOffset(cfg_.nmax, slot, a, 0);
        if (!xgt_.writeBytes(off, static_cast<uint16_t>(n * 8), reinterpret_cast<const uint8_t *>(arrs[a]->data())))
            return false;
        if (run_ && !pollOnce())  // RUN 중에는 배열 사이에 폴링해 시계 표본이 끊기지 않게
            return false;
    }
    return sendCommand(plc::CMD_LOAD, {{plc::CMD_SLOT, u16(static_cast<uint16_t>(slot))},
                                       {plc::CMD_SEG, u16(static_cast<uint16_t>(seg))},
                                       {plc::CMD_NPTS, u16(n)},
                                       {plc::CMD_SLOPE_X0, f64(g.vx0)},
                                       {plc::CMD_SLOPE_X1, f64(g.vx1)},
                                       {plc::CMD_SLOPE_Y0, f64(g.vy0)},
                                       {plc::CMD_SLOPE_Y1, f64(g.vy1)}});
}

void PlcGantry::commLoop()
{
    const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / cfg_.poll_hz));
    const int alive_every = std::max(1, static_cast<int>(cfg_.poll_hz / cfg_.watchdog_alive_hz));
    auto next = std::chrono::steady_clock::now();
    uint16_t alive = 0;
    long i = 0;
    bool stop_sent = false;
    const int nseg = static_cast<int>(plan_.segs.size());
    while (run_)
    {
        next += period;
        std::this_thread::sleep_until(next);
        bool ok = pollOnce();
        if (ok && (++i % alive_every) == 0)
        {
            auto a = u16(++alive);
            ok = xgt_.writeBytes(cfg_.base_byte + plc::PC_ALIVE, 2, a.data());
        }
        if (ok && stop_req_ && !stop_sent)
        {
            stop_sent = true;
            ok = sendCommand(plc::CMD_STOP, {{plc::CMD_HALT_DECEL, f64(cfg_.halt_decel)}});
        }
        else if (ok && start_req_ && !started_ && !stop_req_)
        {
            started_ = true;
            ok = sendCommand(plc::CMD_START, {{plc::CMD_NSEG, u16(static_cast<uint16_t>(nseg))}});
        }
        else if (ok && started_ && !stop_req_ && next_load_ < nseg)
        {
            const PlcStatus s = status();
            // 세그먼트 n 은 슬롯 n%2 를 쓰는데, 그 슬롯의 이전 사용자(n-2)가 끝나야(= active >= n-1) 덮어쓸 수 있다
            if (s.state == plc::ST_RUNNING && static_cast<int>(s.active_seg) >= next_load_ - 1)
            {
                ok = loadSegment(next_load_);
                if (ok)
                    ++next_load_;
            }
        }
        if (ok && started_ && !stop_req_)
        {
            // 공급 지연 감시: 현재 세그먼트 끝 100 ms 전인데 다음 세그먼트가 예약 안 됐으면 실패
            const PlcStatus s = status();
            const int a = s.active_seg;
            if (s.state == plc::ST_RUNNING && a + 1 < nseg &&
                s.master_pos > plan_.segs[static_cast<size_t>(a)].master.back() - 0.1 && s.queued_seg != a + 1)
                underrun_ = true;
        }
        if (!ok)
        {
            std::cerr << "[plc] 통신 실패: " << xgt_.lastError() << "\n";
            comm_fail_ = true;
            break;
        }
    }
}

}  // namespace sync_exec
