#pragma once
// compile_motion.py 가 만든 1 ms 8축 타임라인 로더.
// 축 순서: gantry_x, gantry_y, joint_1..joint_6. 단위: 직동 m, m/s / 회전 rad, rad/s (컬럼 이름에 명시).

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace sync_exec
{

constexpr int kAxes = 8;
constexpr int kGantryAxes = 2;
constexpr int kArmAxes = 6;
using Vec8 = std::array<double, kAxes>;

struct MotionTable
{
    double dt = 0.001;
    std::vector<int> seg;
    std::vector<Vec8> q;   // [m, m, rad x6]
    std::vector<Vec8> qd;  // [m/s, m/s, rad/s x6]
    std::size_t size() const { return q.size(); }
};

inline std::vector<std::string> splitCsv(const std::string &line)
{
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ','))
    {
        while (!cell.empty() && (cell.back() == '\r' || cell.back() == ' '))
            cell.pop_back();
        out.push_back(cell);
    }
    return out;
}

inline MotionTable loadMotion(const std::string &path, double dt)
{
    static const std::vector<std::string> kHeader = {
        "k", "t", "seg",
        "gantry_x_m", "gantry_y_m",
        "joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad", "joint_5_rad", "joint_6_rad",
        "gantry_x_mps", "gantry_y_mps",
        "joint_1_radps", "joint_2_radps", "joint_3_radps", "joint_4_radps", "joint_5_radps", "joint_6_radps"};

    std::ifstream f(path);
    if (!f)
        throw std::runtime_error("motion 파일을 열 수 없다: " + path);
    std::string line;
    if (!std::getline(f, line))
        throw std::runtime_error("motion 파일이 비었다: " + path);
    if (!line.empty() && static_cast<unsigned char>(line[0]) == 0xEF)
        line = line.substr(3);  // UTF-8 BOM
    const auto header = splitCsv(line);
    if (header != kHeader)
        throw std::runtime_error(
            "motion 헤더가 다르다. compile_motion.py 출력(단위 접미어 _m/_rad/_mps/_radps)만 받는다: " + path);

    MotionTable m;
    m.dt = dt;
    std::size_t row = 0;
    while (std::getline(f, line))
    {
        if (line.empty())
            continue;
        const auto c = splitCsv(line);
        if (c.size() != kHeader.size())
            throw std::runtime_error("motion 행 컬럼 수 오류: row " + std::to_string(row));
        const long k = std::stol(c[0]);
        const double t = std::stod(c[1]);
        if (k != static_cast<long>(row))
            throw std::runtime_error("k 가 연속이 아니다: row " + std::to_string(row));
        if (std::fabs(t - static_cast<double>(row) * dt) > 1e-6)
            throw std::runtime_error("t != k*dt (dt 불일치?) row " + std::to_string(row));
        Vec8 q{}, qd{};
        for (int i = 0; i < kAxes; ++i)
        {
            q[i] = std::stod(c[3 + i]);
            qd[i] = std::stod(c[3 + kAxes + i]);
            if (!std::isfinite(q[i]) || !std::isfinite(qd[i]))
                throw std::runtime_error("NaN/Inf row " + std::to_string(row));
        }
        m.seg.push_back(std::stoi(c[2]));
        m.q.push_back(q);
        m.qd.push_back(qd);
        ++row;
    }
    if (m.size() < 2)
        throw std::runtime_error("motion 행이 2개 미만");
    return m;
}

}  // namespace sync_exec
