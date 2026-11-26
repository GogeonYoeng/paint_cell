#include "sync_executor/xgt_client.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace sync_exec
{

namespace
{
void put16(std::vector<uint8_t> &v, uint16_t x)
{
    v.push_back(static_cast<uint8_t>(x & 0xFF));
    v.push_back(static_cast<uint8_t>(x >> 8));
}
uint16_t get16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
}  // namespace

XgtClient::~XgtClient() { close(); }

bool XgtClient::connect(const std::string &ip, int port, int timeout_ms)
{
    close();
    timeout_ms_ = timeout_ms;
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0)
    {
        err_ = std::strerror(errno);
        return false;
    }
    int one = 1;
    setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
    {
        err_ = "잘못된 IP " + ip;
        close();
        return false;
    }
    if (::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
        err_ = std::string("connect: ") + std::strerror(errno);
        close();
        return false;
    }
    return true;
}

void XgtClient::close()
{
    if (fd_ >= 0)
        ::close(fd_);
    fd_ = -1;
}

bool XgtClient::sendAll(const uint8_t *p, size_t n)
{
    while (n > 0)
    {
        const ssize_t r = ::send(fd_, p, n, MSG_NOSIGNAL);
        if (r <= 0)
        {
            err_ = std::string("send: ") + std::strerror(errno);
            return false;
        }
        p += r;
        n -= static_cast<size_t>(r);
    }
    return true;
}

bool XgtClient::recvAll(uint8_t *p, size_t n)
{
    while (n > 0)
    {
        const ssize_t r = ::recv(fd_, p, n, 0);
        if (r <= 0)
        {
            err_ = r == 0 ? "연결 종료" : std::string("recv: ") + std::strerror(errno);
            return false;
        }
        p += r;
        n -= static_cast<size_t>(r);
    }
    return true;
}

bool XgtClient::transact(const std::vector<uint8_t> &body, std::vector<uint8_t> *resp_body)
{
    if (fd_ < 0)
    {
        err_ = "연결 안 됨";
        return false;
    }
    const uint16_t id = ++invoke_;
    std::vector<uint8_t> f;
    f.reserve(20 + body.size());
    const char company[10] = {'L', 'S', 'I', 'S', '-', 'X', 'G', 'T', 0, 0};
    f.insert(f.end(), company, company + 10);
    put16(f, 0x0000);  // PLC info (클라이언트 요청: 0)
    f.push_back(0x00);  // CPU info
    f.push_back(0x33);  // source of frame: client → server
    put16(f, id);
    put16(f, static_cast<uint16_t>(body.size()));
    f.push_back(0x00);  // FEnet position
    uint8_t bcc = 0;
    for (uint8_t b : f)
        bcc = static_cast<uint8_t>(bcc + b);
    f.push_back(bcc);
    f.insert(f.end(), body.begin(), body.end());
    if (!sendAll(f.data(), f.size()))
    {
        close();
        return false;
    }

    uint8_t hdr[20];
    if (!recvAll(hdr, 20))
    {
        close();
        return false;
    }
    if (std::memcmp(hdr, "LSIS-XGT", 8) != 0 || hdr[13] != 0x11)
    {
        err_ = "응답 헤더 이상";
        close();
        return false;
    }
    const uint16_t rid = get16(hdr + 14);
    const uint16_t len = get16(hdr + 16);
    resp_body->resize(len);
    if (len && !recvAll(resp_body->data(), len))
    {
        close();
        return false;
    }
    if (rid != id)
    {
        err_ = "invoke id 불일치";
        close();
        return false;
    }
    // command(2) type(2) reserved(2) error(2)
    if (len < 8)
    {
        err_ = "응답 길이 부족";
        return false;
    }
    const uint16_t status = get16(resp_body->data() + 6);
    if (status != 0)
    {
        const uint16_t code = len >= 10 ? get16(resp_body->data() + 8) : 0;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "XGT 오류 status=0x%04X code=0x%04X", status, code);
        err_ = buf;
        return false;
    }
    return true;
}

bool XgtClient::readBytes(uint32_t byte_addr, uint16_t n, uint8_t *out)
{
    uint16_t done = 0;
    while (done < n)
    {
        const uint16_t chunk = std::min<uint16_t>(static_cast<uint16_t>(n - done), max_chunk_);
        const std::string name = prefix_ + std::to_string(byte_addr + done);
        std::vector<uint8_t> b;
        put16(b, 0x0054);
        put16(b, 0x0014);
        put16(b, 0x0000);
        put16(b, 1);
        put16(b, static_cast<uint16_t>(name.size()));
        b.insert(b.end(), name.begin(), name.end());
        put16(b, chunk);
        std::vector<uint8_t> r;
        if (!transact(b, &r))
            return false;
        // command type reserved error blockcount(2) size(2) data
        if (r.size() < 12 + static_cast<size_t>(chunk) || get16(r.data()) != 0x0055 ||
            get16(r.data() + 10) != chunk)
        {
            err_ = "읽기 응답 형식 이상";
            return false;
        }
        std::memcpy(out + done, r.data() + 12, chunk);
        done = static_cast<uint16_t>(done + chunk);
    }
    return true;
}

bool XgtClient::writeBytes(uint32_t byte_addr, uint16_t n, const uint8_t *data)
{
    uint16_t done = 0;
    while (done < n)
    {
        const uint16_t chunk = std::min<uint16_t>(static_cast<uint16_t>(n - done), max_chunk_);
        const std::string name = prefix_ + std::to_string(byte_addr + done);
        std::vector<uint8_t> b;
        put16(b, 0x0058);
        put16(b, 0x0014);
        put16(b, 0x0000);
        put16(b, 1);
        put16(b, static_cast<uint16_t>(name.size()));
        b.insert(b.end(), name.begin(), name.end());
        put16(b, chunk);
        b.insert(b.end(), data + done, data + done + chunk);
        std::vector<uint8_t> r;
        if (!transact(b, &r))
            return false;
        if (r.size() < 10 || get16(r.data()) != 0x0059)
        {
            err_ = "쓰기 응답 형식 이상";
            return false;
        }
        done = static_cast<uint16_t>(done + chunk);
    }
    return true;
}

}  // namespace sync_exec
