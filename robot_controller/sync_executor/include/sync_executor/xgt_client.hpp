#pragma once
// LS ELECTRIC XGT 전용 프로토콜(FEnet, TCP 2004) 클라이언트 — 연속(블록) 읽기/쓰기만.
//
// 프레임 (리틀엔디언):
//   헤더 20 B : "LSIS-XGT"+00 00 | PLC info 2 | CPU info 1 | source 0x33 | invoke id 2 | length 2 | FEnet pos 1 | BCC 1
//   명령      : command 2 (0x54 읽기 / 0x58 쓰기) | data type 2 (0x14 연속) | reserved 2 | block 수 2 (=1)
//               | 변수명 길이 2 | 변수명 ("%MB20000") | 바이트 수 2 | (쓰기) 데이터
//   응답      : command+1 (0x55/0x59) | data type | reserved | error status 2 (0=정상, 아니면 error code 2 뒤따름)
//               | block 수 2 | (읽기) 바이트 수 2 + 데이터
// ※ XGT FEnet 매뉴얼 규격 기준. XMC 실기에서 첫 시험으로 %MW 읽기/쓰기를 확인할 것 (2026-11-23 미검증).

#include <cstdint>
#include <string>
#include <vector>

namespace sync_exec
{

class XgtClient
{
public:
    XgtClient() = default;
    ~XgtClient();
    XgtClient(const XgtClient &) = delete;
    XgtClient &operator=(const XgtClient &) = delete;

    bool connect(const std::string &ip, int port, int timeout_ms = 1000);
    void close();
    bool connected() const { return fd_ >= 0; }

    // device 는 바이트 주소 접두어 ("%MB"). byte_addr 부터 n 바이트.
    bool readBytes(uint32_t byte_addr, uint16_t n, uint8_t *out);
    bool writeBytes(uint32_t byte_addr, uint16_t n, const uint8_t *data);

    void setDevicePrefix(const std::string &p) { prefix_ = p; }
    void setMaxChunk(uint16_t n) { max_chunk_ = n; }
    uint16_t maxChunk() const { return max_chunk_; }
    const std::string &lastError() const { return err_; }

private:
    bool transact(const std::vector<uint8_t> &req, std::vector<uint8_t> *resp_body);
    bool sendAll(const uint8_t *p, size_t n);
    bool recvAll(uint8_t *p, size_t n);

    int fd_ = -1;
    int timeout_ms_ = 1000;
    uint16_t invoke_ = 0;
    std::string prefix_ = "%MB";
    uint16_t max_chunk_ = 1024;  // XGT 연속 읽기/쓰기 1회 최대 1400 B 보다 작게
    std::string err_;
};

}  // namespace sync_exec
