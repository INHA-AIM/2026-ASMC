#include "Lidar/UDPReceive/LiDARReceiver.hpp"
#include <iostream>

LiDARReceiver::LiDARReceiver(const LiDARUDP& st_UDP) {
    // TODO: 나중에 socket 초기화
}

LiDARReceiver::~LiDARReceiver() {
    // TODO: 리소스 해제
}

void LiDARReceiver::start() {
    // TODO: 스레드 시작
}

void LiDARReceiver::stop() {
    // TODO: 스레드 종료
}

boost::optional<std::vector<uint8_t>> LiDARReceiver::get_packet() {
    return boost::none;  // 아직은 더미 반환
}

void* LiDARReceiver::receiver_thread_function(void* context) {
    return nullptr;  // 더미
}

void LiDARReceiver::receive_loop() {
    // TODO: 나중에 데이터 수신 루프 작성
}
