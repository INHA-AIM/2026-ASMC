#pragma once
#include <Global/Global.hpp>
#include <boost/optional.hpp>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <atomic>
#include <vector>
#include <pthread.h>

using std::vector;
using std::string;
using std::atomic;

class LiDARReceiver {
public:
    LiDARReceiver(const LiDARUDP& st_UDP);
    ~LiDARReceiver();

    void start();
    void stop();

    // 수신 루프 및 스레드 함수
    void receive_loop();
    static void* receiver_thread_function(void* context);

    // 수신된 패킷 가져오기
    boost::optional<vector<uint8_t>> get_packet();

private:
    int socket_fd_;  // 소켓 디스크립터
    struct sockaddr_in server_addr_;  // 바인딩 주소
    atomic<bool> stop_flag_{false};   // 종료 플래그
    pthread_t receiver_thread_;       // 수신 스레드
    pthread_mutex_t packet_mutex_;    // 패킷 접근 보호 뮤텍스

    vector<uint8_t> completed_packet_data_;  // 수신 완료된 패킷
};

