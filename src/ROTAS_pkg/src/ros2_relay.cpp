/*
    Relay program
    Robotec 2025

    warning, hot garbage code ahead
*/

#ifndef NOMINMAX
    #define NOMINMAX
#endif

#include <portaudio.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <cstdlib>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opus/opus.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
//#include "pkg_test/msg/Struct.hpp"
#pragma comment(lib, "ws2_32.lib")
using namespace std;

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 8000
#define AUDIO_SAMPLE_RATE 48000     // 48kHz
#define AUDIO_FRAME_SIZE 2880       // 2880 bytes = ~120ms
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
#define VIDEO_FPS 30

std::vector<int> cam_ports;

struct RTPHeader{
    uint16_t cc:4;
    uint16_t x:1;
    uint16_t p:1;
    uint16_t version:2;
    uint16_t pt:7;
    uint16_t m:1;
    uint16_t seq;
    uint32_t timestamp;
    uint32_t ssrc;
};

enum class PayloadType : uint8_t {
    VIDEO_MJPEG = 97,
    AUDIO_PCM = 98,
    ROS2_ARRAY = 99
};

std::vector<int> scanWebcams(int num_ports = 5){
    std::vector<int> ports;
    for(int i = 0; i < num_ports; i++){
        cv::VideoCapture cap(i);
        if(cap.isOpened()){
            ports.push_back(i);
            cap.release();
        }
    }
    return ports;
}

int nMap(int n, int minIn, int maxIn, int minOut, int maxOut){
    return float((n - minIn)) / float((maxIn - minIn)) * (maxOut - minOut) + minOut;
}

int nMap(float n, float minIn, float maxIn, float minOut, float maxOut){
    return float((n - minIn)) / float((maxIn - minIn)) * (maxOut - minOut) + minOut;
}

class RTPStreamHandler{    
public:
    RTPStreamHandler(int port, std::string address, PayloadType type){
        // ---- Stream info ---
        stream = new Stream;
        stream->ssrc = 0;
        stream->seq_num = 0 & 0xFFFF;
        stream->timestamp = 0;
        stream->payload_type = type;
        // --- UDP Socket init ---
        // --- send ---
        send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        send_socket_address.sin_family = AF_INET;
        send_socket_address.sin_port = htons(port);
        inet_pton(AF_INET, address.c_str(), &send_socket_address.sin_addr);
        // -- recv --  
        recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        int recv_buff_size = 1024 * 1024;   // 1MB
        setsockopt(recv_socket, SOL_SOCKET, SO_RCVBUF, (char*)&recv_buff_size, sizeof(recv_buff_size));
        recv_socket_address.sin_family = AF_INET;
        recv_socket_address.sin_port = htons(port + 1);
        recv_socket_address.sin_addr.s_addr = INADDR_ANY;
        ::bind(recv_socket, (struct sockaddr*)&recv_socket_address, socket_address_size);
        
    }
    ~RTPStreamHandler(){
        closesocket(send_socket);
        closesocket(recv_socket);
    }
    void destroy(){
        delete this;
    }
    template <typename T> void sendPacket(std::vector<T> data){
        // --- RTP header info ---
        RTPHeader header;
        header.version = 2;
        header.p = 0;
        header.x = 0;
        header.cc = 0;
        header.m = 1;
        header.pt = static_cast<uint8_t>(stream->payload_type);
        header.seq = stream->seq_num++;
        header.timestamp = stream->timestamp;
        header.ssrc = stream->ssrc;
        stream->timestamp += 100; // fix this shit
        // --- Prepare packet for send ---
        std::vector<char> packet((data.size() * sizeof(T)) + sizeof(RTPHeader));
        std::memcpy(packet.data(), &header, sizeof(RTPHeader));
        std::memcpy(packet.data() + sizeof(RTPHeader), data.data(), data.size() * sizeof(T));
        // --- Simulated network degradation (lowkey trash implementation but idc) ---
        /*
        this_thread::sleep_for(std::chrono::milliseconds(rand() % 150));    // ~100ms latency
        if(rand() % 10 == 0) return;    // ~10% packet loss
        */
        if(sendto(send_socket, packet.data(), packet.size(), 0, (struct sockaddr*)&socket_address, socket_address_size) == SOCKET_ERROR){
            std::cout << "[w] Packet send failed. Winsock error: " << WSAGetLastError() << "\n";
        }
    }
    bool handshake(){
        // -- WIP, untested --
        /*
        std::vector<char> packet(cam_ports.size() * sizeof(int));
        std::memcpy(packet.data(), cam_ports.data(), packet.size());
        int handshake_msg = -1;
        while(true){
            sendto(send_socket, packet.data(), packet.size(), 0, (struct sockaddr*)&socket_address, socket_address_size);
            fd_set read_set;
            FD_ZERO(&read_set);
            FD_SET(recv_socket, &read_set);
            timeval timeout{ 0, 1000000 };
            int result = select(0, &read_set, nullptr, nullptr, &timeout);
            if(result > 0){
                int bytes_received = recvfrom(recv_socket, (char*)&handshake_msg, sizeof(int), 0, (struct sockaddr*)&socket_address, &socket_address_size);
                if(bytes_received < sizeof(int)){
                    std::cout << "[e] Handshake error, bytes received: " << bytes_received << "\n";
                    continue;
                }
                else if(handshake_msg != 200){
                    std::cout << "[e] Wrong handshake message received: " << handshake_msg << "\n";
                    return false;
                }
                return true;
            }
            else{
                std::cout << "[i] Waiting for server...\n";
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        */
        return false;
    }
    std::vector<int> recvPacket(){
        std::vector<char> packet(4096);
        std::cout << "before recvfrom\t";
        int bytes_received = recvfrom(recv_socket, packet.data(), packet.size(), 0, (struct sockaddr*)&recv_socket_address, &socket_address_size);
        std::cout << "after, received: " << bytes_received << "\n";
        if(bytes_received == SOCKET_ERROR){
            std::cout << "[e] Packet recv failed. Winsock error: " << WSAGetLastError() << "\n";
            return {};
        }
        std::vector<int> data(bytes_received / sizeof(int));
        std::memcpy(data.data(), packet.data(), bytes_received);
        return data;
    }
private:
    struct Stream{
        uint32_t ssrc;
        uint16_t seq_num;
        uint32_t timestamp;
        PayloadType payload_type;
    };
    Stream* stream;
    SOCKET send_socket;
    SOCKET recv_socket;
    sockaddr_in send_socket_address;
    sockaddr_in recv_socket_address;
    int socket_address_size = sizeof(send_socket_address);
};

class RelayNode : public rclcpp::Node{
public:
    RelayNode() : Node("relay_node"){
        // --- Sockets + ROS2 startup ---
        std::cout << "[i] Starting node...\n";
        WSAStartup(MAKEWORD(2, 2), &wsa_data);
        base_socket.target_socket = new RTPStreamHandler(SERVER_PORT, SERVER_IP, PayloadType::ROS2_ARRAY);
        std::cout << "stream started\n";
        //audio_socket = new RTPStreamHandler(SERVER_PORT + 2, SERVER_IP, PayloadType::AUDIO_PCM);
        //video_socket = new RTPStreamHandler(SERVER_PORT+2, SERVER_IP, PayloadType::VIDEO_MJPEG);
        /*
        for(int i = 0; i < cam_ports.size(); i++){
            SocketStruct socket_struct;
            socket_struct.target_socket = new RTPStreamHandler(SERVER_PORT + (2*i) + 4, SERVER_IP, PayloadType::VIDEO_MJPEG);
            video_sockets.push_back(std::move(socket_struct));
        }
        */
        base_socket.is_recv_running.store(true);
        base_socket.is_send_running.store(true);
        //is_video_running = true;
        /*
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].is_send_running.store(true);
            video_sockets[i].is_recv_running.store(true);
        }
        */
        stop_flag.store(false);
        // --- Opus + PortAudio startup ---
        /*
        int opus_error;
        opus_encoder = opus_encoder_create(AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_AUDIO, &opus_error);
        Pa_Initialize();
        Pa_OpenDefaultStream(&stream, 1, 0, paInt16, AUDIO_SAMPLE_RATE, AUDIO_FRAME_SIZE, audioCallback, this);     // Also initializes audio thread
        Pa_StartStream(stream);
        */
        // --- Threads startup - Program begins ---
        //video_thread = std::thread(&RelayNode::videoCallback, this);
        base_socket.send_thread = std::thread(&RelayNode::baseCallback, this);
        base_socket.recv_thread = std::thread(&RelayNode::recvCallback, this, PayloadType::ROS2_ARRAY, 0);
        /*
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].send_thread = std::thread(&RelayNode::videoCallback, this, i);
            video_sockets[i].recv_thread = std::thread(&RelayNode::recvCallback, this, PayloadType::VIDEO_MJPEG, i);
        }  
        */
        ros2_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>("float_topic", 10, std::bind(&RelayNode::topicCallback, this, std::placeholders::_1));
        std::cout << "[i] Setup done\n";
    }
    ~RelayNode(){
        // --- Stop loops + join threads + destroy objects ---
        std::cout << "[i] Closing node...\n";
        stop_flag.store(true);
        is_audio_running.store(false);
        base_socket.is_recv_running.store(false);
        base_socket.is_send_running.store(false);
        /*
        for(int i = 0; i < video_sockets.size(); i++){
            video_sockets[i].is_recv_running.store(false);
            video_sockets[i].is_send_running.store(false);
        }
        */
        //is_video_running = false;
        Pa_StopStream(stream);
        Pa_CloseStream(stream);
        Pa_Terminate();
        opus_encoder_destroy(opus_encoder);
        if(base_socket.recv_thread.joinable()) base_socket.recv_thread.join();
        if(base_socket.send_thread.joinable()) base_socket.send_thread.join();
        //if(video_thread.joinable()) video_thread.join();
        /*
        for(int i = 0; i < video_sockets.size(); i++){
            if(video_sockets[i].recv_thread.joinable()) video_sockets[i].recv_thread.join();
            if(video_sockets[i].send_thread.joinable()) video_sockets[i].send_thread.join();
            video_sockets[i].target_socket->destroy();
        }
        */
        //delete video_socket;
        base_socket.target_socket->destroy();
        delete audio_socket;
        WSACleanup();
    }
private:
    static int audioCallback(const void* input, void* output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData) {
        // --- PortAudio callback requires a static function pointer, so this is needed as a middleman ---
        RelayNode* self = static_cast<RelayNode*>(userData);
        return self->audioProcess(input, output, frameCount, timeInfo, statusFlags);
    }
    int audioProcess(const void* input, void* output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags){
        // --- Callback runs again after paContinue is returned, so no loop required ---
        if(stop_flag.load()) return paComplete;
        if(!input || !is_audio_running.load()) return paContinue;
        // --- Opus encode + send ---
        unsigned char encoded_data[4096];
        int encoded_size = opus_encode(opus_encoder, (const opus_int16*)input, AUDIO_FRAME_SIZE, encoded_data, sizeof(encoded_data));
        if(encoded_size > 0){
            std::vector<unsigned char> packet(encoded_size);
            std::memcpy(packet.data(), encoded_data, encoded_size);
            audio_socket->sendPacket(packet);
        }
        return paContinue;
    }
    void videoCallback(int id = 0){
        // -- WIP, untested, should probably work tho --
        // --- Webcam setup ---
        cv::VideoCapture cap(cam_ports[id]);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
        if(!cap.isOpened()){
            std::cout << "[e] Cannot open webcam\n";
            return;
        }
        cv::Mat frame;
        std::vector<unsigned char> compressed_data;
        // --- MJPEG encode + send ---
        while(!stop_flag.load()){
            while(video_sockets[id].is_send_running.load()){
                cap >> frame;
                if(frame.empty()) break;
                cv::imencode(".jpg", frame, compressed_data, {cv::IMWRITE_JPEG_QUALITY, 50});
                //video_socket->sendPacket(compressed_data);
                video_sockets[id].target_socket->sendPacket(compressed_data);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000 / VIDEO_FPS));   // May or may not be necessary (due to dropped frames)
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        cap.release();
    }
    void recvCallback(PayloadType type, int id = 0){
        // -- WIP, halfway done, need to implement specific data callbacks --
        while(!stop_flag.load()){
            if(type == PayloadType::ROS2_ARRAY){
                while(base_socket.is_recv_running.load()){
                    std::vector<int> data = base_socket.target_socket->recvPacket();
                    if(data.size() <= 0 || data[0] != 200){
                        std::cout << "[e] Corrupted packet received on base socket\n";
                        continue;
                    }
                    // do stuff with recv data
                }
            }
            else{
                while(video_sockets[id].is_recv_running.load()){
                    std::vector<int> data = video_sockets[id].target_socket->recvPacket();
                    if(data.size() <= 0 || data[0] != 200){
                        std::cout << "[e] Corrupted packet received on video" << id << " socket\n";
                        continue;
                    }
                    video_sockets[id].is_send_running.store(data[1]);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    void topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // --- Mutex lock for thread-safe updates ---
        std::lock_guard<std::mutex> lock(data_mutex);
        latest_data = msg->data;
    }
    void baseCallback(){
        while(base_socket.is_send_running.load()){
            // --- Mutex lock for thread-safe access ---
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                base_socket.target_socket->sendPacket(latest_data);
            }
            this_thread::sleep_for(chrono::milliseconds(500));  // Immediate updates are not necessary
        }
    }
    struct SocketStruct{
        RTPStreamHandler* target_socket;
        std::thread send_thread;
        std::thread recv_thread;
        std::atomic<bool> is_send_running;
        std::atomic<bool> is_recv_running;
        // --- Thingamajig to transfer std::thread ownership ---
        SocketStruct() : target_socket(nullptr) {}
        SocketStruct(SocketStruct&& other) noexcept
            : recv_thread(std::move(other.recv_thread)),
              send_thread(std::move(other.send_thread)),
              target_socket(std::move(other.target_socket)){}
        SocketStruct& operator=(SocketStruct&& other) noexcept {
            if(this != &other){
                recv_thread = std::move(other.recv_thread);
                send_thread = std::move(other.send_thread);
                target_socket = std::move(other.target_socket);
            }
            return *this;
        }
        SocketStruct(const SocketStruct&) = delete;
        SocketStruct& operator=(const SocketStruct&) = delete;
    };
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ros2_subscription;
    std::vector<float> latest_data;
    std::vector<int> cam_ports;
    std::mutex data_mutex;
    std::mutex flag_mutex;
    std::atomic<bool> is_audio_running;
    std::atomic<bool> stop_flag;
    PaStream* stream;
    PaError err;
    OpusEncoder* opus_encoder;
    WSADATA wsa_data;
    RTPStreamHandler* audio_socket;
    SocketStruct base_socket;
    std::vector<SocketStruct> video_sockets;
};

int main(int argc, char** argv){
    //cam_ports = scanWebcams();
    // --- RelayNode handles everything ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
