#include <opencv2/opencv.hpp>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#pragma comment(lib, "ws2_32.lib")

using namespace std;
using namespace cv;

string SERVER_IP = "127.0.0.1";
vector<int> CAMS{0};
int PORT = 8080, MODE = 0, WIDTH = 1280, HEIGHT = 720, QUALITY = 75, ROS2_PACKET_SIZE = 5;
bool VERBOSE = false;
vector<int> ros2_data(ROS2_PACKET_SIZE, 0);

int args(int argc, char* argv[]);
bool sendPacket(SOCKET socket_fd, vector<uchar>& buffer, int packetNumber);
bool handshake(SOCKET socket_fd);
void cnlog(const string& str, int lvl);
void ros2Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
void ros2Thread();

int main(int argc, char* argv[]){
    if (args(argc, argv)) return -1;
    cnlog("[i] Initializing client...", 2);

    WSADATA wsaData;
    u_long socket_mode = 1;
    vector<SOCKET> socket_fds(CAMS.size());
    vector<VideoCapture> sources(CAMS.size());
    vector<vector<uchar>> buffers(CAMS.size());
    vector<int> packet_numbers(CAMS.size(), 0);

    if(WSAStartup(MAKEWORD(2, 2), &wsaData) != 0){
        cnlog("[e] Failed to initialize Winsock", 0);
        return -1;
    }

    cnlog("[i] Initializing ros2 node...", 2);
    thread rosThread(ros2Thread);

    cnlog("[i] Initializing capture devices...", 2);
    for (int i = 0; i < CAMS.size(); i++) {
        sources[i].open(CAMS[i]);
        if(!sources[i].isOpened()){
            cnlog("[e] Could not open source " + to_string(i), 0);
            WSACleanup();
            return -1;
        }
        sources[i].set(CAP_PROP_FRAME_WIDTH, WIDTH);
        sources[i].set(CAP_PROP_FRAME_HEIGHT, HEIGHT);

        socket_fds[i] = socket(AF_INET, SOCK_STREAM, 0);
        if(socket_fds[i] == INVALID_SOCKET){
            cnlog("[e] Could not create socket for source " + to_string(i), 0);
            WSACleanup();
            return -1;
        }
        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(PORT + i);
        inet_pton(AF_INET, SERVER_IP.c_str(), &server_addr.sin_addr);

        connect(socket_fds[i], (sockaddr*)&server_addr, sizeof(server_addr));
        if(i == 0 && !handshake(socket_fds[i])){
            for(int j = 0; j < CAMS.size(); j++){
                closesocket(socket_fds[i]);
                sources[j].release();
            }
            WSACleanup();
            return -1;
        }
    }

    cnlog("[i] Starting stream...", 2);
    while(1){
        for(int i = 0; i < CAMS.size(); i++){
            Mat frame;
            sources[i] >> frame;
            if(frame.empty()){
                cnlog("[w] Source " + to_string(i) + " captured an empty frame", 1);
                continue;
            }
            buffers[i].clear();
            imencode(".jpg", frame, buffers[i], {IMWRITE_JPEG_QUALITY, QUALITY});
            if(!sendPacket(socket_fds[i], buffers[i], packet_numbers[i]++)){
                cnlog("[w] Failed to send frame from source " + to_string(i), 1);
                return 1;
            }
            packet_numbers[i] %= 100;
        }
        Sleep(20);
    }

    cnlog("[i] Shutting down client...", 2);
    for(int i = 0; i < CAMS.size(); i++){
        sources[i].release();
        closesocket(socket_fds[i]);
    }
    rosThread.join();
    WSACleanup();
    return 0;
}

void cnlog(const string& str, int lvl) {
    if(VERBOSE || lvl == 0) cout << str << '\n';
}

bool handshake(SOCKET socket_fd) {
    cnlog("[i] Starting handshake...", 2);

    int handshakeMessage[] = {0, MODE, CAMS.size()}, handshakeAck = 0;
    if(send(socket_fd, (char*)handshakeMessage, sizeof(handshakeMessage), 0) == SOCKET_ERROR){
        cnlog("[e] Could not send handshake", 0);
        return false;
    }

    while(1){
        if(recv(socket_fd, (char*)&handshakeAck, sizeof(handshakeAck), 0) <= 0){ 
            if(WSAGetLastError() == WSAEWOULDBLOCK){
                Sleep(50);
                continue; 
            }
            else{
                cnlog("[e] Did not receive handshake acknowledgment. Code: " + to_string(WSAGetLastError()), 0);
                return false;
            }
        }
        else break;
    }
    if(handshakeAck != 400){
        cnlog("[e] Invalid handshake response: " + to_string(handshakeAck), 0);
        return false;
    }

    cnlog("[i] Handshake complete", 2);
    return true;
}

bool sendPacket(SOCKET socket_fd, vector<uchar>& buffer, int packet_number){
    int metadata[ROS2_PACKET_SIZE+2] = {buffer.size(), packet_number};
    for(int i = 0; i < ROS2_PACKET_SIZE; i++){
        metadata[i+2] = ros2_data[i];
    }
    if(send(socket_fd, (char*)metadata, sizeof(metadata), 0) == SOCKET_ERROR){
        cnlog("[e] Metadata send failed. Code: " + to_string(WSAGetLastError()), 0);
        return false;
    }
    if(send(socket_fd, (char*)buffer.data(), buffer.size(), 0) == SOCKET_ERROR){
        cnlog("[e] Frame send failed. Code: " + to_string(WSAGetLastError()), 0);
        return false;
    }
    return true;
}

void ros2Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
    if(msg->data.size() == R0S2_PACKET_SIZE){
        ros_received_floats = msg->data;
        stringstream stream;
        stream << "[ros2 recv] ";
        for(int i = 0; i < ROS2_PACKET_SIZE; i++){
            stream << i << ": " << ros2_data[i] << " ";
        }
        cnlog(stream.str(), 2);
    }
    else cnlog("[w] ros2 topic sync discrepancy", 1);
}

void ros2Thread(){
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("relay_node");
    auto subscription = node->create_subscription<std_msgs::msg::Int32MultiArray>("int_topic", 10, ros2Callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
}

int args(int argc, char* argv[]) {
    for(int i = 1; i < argc; i++){
        string arg = argv[i];
        if(arg == "--ip" || arg == "-i"){
            if(i+1 < argc) SERVER_IP = argv[++i];
            else{
                cout << "[e] --ip requires an ip address\n";
                return 1;
            }
        }
        else if(arg == "--port" || arg == "-p"){
            if(i+1 < argc) PORT = atoi(argv[++i]);
            else{
                cout << "[e] --port requires a port number\n";
                return 1;
            }
        }
        else if(arg == "--width" || arg == "-w"){
            if(i+1 < argc) WIDTH = atoi(argv[++i]);
            else{
                cout << "[e] --width requires a horizontal resolution\n";
                return 1;
            }
        }
        else if(arg == "--height" || arg == "-h"){
            if(i+1 < argc) HEIGHT = atoi(argv[++i]);
            else{
                cout << "[e] --height requires a vertical resolution\n";
                return 1;
            }
        }
        else if(arg == "--cams" || arg == "-c"){
            int n = 0;
            if(i+3 <= argc){
                n = atoi(argv[i+1]);
                if(argc < i+n){
                    cout << "[e] Incomplete camera list\n";
                    return 1;
                }
                CAMS.clear();
                for(int j = 0; j < n; j++){
                    CAMS.push_back(atoi(argv[i+j+2]));
                }
            }
            else{
                cout << "[e] --cams requires a camera list\n";
                return 1;
            }
            i += n+1;
        }
        else if(arg == "--mode" || arg == "-m"){
            if(i+1 < argc) MODE = atoi(argv[++i]);
            else{
                cout << "[e] --mode requires a number\n";
                return 1;
            }
        }
        else if(arg == "--quality" || arg == "-q"){
            if(i+1 < argc) QUALITY = atoi(argv[++i]);
            else{
                cout << "[e] --quality requires a camera amount\n";
                return 1;
            }
        }
        else if(arg == "--help" || arg == "-H"){
            cout << "Options\n"
                 << "  -v\t\t\t= Verbose output\n"
                 << "  -H\t\t\t= Displays available options\n"
                 << "  -i <address>\t\t= Server IP address\n"
                 << "  -p <number>\t\t= Server TCP port number\n"
                 << "  -w <pixels>\t\t= Video horizontal resolution\n"
                 << "  -h <pixels>\t\t= Video vertical resolution\n"
                 << "  -c <number> <list>\t\t= Camera inputs to transmit\n"
                 << "  -q <number>\t\t= Transmission video quality (0-100)\n";
            return 1;
        }
        else if(arg == "--verbose" || arg == "-v") VERBOSE = true;
        else{
            cout << "[e] Invalid argument detected\n\nOptions\n"
                 << "  -v\t\t\t= Verbose output\n"
                 << "  -H\t\t\t= Displays available options\n"
                 << "  -i <address>\t\t= Server IP address\n"
                 << "  -p <number>\t\t= Server TCP port number\n"
                 << "  -w <pixels>\t\t= Video horizontal resolution\n"
                 << "  -h <pixels>\t\t= Video vertical resolution\n"
                 << "  -c <number> <list>\t\t= Camera inputs to transmit\n"
                 << "  -q <number>\t\t= Transmission video quality (0-100)\n";
            return 1;
        }
    }
    return 0;
}
