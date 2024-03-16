#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h> 
#include <fcntl.h> 
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h> 
#include <sys/epoll.h> 
#include <netinet/in.h> 
#include <arpa/inet.h> 
#include <time.h>
#include <thread>

#include <ntrip_util.h>

#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_spp.hpp>


std::string server_ip;
int server_port;
std::string mountpoint;
std::string user;
std::string passwd;

// RTCM receive related
bool rtcm_request = false; 
bool rtcm_receive = false; 
bool client_connected = false; 
bool rtcm_transfer = false;

int m_sock;
int ret;
char recv_buf[1024] = {0};
char request_data[1024] = {0};
char userinfo_raw[48] = {0};
char userinfo[64] = {0};
char gpgga[256];
int rtcm_len = 0;

struct sockaddr_in server_addr;

// Port transfer related
int serv_sock;
int clnt_sock;
struct sockaddr_in serv_addr;
struct sockaddr_in clnt_addr;
socklen_t clnt_addr_size;
int transfer_port;
std::string transfer_ip;

char test_msg[]="Hello World!";


uint8_t calculateChecksum(const std::string& sentence) {
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence.size(); ++i) {
        if (sentence[i] == '*') {
            break;
        }
        checksum ^= static_cast<uint8_t>(sentence[i]);
    }
    return checksum;
}

void receivePVTCallback(const gnss_comm::GnssPVTSolnMsgConstPtr &pvt_msg) {
    
    static int cnt = -1;
    
    // Parse "/ublox_f9p/receive_pvt" topic
    double epoch[6];
    gnss_comm::time2epoch(gnss_comm::gpst2time(pvt_msg->time.week, pvt_msg->time.tow), epoch);
    // printf("UTC_time: %04.f-%02.f-%02.f %02.f:%02.f:%02.2f\n", epoch[0], epoch[1], epoch[2], epoch[3], epoch[4], epoch[5]);

    // LLA
    double latitude = pvt_msg->latitude;          
    double longitude = pvt_msg->longitude;        
    double altitude = pvt_msg->altitude;     
    // printf("Lat:%lf, lon:%lf, alt:%lf\n", latitude, longitude, altitude);

    int lat_degrees = static_cast<int>(latitude); 
    double lat_remainder = (latitude - lat_degrees) * 60; 

    int lon_degrees = static_cast<int>(longitude); 
    double lon_remainder = (longitude - lon_degrees) * 60; 
    // std::cout << lat_degrees << "~" << lat_remainder << std::endl;
    // std::cout << lon_degrees << "~" << lon_remainder << std::endl;

    int sat_num = 0;
    if(pvt_msg->num_sv < 12)
        sat_num = pvt_msg->num_sv;
    else 
        sat_num = 12;

    // HDOP，An approximate solution since the drawing does not contain "HDOP" messages
    double hdop = pvt_msg->p_dop/2;
    if(hdop < 0.55)
        hdop = 0.55;
    double msl = -28.2; //Height of the geoid，a city of Shaanxi Province, China.

    // Encapsulate GGA messages
    //                     |UTC            |lat             |lon           |qa|sats|hdop|alt|  |     | | 
    sprintf(gpgga, "$GPGGA,%02.f%02.f%05.2f,%02d%010.7lf,%c,%03d%010.7lf,%c,1,%02d,%.2f,%.3f,M,-28.2,M,,*",
            epoch[3], epoch[4], epoch[5], 
            lat_degrees, lat_remainder, latitude  < 0 ? 'S' : 'N', 
            lon_degrees, lon_remainder, longitude < 0 ? 'W' : 'E',
            sat_num, hdop, altitude);
    uint8_t c_sum = calculateChecksum(gpgga);
    // printf("%x ", c_sum);
    sprintf(gpgga + strlen(gpgga), "%02X\r\n", c_sum); 
    // printf("%s\n", gpgga); 

    // Frequency control
    cnt++;
    if(cnt % 29 != 0) 
        return;
    
    if (!rtcm_request)
        rtcm_request = true; 
}

void rtcm_receiver() {
    while(true) {
        if (rtcm_request) {
            m_sock = socket(AF_INET, SOCK_STREAM, 0); //int socket(int domain, int type, int protocol) 
            if(m_sock == -1) {
                ROS_ERROR("Create socket fail");
                exit(1);
            }
            
            /* 1. Connect to caster. */
            ret = connect(m_sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in)); 
            if(ret < 0){
                ROS_ERROR("Connect caster fail!");
                exit(1);
            }
            
            /* 2. Send request data. */
            ret = send(m_sock, request_data, strlen(request_data), 0);		
            if(ret < 0){
                ROS_ERROR("Send request fail!");
                exit(1);
            }
            
            // 3. request + GGA
            ret = recv(m_sock, (void *)recv_buf, sizeof(recv_buf), 0);
            if(ret > 0 && !strncmp(recv_buf, "ICY 200 OK\r\n", 12)){
                ret = send(m_sock, gpgga, strlen(gpgga), 0);
                if(ret < 0){
                    ROS_ERROR("Send GPGGA message FAILED!");
                }
                ROS_INFO("send gpgga data ok");
            }
            rtcm_receive = true;
            rtcm_request = false;
        }

        if(rtcm_receive) {
            // 1. receive RTCM
            ret = recv(m_sock, (void *)recv_buf, sizeof(recv_buf), 0);
            if(ret > 0){
                ROS_INFO("RTCM receive OK! recv data:[%d] ", ret);
                rtcm_len = ret;
                if (!rtcm_transfer)
                    rtcm_transfer = true;
                // std::cout << recv_buf << std::endl;
            }else{
                ROS_INFO("RTCM receive FAILED! Maybe Remote socket closed!!!");
                ros::Duration(1.0).sleep();
                // break;
            }
        }
    }
}

void port_transfer() {
    
    serv_sock=socket(PF_INET, SOCK_STREAM, 0);
    if(serv_sock == -1){
        ROS_ERROR("Create socket error!");
        exit(1);
    }
        
    // set SO_REUSEADDR option
    int option = 1;
    if(setsockopt(serv_sock, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option)) == -1) {
        ROS_ERROR("set sockopt error");
        exit(1);
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family=AF_INET;
    inet_pton(AF_INET, transfer_ip.c_str(), &serv_addr.sin_addr); 
    serv_addr.sin_port=htons(transfer_port);

    if(bind(serv_sock, (struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1) {
        ROS_ERROR("bind error");
        exit(1);
    }
    ROS_INFO("The TCP service is started, IP address: %s, Port: %d", inet_ntoa(serv_addr.sin_addr), ntohs(serv_addr.sin_port));
    
relisten:    
    if(listen(serv_sock, 5)==-1) {
        ROS_ERROR("listen error");
        exit(1);
    }
    ROS_INFO("Start listening the client...");

    clnt_addr_size=sizeof(clnt_addr);
    clnt_sock = accept(serv_sock, (struct sockaddr*)&clnt_addr,&clnt_addr_size);
    if(clnt_sock==-1) {
        ROS_ERROR("Client accept error");
        exit(1);
    }
    client_connected = true;
    ROS_INFO("Client access!--------");
        
    while(client_connected) {

        if(rtcm_transfer) {
            if (recv(clnt_sock, NULL, 0, MSG_PEEK | MSG_DONTWAIT) == 0) {
                ROS_ERROR("The client is disconnected! Enters the listening mode");
                close(clnt_sock);
                client_connected = false; 
                goto relisten;
            }

            int send_len = write(clnt_sock, recv_buf, rtcm_len);
            if (send_len <= -1) {
                ROS_ERROR("RTCM tranfer Error!"); 
            } else {
                ROS_INFO("RTCM tranfer success! Date_bytes:[%d]", rtcm_len);
            }
            rtcm_transfer = false;
        }
        
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ntrip_client");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string >("server_ip", server_ip, "127,0,0,1");
    private_nh.param<int>("server_port", server_port, 8002);
    private_nh.param<std::string>("mountpoint", mountpoint, "mountpoint");
    private_nh.param<std::string>("user", user, "username");
    private_nh.param<std::string>("passwd", passwd, "password");

    private_nh.param<std::string>("transfer_ip", transfer_ip, "127.0.0.1");
    private_nh.param<int>("transfer_port", transfer_port, 3503);
    

    ROS_INFO("Port:%d", transfer_port);

    std::string config_filename;
    nh.param<std::string>("yaml_file", config_filename, "rtcm_config.yaml");
    
	memset(&server_addr, 0, sizeof(struct sockaddr_in));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(server_port);
	server_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
	
	/* Generate base64 encoding of user and passwd. */
    sprintf(userinfo_raw, "%s:%s", user.c_str(), passwd.c_str());
	base64_encode(userinfo_raw, userinfo);
 
 	/* Generate request data format of ntrip. */
    sprintf(request_data,
    "GET /%s HTTP/1.1\r\n"
    "User-Agent: %s\r\n"
    "Accept: */*\r\n"
    "Connection: close\r\n"
    "Authorization: Basic %s\r\n"
    "\r\n"
    , mountpoint.c_str(), client_agent, userinfo);
    
    ros::Subscriber sub = nh.subscribe("/ublox_driver/receiver_pvt", 1000, receivePVTCallback);


    while(ros::ok()) {
        
        std::thread RTCM_receive_process{rtcm_receiver};
        std::thread port_transfer_process{port_transfer};

        ros::spin();
    }

	close(m_sock);
    close(clnt_sock);
    close(serv_sock);
    return 0;
}
