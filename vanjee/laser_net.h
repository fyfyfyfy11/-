#ifndef LASER_NET_H
#define LASER_NET_H
#include <stdint.h>

typedef enum {
    eLaserNetOk = 0,
    eLaserNetError = -1
} laser_net_result_code_t;

typedef struct {
    char name[32];
    char dest_ip[32];
    uint16_t dest_port;
    uint16_t local_port;
    int sockfd;
} laser_net_t;

laser_net_result_code_t laser_net_init(laser_net_t* net, const char* name, const char* dest_ip, uint16_t dest_port, uint16_t local_port);
laser_net_result_code_t laser_net_connect_udp(laser_net_t* net);
laser_net_result_code_t laser_net_recv_some(laser_net_t* net, uint8_t* buf, int bufsize, uint32_t* rx_count);
laser_net_result_code_t laser_net_close(laser_net_t* net);

#endif