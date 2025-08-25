#include "laser_net.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

laser_net_result_code_t laser_net_init(laser_net_t* net, const char* name, const char* dest_ip, uint16_t dest_port, uint16_t local_port) {
    strncpy(net->name, name, sizeof(net->name)-1);
    strncpy(net->dest_ip, dest_ip, sizeof(net->dest_ip)-1);
    net->dest_port = dest_port;
    net->local_port = local_port;
    net->sockfd = -1;
    return eLaserNetOk;
}

laser_net_result_code_t laser_net_connect_udp(laser_net_t* net) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) return eLaserNetError;

    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(net->local_port);

    if (bind(sockfd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        close(sockfd);
        return eLaserNetError;
    }
    net->sockfd = sockfd;
    return eLaserNetOk;
}

laser_net_result_code_t laser_net_recv_some(laser_net_t* net, uint8_t* buf, int bufsize, uint32_t* rx_count) {
    if (net->sockfd < 0) return eLaserNetError;
    int n = recv(net->sockfd, buf, bufsize, MSG_DONTWAIT);
    if (n > 0) {
        *rx_count = n;
        return eLaserNetOk;
    } else {
        *rx_count = 0;
        return eLaserNetError;
    }
}

laser_net_result_code_t laser_net_close(laser_net_t* net) {
    if (net->sockfd >= 0) {
        close(net->sockfd);
        net->sockfd = -1;
    }
    return eLaserNetOk;
}