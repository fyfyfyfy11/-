#ifndef LASER_DRV_VANJEE_H
#define LASER_DRV_VANJEE_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include "laser_net.h"
#include "efifo.h"
#include "rtthread_port.h"
#include "laser_time.h"

typedef enum {
    eLaserDrvOk = 0,
    eLaserDrvError = -1,
    eLaserDrvReadDataError = -2,
    eLaserDrvPushDataError = -3
} laser_drv_result_code_t;

typedef enum {
    LASER_TYPE_VANJEE = 0
} laser_drv_type_t;

typedef struct {
    char* drv_name;
    laser_drv_type_t drv_type;
    bool is_init;
    bool is_connected;
    int read_cnt;
    int ok_cnt;
    int error_cnt;
    int reconnect_cnt;
    struct timeval start_time;
    struct timeval end_time;
    long diff_time;
} laser_drv_base_info_t;

typedef struct {
    uint32_t seq;
    struct timeval stamp;
    char* frame_id;
    uint32_t raw_data_count;
    uint8_t* raw_data_buffer;
} laser_drv_raw_data_t;

typedef struct {
    laser_drv_base_info_t base_info;
    laser_drv_raw_data_t raw_data;
    laser_net_t laser_net;
    efifo_t* parase_fifo;
    uint8_t* net_rx_buffer;
    uint8_t* net_tx_buffer;
    uint32_t net_rx_count;
    uint32_t net_tx_count;
} laser_drv_t;

// 设备工作状态
typedef enum {
    LASER_STATUS_IDLE = 0,
    LASER_STATUS_RUNNING = 1,
    LASER_STATUS_ERROR = 2
} laser_device_status_t;

// 设备状态信息结构体
typedef struct {
    bool is_init;
    bool is_connected;
    laser_device_status_t work_status;
    int error_code;
    int read_cnt;
    int ok_cnt;
    int error_cnt;
    int reconnect_cnt;
    long diff_time;
} laser_device_state_info_t;

// 采集任务控制接口
laser_drv_result_code_t laser_device_start(laser_drv_t *laser_obj);
laser_drv_result_code_t laser_device_stop(laser_drv_t *laser_obj);
// 状态查询接口
laser_drv_result_code_t laser_device_get_status(laser_drv_t *laser_obj, laser_device_state_info_t *state_info);

laser_drv_result_code_t laser_drv_vanjee_init(laser_drv_t *laser_obj, const char *drv_name, laser_drv_type_t drv_type, const char *dest_ip, uint16_t dest_port, uint16_t local_port);
laser_drv_result_code_t laser_drv_vanjee_destroy(laser_drv_t *laser_obj);
laser_drv_result_code_t laser_drv_vanjee_reconnect(laser_drv_t *laser_obj);
laser_drv_result_code_t laser_drv_vanjee_read_data(laser_drv_t *laser_obj);
laser_drv_result_code_t laser_drv_vanjee_get_one_frame(laser_drv_t *laser_obj);
laser_drv_result_code_t laser_drv_vanjee_get_raw_data(laser_drv_t *laser_obj, laser_drv_raw_data_t *laser_data);

#endif