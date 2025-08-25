/**
 * @file    laser_drv_vanjee.c
 * @brief   万集数据处理
 * @details 
 * @par Copyright (C):
 *          HangZhou JiaZhi Science and Technology Ltd. All Rights Reserved.
 * @par Encoding:
 *          UTF-8
 * @par Modification:
 * -# Date    -#Author     -#details
 * 20230904     pez         create
 */

/* =============================================================================
 *                              header files
 * ========================================================================== */

#include "laser_drv_vanjee.h"
#include "rtthread_port.h"

/* =============================================================================
 *                            macro definition
 * ========================================================================== */



//#define DATA_BUFFER_SIZE 1500
#define DATA_BUFFER_SIZE 8192

#define NET_DEFAULT_PORT 2110

/* =============================================================================
 *                        static function definition
 * ========================================================================== */

static unsigned int check_xor(unsigned char *recv_buf, int recv_len);

/* =============================================================================
 *                             global function
 * ========================================================================== */

/**
 * vanjee初始化接口.
 *
 * @param[in]
 *      drv_name  : 设备名称
 *      drv_type  : 设备类型
 *      dest_ip   : 目标地址
 *      dest_port : 目标端口
 * @param[out]
 *      laser_obj : 激光相关参数
 * @return
 *      返回错误码
 */
laser_drv_result_code_t laser_drv_vanjee_init(laser_drv_t *laser_obj, const char *drv_name, laser_drv_type_t drv_type, const char *dest_ip, uint16_t dest_port, uint16_t local_port)
{
    laser_net_result_code_t net_ret = eLaserNetError;

    /* 判断输出参数有效性 */
    if(laser_obj == NULL) 
    {
        LOG_E("[%s]激光初始化错误: laser_obj == NULL !", drv_name);
        return eLaserDrvError;
    }

    /* 初始化参数默认值 */
    rt_memset(&laser_obj->base_info, 0u, sizeof(laser_drv_base_info_t));
    laser_obj->base_info.drv_name = (char *)rt_malloc(20);
    rt_strcpy(laser_obj->base_info.drv_name, drv_name);
    laser_obj->base_info.drv_type = drv_type;

    rt_memset(&laser_obj->raw_data, 0u, sizeof(laser_drv_raw_data_t));
    laser_obj->raw_data.frame_id = laser_obj->base_info.drv_name;
    laser_obj->raw_data.raw_data_buffer = NULL;
    laser_obj->raw_data.raw_data_buffer = (uint8_t *)rt_malloc(DATA_BUFFER_SIZE);
    if(laser_obj->raw_data.raw_data_buffer == NULL)
    {
        LOG_E("[%s]激光数据缓存申请失败 !", drv_name);
        return eLaserDrvError;
    }

    /* 申请网络数据存储空间 */
    laser_obj->net_rx_count = 0;
    laser_obj->net_rx_buffer = NULL;
    laser_obj->net_rx_buffer = (uint8_t *)rt_malloc(DATA_BUFFER_SIZE);
    if(laser_obj->net_rx_buffer == NULL)
    {
        LOG_E("[%s]激光接收缓存申请失败 !", drv_name);
        return eLaserDrvError;
    }
    laser_obj->net_tx_count = 0;
    laser_obj->net_tx_buffer = NULL;
    laser_obj->net_tx_buffer = (uint8_t *)rt_malloc(DATA_BUFFER_SIZE);
    if(laser_obj->net_tx_buffer == NULL)
    {
        LOG_E("[%s]激光数据处理缓存申请失败 !", drv_name);
        return eLaserDrvError;
    }

    /* 初始化网络参数 */
    net_ret = laser_net_init(&laser_obj->laser_net, laser_obj->base_info.drv_name, dest_ip, dest_port, local_port);
    if(net_ret != eLaserNetOk)
    {
        LOG_E("[%s]激光网络初始化错误: net_ret=%d !", drv_name, net_ret);
        return eLaserDrvError;
    }

    /* 根据端口选择连接方式 */
    if(dest_port == NET_DEFAULT_PORT)
    {
        net_ret = laser_net_connect_udp(&laser_obj->laser_net);
        if(net_ret != eLaserNetOk)
        {
            LOG_E("[%s]激光网络连接失败: net_ret=%d !", drv_name, net_ret);
            return eLaserDrvError;
        }
    }
    else
    {
        LOG_E("[%s]激光端口选择错误: dest port = %d(2110 - udp) !", drv_name, dest_port);
        return eLaserDrvError;
    }

    /* 初始化解析队列 */
    laser_obj->parase_fifo = efifo_creat(laser_obj->base_info.drv_name, DATA_BUFFER_SIZE*4, true);
    efifo_clean(laser_obj->parase_fifo);

    /* 设置激光初始化完成标志 */
    laser_obj->base_info.is_init = true;
    laser_obj->base_info.is_connected = true;
    LOG_I("[%s]激光初始化成功 .", drv_name);

    // 更新起始时间
    gettimeofday(&laser_obj->base_info.start_time, NULL);

    return eLaserDrvOk;
}

/**
 * vanjee资源释放.
 *
 * @param[in]
 * @param[out]
 *      laser_obj : 激光相关参数
 * @return
 *      返回错误码
 */
laser_drv_result_code_t laser_drv_vanjee_destroy(laser_drv_t *laser_obj)
{
    RT_ASSERT(laser_obj != NULL);

    if(laser_obj->base_info.is_init) 
    {
        /* 释放网络资源 */
        laser_net_close(&laser_obj->laser_net);

        /* 释放网络数据缓存资源 */
        rt_free(laser_obj->net_rx_buffer);
        laser_obj->net_rx_buffer = NULL;
        laser_obj->net_rx_count = 0;
        rt_free(laser_obj->net_tx_buffer);
        laser_obj->net_tx_buffer = NULL;
        laser_obj->net_tx_count = 0;
        efifo_destroy(laser_obj->parase_fifo);
        /* 初始化驱动信息 */
        laser_obj->base_info.read_cnt = 0;
        laser_obj->base_info.ok_cnt = 0;
        laser_obj->base_info.error_cnt = 0;
        laser_obj->base_info.reconnect_cnt = 0;

        if(laser_obj->raw_data.raw_data_buffer != NULL)
        {
            rt_free(laser_obj->raw_data.raw_data_buffer);
        }
        laser_obj->raw_data.seq = 0;
        laser_obj->raw_data.raw_data_count = 0;
        laser_obj->raw_data.raw_data_buffer = NULL;

        laser_obj->base_info.is_init = false;
        laser_obj->base_info.is_connected = false;
    }
    LOG_I("[%s]激光资源释放 .", laser_obj->base_info.drv_name);

    return eLaserDrvOk;
}

/**
 * vanjee重新连接.
 *
 * @param[in]
 * @param[out]
 *      laser_obj : 激光相关参数
 * @return
 *      返回错误码
 */
laser_drv_result_code_t laser_drv_vanjee_reconnect(laser_drv_t *laser_obj)
{
    laser_net_result_code_t net_ret = eLaserNetError;

    RT_ASSERT(laser_obj != NULL);
    if(laser_obj->base_info.is_init) 
    {
        // 设置为断开连接
        laser_obj->base_info.is_connected = false;
        // 记录重连次数
        laser_obj->base_info.reconnect_cnt += 1;

        LOG_I("[%s]激光网络重新连接 .", laser_obj->base_info.drv_name);

        /* 关闭socket接口 */
        laser_net_close(&laser_obj->laser_net);
        rt_thread_mdelay(1000);

        /* 根据端口选择连接方式 */
        if(laser_obj->laser_net.dest_port == NET_DEFAULT_PORT)
        {
            net_ret = laser_net_connect_udp(&laser_obj->laser_net);
            if(net_ret != eLaserNetOk)
            {
                LOG_E("[%s]激光网络重新连接失败: net_ret=%d !", laser_obj->base_info.drv_name, net_ret);
                return eLaserDrvError;
            }
        }
        else
        {
            LOG_E("[%s]激光端口选择错误: dest port = %d(2110 - udp) !", laser_obj->base_info.drv_name, laser_obj->laser_net.dest_port);
            return eLaserDrvError;
        }

        /* 设置激光连接成功标志位 */
        laser_obj->base_info.is_connected = true;
        LOG_I("[%s]激光网络重新连接成功 .", laser_obj->base_info.drv_name);

        // 更新起始时间
        gettimeofday(&laser_obj->base_info.start_time, NULL);

        return eLaserDrvOk;
    }

    return eLaserDrvError;
}

/**
 * vanjee读取一帧数据.
 *
 * @param[in]
 * @param[out]
 *      laser_obj : 激光相关参数
 * @return
 *      返回错误码
 */
laser_drv_result_code_t laser_drv_vanjee_read_data(laser_drv_t *laser_obj)
{
    rt_base_t level;
    laser_net_result_code_t net_ret = eLaserNetError;
    uint32_t rx_count = 0;

    /* 未初始化完成 或 未连接成功时 不可读取数据 */
    if(!laser_obj->base_info.is_init || !laser_obj->base_info.is_connected) return eLaserDrvError;

    /* 从网络中读取数据 */
    net_ret = laser_net_recv_some(&laser_obj->laser_net, &laser_obj->net_rx_buffer[laser_obj->net_rx_count], DATA_BUFFER_SIZE - laser_obj->net_rx_count, &rx_count);
    if(net_ret != eLaserNetOk)
    {
        // level = rt_hw_interrupt_disable();
        // // 错误读取次数 +1
        // laser_obj->base_info.error_cnt += 1;
        // rt_hw_interrupt_enable(level);

        // LOG_E("[%s]激光读取数据错误: net_ret=%d !", laser_obj->base_info.drv_name, net_ret);

        // rt_thread_mdelay(20);
        return eLaserDrvReadDataError;
    }

    level = rt_hw_interrupt_disable();
    // 读取数据次数 +1
    laser_obj->base_info.read_cnt += 1;
    laser_obj->net_rx_count += rx_count;
    rt_hw_interrupt_enable(level);

    // LOG_D("[%s]接收数据长度为%d", laser_obj->net_rx_count);
    int fifo_write_cnt = efifo_write(laser_obj->parase_fifo, laser_obj->net_rx_buffer, laser_obj->net_rx_count);
    level = rt_hw_interrupt_disable();
    laser_obj->net_rx_count = 0;
    rt_hw_interrupt_enable(level);
    if (fifo_write_cnt != rx_count)
    {
        LOG_E("[%s]数据队列空间不足", laser_obj->base_info.drv_name);
        // return eLaserDrvPushDataError;
    }
    
    return eLaserDrvOk;
}

/**
 * vanjee读取一帧数据.
 *
 * @param[in]
 * @param[out]
 *      laser_obj : 激光相关参数
 * @return
 *      返回错误码
 */
laser_drv_result_code_t laser_drv_vanjee_get_one_frame(laser_drv_t *laser_obj)
{
    rt_base_t level;
    
    while (efifo_get_used(laser_obj->parase_fifo) > 4)
    {
        /* 包头校验 */
        efifo_pick(laser_obj->parase_fifo, laser_obj->net_tx_buffer, 2);
        if ((laser_obj->net_tx_buffer[0] != 0xFF) || (laser_obj->net_tx_buffer[1] != 0xAA))
        {
            level = rt_hw_interrupt_disable();
            laser_obj->base_info.error_cnt += 1;
            rt_hw_interrupt_enable(level);

            efifo_cut(laser_obj->parase_fifo, 1);
            LOG_E("[%s]包头校验错误,0x%x 0x%x != 0xFF 0xAA", laser_obj->base_info.drv_name, 
                laser_obj->net_tx_buffer[0], laser_obj->net_tx_buffer[1]);
            continue;
        }

        /* 获取帧长度 */
        efifo_pick(laser_obj->parase_fifo, laser_obj->net_tx_buffer, 4);
        unsigned short frame_len = ((laser_obj->net_tx_buffer[2] & 0x00ff) << 8) | (laser_obj->net_tx_buffer[3] & 0x00ff);
        frame_len += 4;

        /* fifo中满足一帧完整数据的长度后开始解析 */
        if (efifo_get_used(laser_obj->parase_fifo) < frame_len) return eLaserDrvError;

        /* 包尾校验 */
        efifo_pick(laser_obj->parase_fifo, laser_obj->net_tx_buffer, frame_len);
        if ((laser_obj->net_tx_buffer[frame_len - 1] != 0xEE) || (laser_obj->net_tx_buffer[frame_len - 2] != 0xEE))
        {
            level = rt_hw_interrupt_disable();
            laser_obj->base_info.error_cnt += 1;
            rt_hw_interrupt_enable(level);

            efifo_cut(laser_obj->parase_fifo, 2);
            LOG_E("[%s]包尾校验错误,0x%x 0x%x != 0xEE 0xEE", laser_obj->base_info.drv_name, 
            laser_obj->net_tx_buffer[frame_len - 1], laser_obj->net_tx_buffer[frame_len - 2]);
            continue;
        }
        else
        {
            /* 数据校验 */
            if (0 != check_xor(laser_obj->net_tx_buffer, frame_len))
            {
                level = rt_hw_interrupt_disable();
                laser_obj->base_info.error_cnt += 1;
                rt_hw_interrupt_enable(level);

                efifo_cut(laser_obj->parase_fifo, frame_len);
                LOG_E("[%s]数据校验错误", laser_obj->base_info.drv_name);
                continue;
            }

            gettimeofday(&laser_obj->base_info.end_time, NULL);
            laser_obj->base_info.diff_time = laser_timeval_diff(&laser_obj->base_info.start_time, &laser_obj->base_info.end_time);
            level = rt_hw_interrupt_disable();
            laser_obj->base_info.ok_cnt += 1;
            laser_obj->raw_data.seq += 1;
            // 激光时间戳记录
            laser_obj->raw_data.stamp = laser_obj->base_info.start_time;
            laser_obj->raw_data.raw_data_count = frame_len;
            rt_hw_interrupt_enable(level);
            rt_memcpy(laser_obj->raw_data.raw_data_buffer, laser_obj->net_tx_buffer, frame_len);

            // rt_kprintf("frame_len = %d\n", frame_len);
            // LOG_D("pro one pack ok");
            
            efifo_cut(laser_obj->parase_fifo, frame_len);

            // 更新起始时间
            gettimeofday(&laser_obj->base_info.start_time, NULL);
            return eLaserDrvOk;
        }
    }
    return eLaserDrvError;
}

/**
 * vanjee获取一帧数据.
 *
 * @param[in]
 * @param[out]
 *      laser_obj  : 激光相关参数
 *      laser_data : 激光原始数据
 * @return
 *      返回错误码
 */
laser_drv_result_code_t laser_drv_vanjee_get_raw_data(laser_drv_t *laser_obj, laser_drv_raw_data_t *laser_data)
{
    RT_ASSERT(laser_obj != NULL);
    RT_ASSERT(laser_data != NULL);

    if(!laser_obj->base_info.is_init) return eLaserDrvError;

    if(laser_obj->raw_data.raw_data_count > 0)
    {
        laser_data->seq = laser_obj->raw_data.seq;
        laser_data->stamp = laser_obj->raw_data.stamp;
        laser_data->frame_id = laser_obj->raw_data.frame_id;
        laser_data->raw_data_count = laser_obj->raw_data.raw_data_count;
        if (laser_data->raw_data_buffer == NULL) laser_data->raw_data_buffer = (uint8_t *)rt_malloc(laser_obj->raw_data.raw_data_count);
        // laser_data->raw_data_buffer = laser_obj->raw_data.raw_data_buffer;
        rt_memcpy(laser_data->raw_data_buffer, laser_obj->raw_data.raw_data_buffer, laser_obj->raw_data.raw_data_count);
        laser_obj->raw_data.raw_data_count = 0;

        return eLaserDrvOk;
    }

    return eLaserDrvError;
}

// 采集任务运行标志（可静态或加到 laser_drv_t 内部）
static bool g_laser_running = false;

laser_drv_result_code_t laser_device_start(laser_drv_t *laser_obj) {
    if (!laser_obj || !laser_obj->base_info.is_init) return eLaserDrvError;
    g_laser_running = true;
    return eLaserDrvOk;
}

laser_drv_result_code_t laser_device_stop(laser_drv_t *laser_obj) {
    if (!laser_obj || !laser_obj->base_info.is_init) return eLaserDrvError;
    g_laser_running = false;
    return eLaserDrvOk;
}

laser_drv_result_code_t laser_device_get_status(laser_drv_t *laser_obj, laser_device_state_info_t *state_info) {
    if (!laser_obj || !state_info) return eLaserDrvError;
    state_info->is_init = laser_obj->base_info.is_init;
    state_info->is_connected = laser_obj->base_info.is_connected;
    state_info->work_status = g_laser_running ? LASER_STATUS_RUNNING : LASER_STATUS_IDLE;
    if (!laser_obj->base_info.is_connected) state_info->work_status = LASER_STATUS_ERROR;
    state_info->error_code = (laser_obj->base_info.is_connected ? 0 : -1);
    state_info->read_cnt = laser_obj->base_info.read_cnt;
    state_info->ok_cnt = laser_obj->base_info.ok_cnt;
    state_info->error_cnt = laser_obj->base_info.error_cnt;
    state_info->reconnect_cnt = laser_obj->base_info.reconnect_cnt;
    state_info->diff_time = laser_obj->base_info.diff_time;
    return eLaserDrvOk;
}

/* =============================================================================
 *                              static function 
 * ========================================================================== */

/**
 * bbc校验.
 *
 * @param[in]
 *      recv_buf : 数据指针
 *      recv_len : 数据长度
 * @param[out]
 * @return
 *      0 - 校验成功
 *     -1 - 校验失败
 */
static unsigned int check_xor(unsigned char *recv_buf, int recv_len)
{
    int i = 0;
    unsigned char check = 0;
    unsigned char *p = recv_buf;
    int len;

    if (*p == 0xFF)
    {
        p = p + 2;
        len = recv_len - 6;
        for (i = 0; i < len; i++)
        {
            check ^= *p++;
        }
        p++;
        if (check == *p)
        {
            return 0;
        }
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/* =============================================================================
 *                              shell function
 * ========================================================================== */


/*******************************************************************************
 *                                  EOF
 ******************************************************************************/
