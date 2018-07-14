/******************************************************************************

  Copyright (C), 2018-2028, Broadsens Co., Ltd.

 ******************************************************************************
  File Name     : drv_bno055.c
  Version       : Initial Draft
  Author        : wuhaibin
  Created       : 2018/4/23
  Last Modified :
  Description   :
  ----------------------------------------------------------------------------
  History       :
  1.Date        : 2018/4/23
    Author      : wuhaibin
    Modification: Created file

******************************************************************************/

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <math.h>
#include "linux/i2c-dev.h"
#include "linux/i2c.h"
#include <sys/stat.h>
#include <stdbool.h>
#include <pthread.h>

#include "drv_bno055.h"
#include "drv_common.h"
#include "drv_e2prom.h"

#include "drv_i2c.h"

/* 全局变量定义区 */
BNO055_CALIB_DATA_S g_calib_data;
static bool g_bno055_calib_stat = false;
static int g_bno055_iio_fd = -1;

DRV_I2C_DEV_S g_bno055_i2c_dev;

/*****************************************************************************
 Function     : bno055_i2c_write
 Description  :
 Input        : unsigned char reg
                unsigned char *buf
                unsigned short len
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
static int bno055_i2c_write(unsigned char reg, unsigned char *buf, unsigned short len)
{
    return drv_i2c_write(&g_bno055_i2c_dev, reg, buf, len);
}

/*****************************************************************************
 Function     : bno055_i2c_read
 Description  :
 Input        : unsigned char reg
                unsigned char *buf
                unsigned short len
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
static int bno055_i2c_read(unsigned char reg, unsigned char *buf, unsigned short len)
{
    return drv_i2c_read(&g_bno055_i2c_dev, reg, buf, len);
}

/*****************************************************************************
 Function     : bno055_i2c_write_byte
 Description  :
 Input        : unsigned char reg
                unsigned char data
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
static int bno055_i2c_write_byte(unsigned char reg, unsigned char data)
{
    return drv_i2c_write_byte(&g_bno055_i2c_dev, reg, data);
}

/*****************************************************************************
 Function     : bno055_i2c_write_bits
 Description  :
 Input        : unsigned char reg
                unsigned char mask
                unsigned char data
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
static int bno055_i2c_write_bits(unsigned char reg, unsigned char mask, unsigned char data)
{
    int ret = -1;
    unsigned char reg_data;

    ret = bno055_i2c_read(reg, &reg_data, 1);
    if (ret < 0)
    {
        return -1;
    }

    reg_data = (reg_data & (~mask)) | (data & mask);
    ret = bno055_i2c_write(reg, &reg_data, 1);
    if (ret < 0)
    {
        return -1;
    }

    return 0;
}

/*****************************************************************************
 函 数 名  : bno055_get_operation_mode
 功能描述  :
 输入参数  : int fd
             unsigned char *op_mode
 输出参数  : 无
 返 回 值  :
 备    注  :
 ----------------------------------------------------------------------------
 修改历史      :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 新生成函数

*****************************************************************************/
int bno055_get_operation_mode(BNO055_OPMODE_E *op_mode)
{
    int ret = -1;
    unsigned char data;

    ret = bno055_i2c_read(BNO055_REG_OPR_MODE, &data, 1);
    if (ret < 0)
    {
        return -1;
    }

    *op_mode = (BNO055_OPMODE_E)(data & BNO055_OPR_MODE_MASK);

    return 0;
}

/*****************************************************************************
 函 数 名  : bno055_set_operation_mode
 功能描述  :
 输入参数  : int fd
             unsigned char operation_mode
 输出参数  : 无
 返 回 值  :
 备    注  :
 ----------------------------------------------------------------------------
 修改历史      :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 新生成函数

*****************************************************************************/
int bno055_set_operation_mode(BNO055_OPMODE_E op_mode)
{
    BNO055_OPMODE_E pre_op_mode;
    unsigned char ret;
    unsigned long switch_time = 0;  /* unit:ms */

    ret = bno055_get_operation_mode(&pre_op_mode);
    if (ret < 0)
    {
        return -1;
    }

    if (op_mode == pre_op_mode)
    {
        return 0;
    }
    else if (BNO055_CONFIG_MODE == pre_op_mode)
    {
        switch_time = 600;
    }
    else
    {
        switch_time = 20;
    }

    ret = bno055_i2c_write_bits(BNO055_REG_OPR_MODE, BNO055_OPR_MODE_MASK, op_mode);
    if (ret < 0)
    {
        return -1;
    }

    usleep(switch_time * 1000);

    return 0;
}

/*****************************************************************************
 Function     : bno055_set_page
 Description  :
 Input        : int page
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_set_page(unsigned char page)
{
    int ret = -1;

    if (page > 1 || page < 0)
    {
        DRV_PRINT_ERROR("page val is invalid");
        return -1;
    }

    ret = bno055_i2c_write_byte(BNO055_REG_PAGE_ID, page);

    if (ret < 0)
    {
        DRV_PRINT_ERROR("failed to switch to register page %d", page);
    }

    return ret;
}

/*****************************************************************************
 Function     : bno055_soft_reset
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_soft_reset(void)
{
    int ret;

    ret = bno055_i2c_write_bits(BNO055_REG_SYS_TRIGGER, BNO055_SYS_TRIGGER_MASK, BNO055_SYS_RST);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("failed to reset chip\n");
        return ret;
    }

    ret = bno055_i2c_write_bits(BNO055_REG_SYS_TRIGGER, BNO055_SYS_TRIGGER_MASK, 0);

    return ret;
}

/*****************************************************************************
 Function     : bno055_get_common_data
 Description  :
 Input        : COMMON_DATA_S *data
                unsigned char addr
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
static int bno055_get_common_data(COMMON_DATA_S *data, unsigned char addr)
{
    int ret = -1;
    unsigned char buf[6] = {0};
    unsigned short tmp;

    ret = bno055_i2c_read(addr, buf, 6);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("failed to read xyz data");
        return -1;
    }

    data->x = comp2ori(buf);
    data->y = comp2ori(buf + 2);
    data->z = comp2ori(buf + 4);

    return 0;
}

/*****************************************************************************
 Function     : bno055_set_common_data
 Description  :
 Input        : COMMON_DATA_S *data
                unsigned char addr
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
static int bno055_set_common_data(COMMON_DATA_S *data, unsigned char addr)
{
    int ret = -1;
    unsigned char buf[6] = {0};

    ori2comp(data->x, buf);
    ori2comp(data->y, buf + 2);
    ori2comp(data->z, buf + 4);

    return bno055_i2c_write(addr, buf, 6);
}

/*****************************************************************************
 Function     : bno055_read_temp
 Description  :
 Input        : int *temp
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_read_temp(char *temp)
{
    int ret = -1;
    unsigned char buf[2];

    ret = bno055_i2c_read(BNO055_REG_TEMP, buf, 1);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("failed to read temp data");
        return -1;
    }

    *temp = (char)buf[0];

    return 0;
}

/*****************************************************************************
 Function     : bno055_get_accel_raw
 Description  :
 Input        : COMMON_DATA_S *accel
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_accel(COMMON_DATA_S *accel)
{
    return bno055_get_common_data(accel, BNO055_REG_ACC_DATA_X_LSB);
}

/*****************************************************************************
 Function     : bno055_get_magn_raw
 Description  :
 Input        : COMMON_DATA_S *magn
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_magn(COMMON_DATA_S *magn)
{
    return bno055_get_common_data(magn, BNO055_REG_MAG_DATA_X_LSB);
}

/*****************************************************************************
 Function     : bno055_get_linear_accel_raw
 Description  :
 Input        : COMMON_DATA_S *lia_accel
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_linear_accel(COMMON_DATA_S *lia_accel)
{
    return bno055_get_common_data(lia_accel, BNO055_REG_LIA_ACC_DATA_X_LSB);
}

/*****************************************************************************
 函 数 名  : bno055_read_eul_data
 功能描述  :
 输入参数  : int fd
             SENSOR_DATA_S *eul
 输出参数  : 无
 返 回 值  :
 备    注  :
 ----------------------------------------------------------------------------
 修改历史      :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 新生成函数

*****************************************************************************/
int bno055_get_eul(COMMON_DATA_S *eul)
{
    return bno055_get_common_data(eul, BNO055_REG_EUL_DATA_X_LSB);
}

/*****************************************************************************
 Function     : bno055_get_gravity
 Description  :
 Input        : COMMON_DATA_S *grv
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_gravity(COMMON_DATA_S *grv)
{
    return bno055_get_common_data(grv, BNO055_REG_GRV_DATA_X_LSB);
}

/*****************************************************************************
 Function     : bno055_get_gyroscope_raw
 Description  :
 Input        : COMMON_DATA_S *gyr
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_angular_velocity(COMMON_DATA_S *gyr)
{
    return bno055_get_common_data(gyr, BNO055_REG_GYR_DATA_X_LSB);
}

/*****************************************************************************
 Function     : bno055_get_quaterion_raw
 Description  :
 Input        : QUATERION_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_quaterion(QUATERION_DATA_S *quat)
{
    int ret = -1;
    unsigned char buf[8] = {0};
    unsigned short tmp;

    ret = bno055_i2c_read(BNO055_REG_QUA_DATA_W_LSB, buf, 8);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("failed to read xyz data");
        return -1;
    }

    tmp = buf[1] << 8 | buf[0];
    if (tmp & 0x8000)
    {
        tmp &= 0x7FFFF;
        tmp = ~tmp + 1;
        quat->w = (short)(-tmp);
    }
    else
    {
        quat->w = tmp;
    }

    quat->w = comp2ori(buf);
    quat->x = comp2ori(buf + 2);
    quat->y = comp2ori(buf + 4);
    quat->z = comp2ori(buf + 6);

    return 0;
}

/*****************************************************************************
 Function     : bno055_get_accel_offset
 Description  :
 Input        : COMMON_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_accel_offset(COMMON_DATA_S *data)
{
    return bno055_get_common_data(data, BNO055_REG_ACC_OFFSET_X_LSB);
}

/*****************************************************************************
 Function     : bno055_get_magn_offset
 Description  :
 Input        : COMMON_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_magn_offset(COMMON_DATA_S *data)
{
    return bno055_get_common_data(data, BNO055_REG_MAG_OFFSET_X_LSB);
}

/*****************************************************************************
 Function     : bno055_get_gyroscope_offset
 Description  :
 Input        : COMMON_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_gyroscope_offset(COMMON_DATA_S *data)
{
    return bno055_get_common_data(data, BNO055_REG_GYR_OFFSET_X_LSB);
}

/*****************************************************************************
 Function     : bno055_set_accel_offset
 Description  :
 Input        : COMMON_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_set_accel_offset(COMMON_DATA_S *data)
{
    int ret = -1;
    BNO055_OPMODE_E pre_op_mode;

    ret = bno055_get_operation_mode(&pre_op_mode);
    if (ret < 0)
    {
        return -1;
    }
    /* switch to config mode */
    bno055_set_operation_mode(BNO055_CONFIG_MODE);
    ret = bno055_set_common_data(data, BNO055_REG_ACC_OFFSET_X_LSB);
    if (ret < 0)
    {
        return -1;
    }
    /* mode recovery */
    bno055_set_operation_mode(pre_op_mode);

    return 0;
}

/*****************************************************************************
 Function     : bno055_set_mag_offset
 Description  :
 Input        : COMMON_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_set_mag_offset(COMMON_DATA_S *data)
{
    int ret = -1;
    BNO055_OPMODE_E pre_op_mode;

    ret = bno055_get_operation_mode(&pre_op_mode);
    if (ret < 0)
    {
        return -1;
    }
    /* switch to config mode */
    bno055_set_operation_mode(BNO055_CONFIG_MODE);
    ret = bno055_set_common_data(data, BNO055_REG_MAG_OFFSET_X_LSB);
    if (ret < 0)
    {
        return -1;
    }
    /* mode recovery */
    bno055_set_operation_mode(pre_op_mode);

    return 0;
}

/*****************************************************************************
 Function     : bno055_set_gyr_offset
 Description  :
 Input        : COMMON_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_set_gyr_offset(COMMON_DATA_S *data)
{
    int ret = -1;
    BNO055_OPMODE_E pre_op_mode;

    ret = bno055_get_operation_mode(&pre_op_mode);
    if (ret < 0)
    {
        return -1;
    }
    /* switch to config mode */
    bno055_set_operation_mode(BNO055_CONFIG_MODE);
    ret = bno055_set_common_data(data, BNO055_REG_GYR_OFFSET_X_LSB);
    if (ret < 0)
    {
        return -1;
    }
    /* mode recovery */
    bno055_set_operation_mode(pre_op_mode);

    return 0;
}

/*****************************************************************************
 Function     : bno055_export_calib_data
 Description  :
 Input        : BNO055_CALIB_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_export_calib_data(BNO055_CALIB_DATA_S *data)
{
    unsigned char buf[18] = {0};

    drv_e2prom_read(BNO055_CALIB_DATA_EXIST_ADDR, buf, 2);
    if (BNO055_CALIB_DATA_EXIST_FLAG == *(unsigned short *)buf)
    {
        drv_e2prom_read(BNO055_CALIB_DATA_SAVE_ADDR, buf, 18);

        data->accel_offset.x = comp2ori(buf);
        data->accel_offset.y = comp2ori(buf + 2);
        data->accel_offset.z = comp2ori(buf + 4);

        data->gyr_offset.x = comp2ori(buf + 6);
        data->gyr_offset.y = comp2ori(buf + 8);
        data->gyr_offset.z = comp2ori(buf + 10);

        data->magn_offset.x = comp2ori(buf + 12);
        data->magn_offset.y = comp2ori(buf + 14);
        data->magn_offset.z = comp2ori(buf + 16);
        return 0;
    }

    return -1;
}

/*****************************************************************************
 Function     : bno055_erase_calib_data
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_erase_calib_data(void)
{
    /* interface save data to e2prom */
    return 0;
}

/*****************************************************************************
 Function     : bno055_save_calib_data
 Description  :
 Input        : BNO055_CALIB_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_save_calibration_data(BNO055_CALIB_DATA_S *data)
{
    int i = 0;
    unsigned char buf[18] = {0};
    /* interface save data to e2prom */

    ori2comp(data->accel_offset.x, buf);
    ori2comp(data->accel_offset.y, buf + 2);
    ori2comp(data->accel_offset.z, buf + 4);

    ori2comp(data->gyr_offset.x, buf + 6);
    ori2comp(data->gyr_offset.y, buf + 8);
    ori2comp(data->gyr_offset.z, buf + 10);

    ori2comp(data->magn_offset.x, buf + 12);
    ori2comp(data->magn_offset.y, buf + 14);
    ori2comp(data->magn_offset.z, buf + 16);

    drv_e2prom_write(BNO055_CALIB_DATA_SAVE_ADDR, buf, 18);

    buf[0] = 0x55;
    buf[1] = 0xAA;
    return drv_e2prom_write(BNO055_CALIB_DATA_EXIST_ADDR, buf,2);
}

/*****************************************************************************
 函 数 名  : bno055_get_accel_calib_stat
 功能描述  :
 输入参数  : int fd
             unsigned char *accle_calib
 输出参数  : 无
 返 回 值  :
 备    注  :
 ----------------------------------------------------------------------------
 修改历史      :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 新生成函数

*****************************************************************************/
int bno055_get_accel_calib_stat(unsigned char *accle_calib)
{
    unsigned char data;
    int ret = -1;

    ret = bno055_i2c_read(BNO055_REG_CALIB_STAT, &data, 1);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("read calib stat failed");
        return -1;
    }

    *accle_calib = data & BNO055_ACCEL_CALIB_STAT_MSK;

    return 0;
}

/*****************************************************************************
 函 数 名  : bno055_get_mag_calib_stat
 功能描述  :
 输入参数  : int fd
             unsigned char *mag_calib
 输出参数  : 无
 返 回 值  :
 备    注  :
 ----------------------------------------------------------------------------
 修改历史      :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 新生成函数

*****************************************************************************/
int bno055_get_mag_calib_stat(unsigned char *mag_calib)
{
    unsigned char data;
    int ret = -1;

    ret = bno055_i2c_read(BNO055_REG_CALIB_STAT, &data, 1);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("read calib stat failed");
        return -1;
    }

    *mag_calib = data & BNO055_MAG_CALIB_STAT_MSK;

    return 0;
}

/*****************************************************************************
 函 数 名  : bno055_get_gyro_calib_stat
 功能描述  :
 输入参数  : int fd
             unsigned char *gyro_calib
 输出参数  : 无
 返 回 值  :
 备    注  :
 ----------------------------------------------------------------------------
 修改历史      :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 新生成函数

*****************************************************************************/
int bno055_get_gyro_calib_stat(unsigned char *gyro_calib)
{
    unsigned char data;
    int ret = -1;

    ret = bno055_i2c_read(BNO055_REG_CALIB_STAT, &data, 1);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("read calib stat failed");
        return -1;
    }

    *gyro_calib = data & BNO055_GYRO_CALIB_STAT_MSK;

    return 0;
}

/*****************************************************************************
 函 数 名  : bno055_get_sys_calib_stat
 功能描述  :
 输入参数  : int fd
             unsigned char *sys_calib
 输出参数  : 无
 返 回 值  :
 备    注  :
 ----------------------------------------------------------------------------
 修改历史      :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 新生成函数

*****************************************************************************/
int bno055_get_sys_calib_stat(unsigned char *sys_calib)
{
    unsigned char data;
    int ret = -1;

    ret = bno055_i2c_read(BNO055_REG_CALIB_STAT, &data, 1);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("read calib stat failed");
        return -1;
    }

    *sys_calib = data & BNO055_SYS_CALIB_STAT_MSK;

    return 0;
}

/*****************************************************************************
 Function     : bno055_check_calib_finish
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
bool bno055_check_calib_finish(void)
{
    int ret = -1;
    unsigned char data = 0;

    ret = bno055_i2c_read(BNO055_REG_CALIB_STAT, &data, 1);
    if (ret < 0)
    {
        DRV_PRINT_ERROR("read calib stat failed");
        return false;
    }

    if (0x3F == (data & 0x3f))
    {
        return true;
    }

    return false;
}

/*****************************************************************************
 Function     : bno055_do_calibrate
 Description  :
 Input        : void
 Output       : None
 Return Value : void
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/11
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_calibration_data(BNO055_CALIB_DATA_S *data)
{
    BNO055_OPMODE_E pre_op_mode;

    if (bno055_get_operation_mode(&pre_op_mode) < 0) {
        return -1;
    }
    if (bno055_set_operation_mode(BNO055_CONFIG_MODE) < 0) {
        return -1;
    }
    if (bno055_get_accel_offset(&data->accel_offset) < 0) {
        return -1;
    }
    if (bno055_get_magn_offset(&data->magn_offset) < 0) {
        return -1;
    }
    if (bno055_get_gyroscope_offset(&data->gyr_offset) < 0) {
        return -1;
    }
    if (bno055_set_operation_mode(pre_op_mode) < 0) {
        return -1;
    }

    return 0;
}

/*****************************************************************************
 Function     : bno055_do_calibrate
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/11
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
void bno055_do_calibrate(void)
{
    g_bno055_calib_stat = false;
    return;
}

/*****************************************************************************
 Function     : bno055_get_calibrate_stat
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/11
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
bool bno055_get_calibration_stat(void)
{
    bool stat = false;
    BNO055_CALIB_DATA_S data;

    if (true == g_bno055_calib_stat) {
        return true;
    } else {
        if (true == bno055_check_calib_finish()) {
            if (bno055_get_calibration_data(&data) < 0) {
                bno055_save_calibration_data(&data);
            }
            g_bno055_calib_stat = true;

            return true;
        }
    }

    return false;
}

/*****************************************************************************
 Function     : bno055_calibration_init
 Description  :
 Input        : BNO055_CALIB_DATA_S *data
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/24
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_calibration_init(BNO055_CALIB_DATA_S *data)
{
    int ret = -1;

    /* export the clibration data */
    ret = bno055_export_calib_data(data);
    if (ret < 0)
    {
        DRV_PRINT_WARN("can not export calibration data");
        return -1;
    }

    DRV_PRINT_DEBUG("accel_offset in rom :%d %d %d", data->accel_offset.x,
                                                    data->accel_offset.y,
                                                    data->accel_offset.z);

    DRV_PRINT_DEBUG("gyr_offset in rom :%d %d %d", data->gyr_offset.x,
                                                    data->gyr_offset.y,
                                                    data->gyr_offset.z);
    DRV_PRINT_DEBUG("magn_offset in rom :%d %d %d", data->magn_offset.x,
                                                    data->magn_offset.y,
                                                    data->magn_offset.z);
    /* to chip */
    bno055_set_accel_offset(&(data->accel_offset));
    bno055_set_gyr_offset(&(data->gyr_offset));
    bno055_set_mag_offset(&(data->magn_offset));

    g_bno055_calib_stat = true;

    return 0;
}

/*****************************************************************************
 Function     : bno055_get_chip_id
 Description  :
 Input        : unsigned char *chip_id
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_chip_id(unsigned char *chip_id)
{
    return bno055_i2c_read(BNO055_REG_CHIP_ID, chip_id, 1);
}

/*****************************************************************************
 Function     : bno055_get_acc_id
 Description  :
 Input        : unsigned char *acc_id
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_acc_id(unsigned char *acc_id)
{
    return bno055_i2c_read(BNO055_REG_ACC_ID, acc_id, 1);
}

/*****************************************************************************
 Function     : bno055_get_mag_id
 Description  :
 Input        : unsigned char *mag
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_mag_id(unsigned char *mag)
{
    return bno055_i2c_read(BNO055_REG_MAG_ID, mag, 1);
}

/*****************************************************************************
 Function     : bno055_get_gyr_id
 Description  :
 Input        : unsigned char *gyr
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_gyr_id(unsigned char *gyr)
{
    return bno055_i2c_read(BNO055_REG_GYR_ID, gyr, 1);
}

/*****************************************************************************
 Function     : bno055_get_sw_rev_id
 Description  :
 Input        : unsigned short *sw_rev_id
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_sw_rev_id(unsigned short *sw_rev_id)
{
    unsigned char buf[2];
    int ret = -1;

    ret = bno055_i2c_read(BNO055_REG_SW_REV_ID_LSB, buf, 2);
    if (ret < 0)
    {
        return -1;
    }

    *sw_rev_id = *((unsigned short *)buf);
}

/*****************************************************************************
 Function     : bno055_get_bl_rev_id
 Description  :
 Input        : unsigned char *bl_rev_id
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_get_bl_rev_id(unsigned char *bl_rev_id)
{
    return bno055_i2c_read(BNO055_REG_BL_REV_ID, bl_rev_id, 1);
}

/*****************************************************************************
 Function     : bno055_show_sw_rev
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/3
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int bno055_show_sw_rev(void)
{
    int ret = -1;
    unsigned short sw_rev_id;
    unsigned char bl_rev_id;

    ret = bno055_get_sw_rev_id(&sw_rev_id);
    if (ret < 0)
    {
        return -1;
    }
    ret = bno055_get_bl_rev_id(&bl_rev_id);
    if (ret < 0)
    {
        return -1;
    }

    DRV_PRINT_INFO("sw_rev_id : 0x%x, bl_rev_id : 0x%x", sw_rev_id, bl_rev_id);

    return 0;
}

/*****************************************************************************
 Function     : drv_check_bno055_sensor_connect
 Description  :
 Input        : void
 Output       : None
 Return Value :
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/5/10
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
int check_bno055_connect(void)
{
    int ret;
    unsigned char chip_id;

    ret = bno055_get_chip_id(&chip_id);
    if (ret < 0)
    {
        return -1;
    }
    if (chip_id != BNO055_CHIP_ID)
    {
        return -1;
    }

    return 0;
}

/*****************************************************************************
 Function     : bno055_i2c_init
 Description  :
 Input        : void
 Output       : None
 Return Value : static
 Note         :
 ----------------------------------------------------------------------------
 History        :
 1.Date         : 2018/4/23
   Author       : wuhaibin
   Modification : Created function

*****************************************************************************/
static int bno055_i2c_init(void)
{
    int fd = -1;

    fd = drv_i2c_dev_open(1, BNO055_I2C_ADDR, &g_bno055_i2c_dev);
    if (fd < 0)
    {
        DRV_PRINT_ERROR("open %s failed!", BNO055_I2C_BUS);
        return -1;
    }

    return 0;
}

static int counter = 0;
int bno055_read_iio_ring(float *data, struct timeval *timestamp)
{
    int size;
    unsigned char buf[16] = {0};
    unsigned char time_buf[8] = {0};
    int64_t *p_time;

    size = read(g_bno055_iio_fd, buf, 16);
    if (16 == size) {
        data[0] = (float)comp2ori(buf) / 100.0;
        data[1] = (float)comp2ori(buf + 2) / 100.0;
        data[2] = (float)comp2ori(buf + 4) / 100.0;

        p_time = (int64_t *)(buf + 8);
        timestamp->tv_sec = *p_time / 1000000000;
        timestamp->tv_usec = *p_time % 1000000000 / 1000;
        counter++;

        if (counter >= 100) {
            DRV_PRINT_DEBUG("[%u.%u]:counter = 100\n", timestamp->tv_sec, timestamp->tv_usec);
            counter = 0;
        }
        return 0;
    }

    return -1;
}

int bno055_init(int fd, BNO055_CFG_S *bno055_cfg)
{
    unsigned char chip_id, chip_id_bytes[7];
    int ret = -1;
    unsigned char buf[2] = {0};

    g_bno055_i2c_dev.fd = fd;
    g_bno055_i2c_dev.addr = BNO055_I2C_ADDR;

    /* clear the calibration data */
    memset(&g_calib_data, 0, sizeof(BNO055_CALIB_DATA_S));
    ret = bno055_calibration_init(&g_calib_data);

    g_bno055_iio_fd = open("/dev/iio:device0", O_NONBLOCK);
    if (g_bno055_iio_fd < 0)
    {
        DRV_PRINT_ERROR("open iio device 0 error");
        return -1;
    }
    system("echo 0 > /sys/bus/iio/devices/iio\\:device0/buffer/enable");
    system("echo 1 > /sys/bus/iio/devices/iio\\:device0/scan_elements/in_accel_x_linear_en");
    system("echo 1 > /sys/bus/iio/devices/iio\\:device0/scan_elements/in_accel_y_linear_en");
    system("echo 1 > /sys/bus/iio/devices/iio\\:device0/scan_elements/in_accel_z_linear_en");
    system("echo 1 > /sys/bus/iio/devices/iio\\:device0/scan_elements/in_timestamp_en");
    system("echo 1024 > /sys/bus/iio/devices/iio\\:device0/buffer/length");
    system("echo 1 > /sys/bus/iio/devices/iio\\:device0/buffer/enable");

    return 0;
}
