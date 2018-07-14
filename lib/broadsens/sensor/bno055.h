/******************************************************************************

                  版权所有 (C), 2018-2019, 嘉兴博感科技有限公司

 ******************************************************************************
  文 件 名   : bno055.h
  版 本 号   : 初稿
  作    者   : wuhaibin
  生成日期   : 2018年3月8日
  最近修改   :
  功能描述   :
  ---------------------------------------------------------------------------
  修改历史   :
  1.日    期   : 2018年3月8日
    作    者   : wuhaibin
    修改内容   : 创建文件

******************************************************************************/

#ifndef __BNO055_H__
#define __BNO055_H__

#ifndef BIT
#define BIT(x)  (1 << (x))
#endif

#define BNO055_I2C_BUS    I2C_BUS_1
#define BNO055_I2C_ADDR   0x29
//#define le16_to_cpu(msb, lsb)
#define BNO055_CALIB_DATA_EXIST_FLAG  0xAA55
#define BNO055_CALIB_DATA_EXIST_ADDR  0x00
#define BNO055_CALIB_DATA_SAVE_ADDR   0x02

#define BNO055_CHIP_ID	              0xA0
#define BNO055_ACC_ID               0xFB
#define BNO055_MAG_ID               0x32
#define BNO055_GYR_ID               0x0F

#define BNO055_OPR_MODE_MASK        (0xF)
#define BNO055_SYS_TRIGGER_MASK     (0xE1)

#define BNO055_ANDROID_ORIENTATION  BIT(7)
#define BNO055_TEMP_CELSIUS         0
#define BNO055_EUL_RADIANS          BIT(2)
#define BNO055_GYR_RADIANS          BIT(1)
#define BNO055_ACC_MPSS             0

#define BNO055_SYS_RST              BIT(5)

#define BNO055_TEMP_ACC             0
#define BNO055_TEMP_GYR             1

#define BNO055_SYS_CALIB_STAT_MSK             (0XC0)
#define BNO055_GYRO_CALIB_STAT_MSK            (0X30)
#define BNO055_ACCEL_CALIB_STAT_MSK           (0X0C)
#define BNO055_MAG_CALIB_STAT_MSK             (0X03)

enum bno055_reg
{
    BNO055_REG_CHIP_ID = 0x00,
    BNO055_REG_ACC_ID  = 0x01,
    BNO055_REG_MAG_ID  = 0x02,
    BNO055_REG_GYR_ID  = 0x03,

    BNO055_REG_SW_REV_ID_LSB = 0x04,
    BNO055_REG_SW_REV_ID_MSB = 0x05,
    BNO055_REG_BL_REV_ID     = 0x06,

    BNO055_REG_PAGE_ID = 0x07,

    /* accel data */
    BNO055_REG_ACC_DATA_X_LSB = 0x08,
    BNO055_REG_ACC_DATA_X_MSB,
    BNO055_REG_ACC_DATA_Y_LSB,
    BNO055_REG_ACC_DATA_Y_MSB,
    BNO055_REG_ACC_DATA_Z_LSB,
    BNO055_REG_ACC_DATA_Z_MSB,

    /* magn data */
    BNO055_REG_MAG_DATA_X_LSB = 0x0E,
    BNO055_REG_MAG_DATA_X_MSB,
    BNO055_REG_MAG_DATA_Y_LSB,
    BNO055_REG_MAG_DATA_Y_MSB,
    BNO055_REG_MAG_DATA_Z_LSB,
    BNO055_REG_MAG_DATA_Z_MSB,

    /* gyr_data */
    BNO055_REG_GYR_DATA_X_LSB = 0x14,
    BNO055_REG_GYR_DATA_X_MSB,
    BNO055_REG_GYR_DATA_Y_LSB,
    BNO055_REG_GYR_DATA_Y_MSB,
    BNO055_REG_GYR_DATA_Z_LSB,
    BNO055_REG_GYR_DATA_Z_MSB,

    /* eul data */
    BNO055_REG_EUL_DATA_X_LSB = 0x1A,
    BNO055_REG_EUL_DATA_X_MSB,
    BNO055_REG_EUL_DATA_Y_LSB,
    BNO055_REG_EUL_DATA_Y_MSB,
    BNO055_REG_EUL_DATA_Z_LSB,
    BNO055_REG_EUL_DATA_Z_MSB,

    BNO055_REG_QUA_DATA_W_LSB = 0x20,
    BNO055_REG_QUA_DATA_W_MSB,
    BNO055_REG_QUA_DATA_X_LSB,
    BNO055_REG_QUA_DATA_X_MSB,
    BNO055_REG_QUA_DATA_Y_LSB,
    BNO055_REG_QUA_DATA_Y_MSB,
    BNO055_REG_QUA_DATA_Z_LSB,
    BNO055_REG_QUA_DATA_Z_MSB,

    /* linear accel data */
    BNO055_REG_LIA_ACC_DATA_X_LSB = 0x28,
    BNO055_REG_LIA_ACC_DATA_X_MSB,
    BNO055_REG_LIA_ACC_DATA_Y_LSB,
    BNO055_REG_LIA_ACC_DATA_Y_MSB,
    BNO055_REG_LIA_ACC_DATA_Z_LSB,
    BNO055_REG_LIA_ACC_DATA_Z_MSB,

    /* gry data */
    BNO055_REG_GRV_DATA_X_LSB = 0x2E,
    BNO055_REG_GRV_DATA_X_MSB,
    BNO055_REG_GRV_DATA_Y_LSB,
    BNO055_REG_GRV_DATA_Y_MSB,
    BNO055_REG_GRV_DATA_Z_LSB,
    BNO055_REG_GRV_DATA_Z_MSB,

    BNO055_REG_TEMP        = 0x34,
    BNO055_REG_CALIB_STAT  = 0x35,
    BNO055_REG_UNIT_SEL    = 0x3B,
    BNO055_REG_OPR_MODE    = 0x3D,
    BNO055_REG_SYS_TRIGGER = 0x3F,
    BNO055_REG_TEMP_SOURCE = 0x40,

    BNO055_REG_ACC_OFFSET_X_LSB = 0x55,
    BNO055_REG_ACC_OFFSET_X_MSB,
    BNO055_REG_ACC_OFFSET_Y_LSB,
    BNO055_REG_ACC_OFFSET_Y_MSB,
    BNO055_REG_ACC_OFFSET_Z_LSB,
    BNO055_REG_ACC_OFFSET_Z_MSB,

    BNO055_REG_MAG_OFFSET_X_LSB = 0x5B,
    BNO055_REG_MAG_OFFSET_X_MSB,
    BNO055_REG_MAG_OFFSET_Y_LSB,
    BNO055_REG_MAG_OFFSET_Y_MSB,
    BNO055_REG_MAG_OFFSET_Z_LSB,
    BNO055_REG_MAG_OFFSET_Z_MSB,

    BNO055_REG_GYR_OFFSET_X_LSB = 0x61,
    BNO055_REG_GYR_OFFSET_X_MSB,
    BNO055_REG_GYR_OFFSET_Y_LSB,
    BNO055_REG_GYR_OFFSET_Y_MSB,
    BNO055_REG_GYR_OFFSET_Z_LSB,
    BNO055_REG_GYR_OFFSET_Z_MSB
};

enum bno055_reg_page1
{
    BNO055_ACC_CONFIG = 0x08,
};

/* struct define */
typedef struct common_data
{
    short x;
    short y;
    short z;

    short scale;
}COMMON_DATA_S;

typedef struct quaterion_data
{
    short w;
    short x;
    short y;
    short z;

    short scale;
}QUATERION_DATA_S;

typedef struct bno055_data
{
    COMMON_DATA_S    accel;        /* acceleration */
    COMMON_DATA_S    lia_accel;    /* linear acceleration */
    COMMON_DATA_S    magn;         /* magnetic */
    COMMON_DATA_S    anglvel;      /* angular velocity */
    COMMON_DATA_S    grv;          /* gravity */
    COMMON_DATA_S    gyr;          /* gyroscope */
    COMMON_DATA_S    eul;          /* euler Angle */
    QUATERION_DATA_S quaterion;    /* quaterion */
    unsigned char    temp;         /* temperature */

    COMMON_DATA_S    accel_offset;
    COMMON_DATA_S    magn_offset;
    COMMON_DATA_S    gyr_offset;
}BNO055_DATA_S;

typedef struct bno055_calib_data
{
    COMMON_DATA_S    accel_offset;
    COMMON_DATA_S    magn_offset;
    COMMON_DATA_S    gyr_offset;
}BNO055_CALIB_DATA_S;

/*
 * Operation modes.  It is important that these are listed in the order
 * they appear in the datasheet, as an index to this table is used to
 * write the actual bits in the operation config register.
 */
typedef enum bno055_opmode
{
	BNO055_CONFIG_MODE,

	/* Non-fusion modes. */
	BNO055_MODE_ACC_ONLY,
	BNO055_MODE_MAG_ONLY,
	BNO055_MODE_GYRO_ONLY,
	BNO055_MODE_ACC_MAG,
	BNO055_MODE_ACC_GYRO,
	BNO055_MODE_MAG_GYRO,
	BNO055_MODE_AMG,

	/* Fusion modes. */
	BNO055_MODE_IMU,
	BNO055_MODE_COMPASS,
	BNO055_MODE_M4G,
	BNO055_MODE_NDOF_FMC_OFF,
	BNO055_MODE_NDOF,

	BNO055_MODE_MAX,
}BNO055_OPMODE_E;

typedef struct bno055_config
{
    BNO055_OPMODE_E op_mode;
    unsigned char unit_sel;
}BNO055_CFG_S;

#define DATA_LENGTH		  64
#define Min(a, b)           (((a) < (b)) ?  (a) : (b))
#define min(a, b)           Min(a, b)

int bno055_set_page(unsigned char page);
int bno055_soft_reset(void);
int bno055_chip_init(int fd, BNO055_CFG_S *bno055_cfg);
/* gui */
int bno055_get_linear_accel(COMMON_DATA_S *lia_accel);
int bno055_get_accel(COMMON_DATA_S *accel);
int bno055_get_magn(COMMON_DATA_S *magn);
int bno055_get_quaterion(QUATERION_DATA_S *quat);
int bno055_get_gravity(COMMON_DATA_S *grv);
int bno055_get_eul(COMMON_DATA_S *eul);
int bno055_get_angular_velocity(COMMON_DATA_S *gyr);
int bno055_read_temp(char *temp);
int check_bno055_connect(void);
void bno055_do_calibrate(void);
bool bno055_get_calibration_stat(void);

unsigned short crc_cctitt(unsigned char *data_p, int length);
int bno055_update_firmware(const char *file_name, int fd);
int bno055_set_accel_offset(COMMON_DATA_S *data);
int bno055_calibration(BNO055_CALIB_DATA_S *data);
int bno055_read_iio_ring(float *data, struct timeval *timestamp);

#endif  /* BNO055_H */