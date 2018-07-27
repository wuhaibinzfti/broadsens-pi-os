/*
 * ADS122 - Texas Instruments Analog-to-Digital Converter
 *
 * Copyright (c) 2016, Intel Corporation.
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for ADS1015 ADC 7-bit I2C slave address:
 *	* 0x48 - ADDR connected to Ground
 *	* 0x49 - ADDR connected to Vdd
 *	* 0x4A - ADDR connected to SDA
 *	* 0x4B - ADDR connected to SCL
 */

#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/types.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#include "ads1015.h"

#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/trigger.h>

#define ADS1015_DRV_NAME "ads1015"
#define ADS122_DRV_NAME "ads122"

#define ADS1015_CONV_REG	0x00
#define ADS1015_CFG_REG		0x01
#define ADS1015_LO_THRESH_REG	0x02
#define ADS1015_HI_THRESH_REG	0x03

#define ADS1015_CFG_COMP_QUE_SHIFT	0
#define ADS1015_CFG_COMP_LAT_SHIFT	2
#define ADS1015_CFG_COMP_POL_SHIFT	3
#define ADS1015_CFG_COMP_MODE_SHIFT	4
#define ADS1015_CFG_DR_SHIFT	5
#define ADS1015_CFG_MOD_SHIFT	8
#define ADS1015_CFG_PGA_SHIFT	9
#define ADS1015_CFG_MUX_SHIFT	12

#define ADS1015_CFG_COMP_QUE_MASK	GENMASK(1, 0)
#define ADS1015_CFG_COMP_LAT_MASK	BIT(2)
#define ADS1015_CFG_COMP_POL_MASK	BIT(3)
#define ADS1015_CFG_COMP_MODE_MASK	BIT(4)
#define ADS1015_CFG_DR_MASK	GENMASK(7, 5)
#define ADS1015_CFG_MOD_MASK	BIT(8)
#define ADS1015_CFG_PGA_MASK	GENMASK(11, 9)
#define ADS1015_CFG_MUX_MASK	GENMASK(14, 12)

#define ADS122_CFG_REG		0x01
#define ADS122_CFG_MOD_SHIFT	3
#define ADS122_CFG_MOD_MASK	BIT(3)

#define ADS122_CFG_DCTN_SHIFT   7
#define ADS122_CFG_DCTN_MASK    BIT(7)

/* Comparator queue and disable field */
#define ADS1015_CFG_COMP_DISABLE	3

/* Comparator polarity field */
#define ADS1015_CFG_COMP_POL_LOW	0
#define ADS1015_CFG_COMP_POL_HIGH	1

/* Comparator mode field */
#define ADS1015_CFG_COMP_MODE_TRAD	0
#define ADS1015_CFG_COMP_MODE_WINDOW	1

/* device operating modes */
#define ADS1015_CONTINUOUS	0
#define ADS1015_SINGLESHOT	1

#define ADS122_SINGLESHOT      0
#define ADS122_CONTINUOUS      1

/* register 0 */
#define ADS122_CFG_MUX_MASK    GENMASK(7, 4)
#define ADS122_CFG_PGA_MASK    GENMASK(3, 1)
#define ADS122_CFG_GAIN_MASK   BIT(s0)


#define ADS122_CFG_PGA_SHIFT   0
#define ADS122_CFG_GAIN_SHIFT  1
#define ADS122_CFG_MUX_SHIFT   4

/* register 1 */
#define ADS122_CFG_TS_BIT_MASK  BIT(0)
#define ADS122_CFG_DR_SHIFT	5

#define ADS1015_SLEEP_DELAY_MS		2000
#define ADS1015_DEFAULT_PGA		2
#define ADS1015_DEFAULT_DATA_RATE	4
#define ADS1015_DEFAULT_CHAN		0

#define ADS122_DEFAULT_OP_MODE      0
#define ADS122_DEFAULT_GAIN         0
#define ADS122_DEFAULT_PGA          0
#define ADS122_DEFAULT_DATA_RATE    1
#define ADS122_DEFAULT_CHAN		 0

#define _DEBUG
#ifdef _DEBUG
#define drv_pr_debug(format,...)   \
        printk("line:%d: "format"", __LINE__, ##__VA_ARGS__)
#else
#define drv_pr_debug(format,...)
#endif

enum chip_ids {
	ADS122_0,
	ADS122_1,
};

enum ads1015_channels {
	ADS1015_AIN0_AIN1 = 0,
	ADS1015_AIN0_AIN3,
	ADS1015_AIN1_AIN3,
	ADS1015_AIN2_AIN3,
	ADS1015_AIN0,
	ADS1015_AIN1,
	ADS1015_AIN2,
	ADS1015_AIN3,
	ADS1015_TIMESTAMP,
};

enum ads122_channels {
	ADS122_PAIN0_NAIN1  = 0,
	ADS122_PAIN0_NAIN2,
	ADS122_PAIN0_NAIN3,
	ADS122_PAIN1_NAIN0,
	ADS122_PAIN1_NAIN2,
	ADS122_PAIN1_NAIN3,
	ADS122_PAIN2_NAIN3,
	ADS122_PAIN3_NAIN2,
	ADS122_PAIN0_NAVSS,
	ADS122_PAIN1_NAVSS,
	ADS122_PAIN2_NAVSS,
	ADS122_PAIN3_NAVSS,
	ADS122_PREF_NREF_4,
	ADS122_AVDD_AVSS_4,
	ADS122_AINP_AVDDVSS,
	ADS122_PREF_NREF,
	ADS122_TEMP,
	ADS122_CHANNEL_NUM,
};

enum
{
	ADS122_CMD_RESET      = 0x06,           /* reset the device [0000 011x] */
	ADS122_CMD_START      = 0x08,           /* start or restart conversions [0000 100x] */
	ADS122_CMD_POWERDOWN  = 0x02,           /* enter power-down mode [0000 001x] */
	ADS122_CMD_RDATA      = 0x10,           /* read data by command [0001 xxxx] */
	ADS122_CMD_RREG       = 0x20,           /* read register at address rr [0010 rrxx] */
	ADS122_CMD_WREG       = 0x40,           /* write register at address rr [0100 rrxx] */
};

enum
{
    ADS122_REG_0   = 0x00 << 2,
    ADS122_REG_1   = 0x01 << 2,
    ADS122_REG_2   = 0x02 << 2,
    ADS122_REG_3   = 0x03 << 2,
};

enum
{
	ADS122_NORMAL_20SPS    = 0,
	ADS122_NORMAL_45SPS    ,
	ADS122_NORMAL_90SPS    ,
	ADS122_NORMAL_175SPS   ,
	ADS122_NORMAL_330SPS   ,
	ADS122_NORMAL_600SPS   ,
	ADS122_NORMAL_1000SPS  ,
	ADS122_NORMAL_RESERVED ,

	ADS122_TURBO_40SPS    ,
	ADS122_TURBO_90SPS    ,
	ADS122_TURBO_180SPS   ,
	ADS122_TURBO_350SPS   ,
	ADS122_TURBO_660SPS   ,
	ADS122_TURBO_1200SPS  ,
	ADS122_TURBO_2000SPS  ,
	ADS122_TURBO_RESERVED ,
};

struct ads1015_channel_data {
    bool enabled;
    unsigned int pga;
    unsigned int gain;
    unsigned int op_mode;
    unsigned int data_rate;

    unsigned int dcnt;
};

struct ads122_channel_data {
    bool enabled;
    int tick;
    int interval;
    int pga;
    int gain;
    int mode;        /* operation mode 0:normal mode, 1:turbo mode */
    int dr;          /* data rate */
    int dcnt;
};

struct ads1015_platform_data {
	struct ads1015_channel_data channel_data[ADS122_CHANNEL_NUM];
};

static const unsigned int ads1015_data_rate[] = {
	128, 250, 490, 920, 1600, 2400, 3300, 3300
};

static const unsigned int ads1115_data_rate[] = {
	8, 16, 32, 64, 128, 250, 475, 860
};

/* unit:SPS */
static const unsigned int ads122_dr_tbl[][7] = {
    {20, 45, 90,  175, 330, 660,  1000},    /* normal rate */
    {40, 90, 180, 350, 660, 1200, 2000}     /* turbo mode */
};


/*
 * Translation from PGA bits to full-scale positive and negative input voltage
 * range in mV
 */
static int ads1015_fullscale_range[] = {
	6144, 4096, 2048, 1024, 512, 256, 256, 256
};

/*
 * Translation from PGA bits to full-scale positive and negative input voltage
 * range in mV, unit:mv
 */
static int32_t ads122_fullscale_range[] = {
    2048, 1024, 512, 256, 128, 64, 32, 16
};

#define ADS122_MIN_INTERVAL 1000

/*
 * Translation from COMP_QUE field value to the number of successive readings
 * exceed the threshold values before an interrupt is generated
 */
static const int ads1015_comp_queue[] = {1, 2, 4};

static const struct iio_event_spec ads1015_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) |
				BIT(IIO_EV_INFO_PERIOD),
	},
};

#define ADS1015_V_CHAN(_chan, _addr) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.shift = 4,					\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan,				\
}

#define ADS1015_V_DIFF_CHAN(_chan, _chan2, _addr) {		\
	.type = IIO_VOLTAGE,					\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 12,					\
		.storagebits = 16,				\
		.shift = 4,					\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}

#define ADS1115_V_CHAN(_chan, _addr) {				\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan,				\
}

#define ADS1115_V_DIFF_CHAN(_chan, _chan2, _addr) {		\
	.type = IIO_VOLTAGE,					\
	.differential = 1,					\
	.indexed = 1,						\
	.address = _addr,					\
	.channel = _chan,					\
	.channel2 = _chan2,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _addr,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_CPU,				\
	},							\
	.event_spec = ads1015_events,				\
	.num_event_specs = ARRAY_SIZE(ads1015_events),		\
	.datasheet_name = "AIN"#_chan"-AIN"#_chan2,		\
}

#define ADS122_V_CHAN(_chan, _addr, _si) {                   \
    .type = IIO_VOLTAGE,                                \
    .indexed = 1,						                \
    .address = _addr,					                \
    .channel = _chan,					                \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |      \
                          BIT(IIO_CHAN_INFO_SCALE) |    \
                          BIT(IIO_CHAN_INFO_SAMP_FREQ), \
    .scan_index = _si,                                \
    .scan_type = {                                      \
        .sign = 's',                                    \
        .realbits = 24,                                 \
        .storagebits = 32,                              \
        .endianness = IIO_CPU,	                         \
    },                                                  \
    .event_spec = ads1015_events,                       \
    .num_event_specs = ARRAY_SIZE(ads1015_events),      \
    .datasheet_name = "AIN"#_chan,				       \
}

#define ADS122_V_DIFF_CHAN(_chan, _chan2, _addr, _si) { \
    .type = IIO_VOLTAGE,                                \
    .differential = 1,	                                  \
    .indexed = 1,                                       \
    .address = _addr,                                   \
    .channel = _chan,                                   \
    .channel2 = _chan2,                                 \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |      \
                          BIT(IIO_CHAN_INFO_SCALE) |    \
                          BIT(IIO_CHAN_INFO_SAMP_FREQ), \
    .scan_index = _si,                                \
    .scan_type = {                                      \
        .sign = 's',                                    \
        .realbits = 24,                                 \
        .storagebits = 32,                              \
        .endianness = IIO_CPU,	                         \
    },                                                  \
    .event_spec = ads1015_events,                       \
    .num_event_specs = ARRAY_SIZE(ads1015_events),      \
    .datasheet_name = "PAIN"#_chan"-NAIN"#_chan2,	   \
}

#define ADS122_TEMP_CHAN(_addr, _si) {                  \
    .type = IIO_TEMP,					                \
    .indexed = 1,                                       \
    .channel = 0,                                       \
    .address = _addr,                                   \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |      \
                          BIT(IIO_CHAN_INFO_SCALE),     \
    .scan_index = _si,                                  \
    .scan_type = {                                      \
        .sign = 's',                                    \
        .realbits = 14,                                 \
        .shift = 2,                                     \
        .storagebits = 16,                              \
    },                                                  \
}

#define ADS122_ADC_CTL(_si) {					\
	.type = IIO_VOLTAGE,						\
	.channel = -1,							\
	.scan_index = _si,						\
	.scan_type = {							\
		.sign = 's',						    \
		.realbits = 24,					    \
		.storagebits = 32,					\
		.endianness = IIO_CPU,	                  \
     },						               	\
}

struct ads1015_thresh_data {
	unsigned int comp_queue;
	int high_thresh;
	int low_thresh;
};

struct ads122_data {
    struct i2c_client *client;
    struct regmap *regmap;
    /*
    * Protects ADC ops, e.g: concurrent sysfs/buffered
    * data reads, configuration updates
    */
    struct mutex lock;
    struct ads122_channel_data channel_data[ADS122_CHANNEL_NUM];

    unsigned int current_channel;
    unsigned int event_channel;
    unsigned int comp_mode;
    struct ads1015_thresh_data thresh_data[ADS122_CHANNEL_NUM];

    const unsigned int (*dr_tbl)[7];
    /*
    * Set to true when the ADC is switched to the continuous-conversion
    * mode and exits from a power-down state.  This flag is used to avoid
    * getting the stale result from the conversion register.
    */
    bool conv_invalid;
    bool block_mode;
    struct work_struct work;
    struct hrtimer timer;
    ktime_t ktime;

    unsigned long tick;    /* unit:us */

    struct iio_trigger *trigger;
};

static bool ads1015_event_channel_enabled(struct ads122_data *data)
{
	return (data->event_channel != ADS122_CHANNEL_NUM);
}

static void ads1015_event_channel_enable(struct ads122_data *data, int chan,
					 int comp_mode)
{
	WARN_ON(ads1015_event_channel_enabled(data));

	data->event_channel = chan;
	data->comp_mode = comp_mode;
}

static void ads1015_event_channel_disable(struct ads122_data *data, int chan)
{
	data->event_channel = ADS122_CHANNEL_NUM;
}

static bool ads1015_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADS1015_CFG_REG:
	case ADS1015_LO_THRESH_REG:
	case ADS1015_HI_THRESH_REG:
		return true;
	default:
		return false;
	}
}

static bool ads122_is_writeable_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config ads1015_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = ADS1015_HI_THRESH_REG,
	.writeable_reg = ads1015_is_writeable_reg,
};

static const struct regmap_config ads122_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = ads122_is_writeable_reg,
};

static const struct iio_chan_spec ads1015_channels[] = {
	ADS1015_V_DIFF_CHAN(0, 1, ADS1015_AIN0_AIN1),
	ADS1015_V_DIFF_CHAN(0, 3, ADS1015_AIN0_AIN3),
	ADS1015_V_DIFF_CHAN(1, 3, ADS1015_AIN1_AIN3),
	ADS1015_V_DIFF_CHAN(2, 3, ADS1015_AIN2_AIN3),
	ADS1015_V_CHAN(0, ADS1015_AIN0),
	ADS1015_V_CHAN(1, ADS1015_AIN1),
	ADS1015_V_CHAN(2, ADS1015_AIN2),
	ADS1015_V_CHAN(3, ADS1015_AIN3),
	IIO_CHAN_SOFT_TIMESTAMP(ADS1015_TIMESTAMP),
};

static const struct iio_chan_spec ads122_chnnels[] = {
    //ADS122_V_DIFF_CHAN(0, 1, ADS122_PAIN0_NAIN1, -1),
    //ADS122_V_DIFF_CHAN(0, 2, ADS122_PAIN0_NAIN2, -1),
    //ADS122_V_DIFF_CHAN(0, 3, ADS122_PAIN0_NAIN3, -1),
    ADS122_V_DIFF_CHAN(1, 0, ADS122_PAIN1_NAIN0, -1),
    //ADS122_V_DIFF_CHAN(1, 2, ADS122_PAIN1_NAIN2, -1),
    //ADS122_V_DIFF_CHAN(1, 3, ADS122_PAIN1_NAIN3, -1),
    //ADS122_V_DIFF_CHAN(2, 3, ADS122_PAIN2_NAIN3, -1),
    ADS122_V_DIFF_CHAN(3, 2, ADS122_PAIN3_NAIN2, -1),
    //ADS122_V_CHAN(0, ADS122_PAIN0_NAVSS, -1),
    //ADS122_V_CHAN(1, ADS122_PAIN1_NAVSS, -1),
    //ADS122_V_CHAN(2, ADS122_PAIN2_NAVSS, -1),
    //ADS122_V_CHAN(3, ADS122_PAIN3_NAVSS, -1),
    //ADS122_V_CHAN(4, ADS122_PREF_NREF_4, -1),
    //ADS122_V_CHAN(5, ADS122_AVDD_AVSS_4, -1),
    //ADS122_V_CHAN(6, ADS122_AINP_AVDDVSS, -1),
    //ADS122_V_CHAN(7, ADS122_PREF_NREF, -1),
    ADS122_TEMP_CHAN(ADS122_TEMP, -1),
    ADS122_ADC_CTL(0),
    IIO_CHAN_SOFT_TIMESTAMP(1),
};


static int ads122_write_cmd(struct i2c_client *client, unsigned char cmd)
{
    int ret;
    struct i2c_msg msg = {
            .addr = client->addr,
            .flags= 0,
            .len = 1,
            .buf = &cmd,
    };

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0) {
        dev_err(&client->dev, "i2c transfer error!\n");
    }

    return ret;
}

static int ads1015_set_power_state(struct ads122_data *data, bool on)
{
	int ret;
	struct device *dev = regmap_get_device(data->regmap);

	if (on) {
		ret = pm_runtime_get_sync(dev);
		if (ret < 0)
			pm_runtime_put_noidle(dev);
	} else {
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_autosuspend(dev);
	}

	return ret < 0 ? ret : 0;
}

static int ads122_get_adc_result(struct ads122_data *data, int chan, __u8 *dst)
{
    __u8 buf[10] = {0};
    int ret, dr, mode, conv_time, old, cfg;

    if (chan < 0 || chan >= ADS122_CHANNEL_NUM) {
        return -EINVAL;
    }

    if (ADS122_TEMP == chan) {
        ret = regmap_update_bits(data->regmap, ADS122_CMD_WREG + ADS122_REG_1,
    			                    ADS122_CFG_TS_BIT_MASK, 1);
        if (ret) {
            return -EIO;
        }
    } else {
        /* reg0: mux[7:4] gain[3:1] pga[0] */
        ret = regmap_read(data->regmap, ADS122_CMD_RREG + ADS122_REG_0, &old);
        if (ret) {
            return -EIO;
        }
        cfg = chan << ADS122_CFG_MUX_SHIFT |
              data->channel_data[chan].gain << ADS122_CFG_GAIN_SHIFT |
              data->channel_data[chan].pga << ADS122_CFG_PGA_SHIFT;

        if (old != cfg) {
            ret = regmap_write(data->regmap, ADS122_CMD_WREG + ADS122_REG_0, cfg);
            if (ret) {
                return -EIO;
            }
        }

        /* register1: dr[7:5] mode[4] cm[3] vref[2:1] ts[0] */
        /* ts disable */
        regmap_read(data->regmap, ADS122_CMD_RREG + ADS122_REG_1, &old);
        cfg = old & 0xFE;
        regmap_write(data->regmap, ADS122_CMD_WREG + ADS122_REG_1, cfg);
    }

    dr = data->channel_data[0].dr;

    /* start conver */
    ads122_write_cmd(data->client, ADS122_CMD_START);

    mode = data->channel_data[0].mode;

    conv_time = DIV_ROUND_UP(USEC_PER_SEC, data->dr_tbl[mode][dr]);
    conv_time += conv_time / 10;  /* 10% internal clock inaccuracy */
    usleep_range(conv_time, conv_time + 1);

    /* get adc conver value */
    ret = regmap_read(data->regmap, ADS122_CMD_RREG + ADS122_REG_2, &old);
    if (ret) {
        return -EIO;
    }
    if (old & 0x80) {
        ret = regmap_bulk_read(data->regmap, ADS122_CMD_RDATA,
                               data->channel_data[0].dcnt ? buf : buf + 1,
                               data->channel_data[0].dcnt ? 4 : 3);
        if (ret) {
            return -EIO;
        }
    }
    else {
        dev_err(&data->client->dev, "conver not finish\n");
        return -EAGAIN;
    }

    memcpy(dst, buf + 1, 3);
    return 3;
}

static void ads122_work(struct work_struct *_work)
{
    int chan;
    __u8 *buf, src[8];

    struct ads122_data *data = container_of(_work, struct ads122_data, work);
    struct iio_dev *indio_dev = i2c_get_clientdata(data->client);

    if (indio_dev->scan_bytes <= 0) {
        goto err;
    }

    buf = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);
    if (!buf) {
        goto err;
    }

    if (test_bit(0, indio_dev->active_scan_mask)) {
        for (chan = 0; chan < ADS122_CHANNEL_NUM; chan++) {
            struct ads122_channel_data *chan_data = &data->channel_data[chan];
            if (!chan_data->enabled) {
                continue;
            }
            if (chan_data->tick < chan_data->interval) {
                chan_data->tick++;
                continue;
            } else {
                chan_data->tick = 0;
            }
            if (ads122_get_adc_result(data, chan, src) < 0) {
                break;
            }

            /* push data to ring buffer */
            *((uint32_t *)buf) = (__u8)chan << 24 | src[0] << 16 | src[1] << 8 | src[2];
            if (indio_dev->scan_timestamp) {
                iio_push_to_buffers_with_timestamp(indio_dev, buf, iio_get_time_ns(indio_dev));
            } else {
                iio_push_to_buffers(indio_dev, buf);
            }
        }
    }

    kfree(buf);
err:
    mutex_lock(&data->lock);
    data->conv_invalid = true;
    mutex_unlock(&data->lock);
    return;
}

enum hrtimer_restart ads122_hrtimer_callback(struct hrtimer *hr_timer)
{
    struct ads122_data *data = container_of(hr_timer, struct ads122_data, timer);

    mutex_lock(&data->lock);
    if (data->conv_invalid) {
        data->conv_invalid = false;
        schedule_work(&data->work);
    }
    mutex_unlock(&data->lock);

    hrtimer_forward_now(&data->timer, data->ktime);
    return HRTIMER_RESTART;
}

static int ads1015_set_scale(struct ads122_data *data,
			     struct iio_chan_spec const *chan,
			     int scale, int uscale)
{
	int i;
	int fullscale = div_s64((scale * 1000000LL + uscale) <<
				(chan->scan_type.realbits - 1), 1000000);

	for (i = 0; i < ARRAY_SIZE(ads1015_fullscale_range); i++) {
		if (ads1015_fullscale_range[i] == fullscale) {
			data->channel_data[chan->address].pga = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int ads1015_set_data_rate(struct ads122_data *data, int chan, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads122_dr_tbl); i++) {
        if (data->dr_tbl[0][i] == rate) {
            data->channel_data[chan].dr = i;
            return 0;
        }
	}

	return -EINVAL;
}

static int ads122_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
                            int *val, int *val2, long mask)
{
    int ret, idx;
    struct ads122_data *data = iio_priv(indio_dev);
    __u8 buf[4];

    mutex_lock(&data->lock);
    switch (mask) {
        case IIO_CHAN_INFO_RAW: {
            ret = ads122_get_adc_result(data, chan->address, buf);
            if (ret < 0)
                break;
            *val = buf[0] << 16 | buf[1] << 8 | buf[2];
            *val = sign_extend32(*val, chan->scan_type.realbits - 1);

            ret = IIO_VAL_INT;
            break;
        }
        case IIO_CHAN_INFO_SCALE: {
            if (ADS122_TEMP == chan->address) {
                *val = 1;
                *val2 = 32;
                ret = IIO_VAL_FRACTIONAL;
            } else {
                idx = data->channel_data[chan->address].gain;
                *val = ads122_fullscale_range[idx];
                *val2 = chan->scan_type.realbits - 1;
                ret = IIO_VAL_FRACTIONAL_LOG2;
            }
            break;
        }
        case IIO_CHAN_INFO_SAMP_FREQ: {
            idx = data->channel_data[chan->address].dr;
            *val = data->dr_tbl[0][idx];
            ret = IIO_VAL_INT;
            break;
        }
        default: {
            ret = -EINVAL;
            break;
        }
    }
    mutex_unlock(&data->lock);

    return ret;
}

static int ads122_write_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan, int val,
                            int val2, long mask)
{
	struct ads122_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		ret = ads1015_set_scale(data, chan, val, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ads1015_set_data_rate(data, chan->address, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_read_event(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct ads122_data *data = iio_priv(indio_dev);
	int ret;
	unsigned int comp_queue;
	int period;
	int dr;

	mutex_lock(&data->lock);

    switch (info) {
        case IIO_EV_INFO_VALUE:
            *val = (dir == IIO_EV_DIR_RISING) ? data->thresh_data[chan->address].high_thresh :
                                                data->thresh_data[chan->address].low_thresh;
            ret = IIO_VAL_INT;
            break;
        case IIO_EV_INFO_PERIOD:
            dr = data->channel_data[chan->address].dr;
            comp_queue = data->thresh_data[chan->address].comp_queue;
            period = ads1015_comp_queue[comp_queue] * USEC_PER_SEC / data->dr_tbl[0][dr];

            *val = period / USEC_PER_SEC;
            *val2 = period % USEC_PER_SEC;
            ret = IIO_VAL_INT_PLUS_MICRO;
            break;
        default:
            ret = -EINVAL;
            break;
    }

	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_write_event(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct ads122_data *data = iio_priv(indio_dev);
	int realbits = chan->scan_type.realbits;
	int ret = 0;
	long long period;
	int i;
	int dr;

	mutex_lock(&data->lock);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (val >= 1 << (realbits - 1) || val < -1 << (realbits - 1)) {
			ret = -EINVAL;
			break;
		}
		if (dir == IIO_EV_DIR_RISING)
			data->thresh_data[chan->address].high_thresh = val;
		else
			data->thresh_data[chan->address].low_thresh = val;
		break;
	case IIO_EV_INFO_PERIOD:
		dr = data->channel_data[chan->address].dr;
		period = val * USEC_PER_SEC + val2;

		for (i = 0; i < ARRAY_SIZE(ads1015_comp_queue) - 1; i++) {
			if (period <= ads1015_comp_queue[i] *
					USEC_PER_SEC / data->dr_tbl[0][dr])
				break;
		}
		data->thresh_data[chan->address].comp_queue = i;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&data->lock);

	return ret;
}

static int ads1015_read_event_config(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct ads122_data *data = iio_priv(indio_dev);
	int ret = 0;

	mutex_lock(&data->lock);
	if (data->event_channel == chan->address) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			ret = 1;
			break;
		case IIO_EV_DIR_EITHER:
			ret = (data->comp_mode == ADS1015_CFG_COMP_MODE_WINDOW);
			break;
		default:
			ret = -EINVAL;
			break;
		}
	}
	mutex_unlock(&data->lock);

	return ret;
}

static ssize_t ads122_show_tick(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct ads122_data *data = iio_priv(indio_dev);

    return sprintf(buf, "%ld\n", data->tick);
}

static ssize_t ads122_store_tick(struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t len)
{
    int ret;
    unsigned long tick;
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct ads122_data *data = iio_priv(indio_dev);

    ret = kstrtoul(buf, 10, &tick);
    if (ret)
        return ret;

    if (tick < 1000)
        return -EINVAL;

    mutex_lock(&data->lock);
    data->ktime = ktime_set(tick / 1000000, (tick % 1000000) * 1000);
    data->tick = tick;
    mutex_unlock(&data->lock);

    return len;
}

static IIO_DEVICE_ATTR(tick, S_IRUGO | S_IWUSR,
                       ads122_show_tick,
                       ads122_store_tick,
                       0);


static int ads1015_enable_event_config(struct ads122_data *data,
	const struct iio_chan_spec *chan, int comp_mode)
{
	int low_thresh = data->thresh_data[chan->address].low_thresh;
	int high_thresh = data->thresh_data[chan->address].high_thresh;
	int ret;

	if (ads1015_event_channel_enabled(data)) {
		if (data->event_channel != chan->address ||
			(data->comp_mode == ADS1015_CFG_COMP_MODE_TRAD &&
				comp_mode == ADS1015_CFG_COMP_MODE_WINDOW))
			return -EBUSY;

		return 0;
	}

	if (comp_mode == ADS1015_CFG_COMP_MODE_TRAD) {
		low_thresh = max(-1 << (chan->scan_type.realbits - 1),
				high_thresh - 1);
	}
	ret = regmap_write(data->regmap, ADS1015_LO_THRESH_REG,
			low_thresh << chan->scan_type.shift);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, ADS1015_HI_THRESH_REG,
			high_thresh << chan->scan_type.shift);
	if (ret)
		return ret;

	ret = ads1015_set_power_state(data, true);
	if (ret < 0)
		return ret;

	ads1015_event_channel_enable(data, chan->address, comp_mode);

	//ret = ads122_get_adc_result(data, chan->address, &val);
	if (ret) {
		ads1015_event_channel_disable(data, chan->address);
		ads1015_set_power_state(data, false);
	}

	return ret;
}

static int ads1015_disable_event_config(struct ads122_data *data,
	const struct iio_chan_spec *chan, int comp_mode)
{
	int ret;

	if (!ads1015_event_channel_enabled(data))
		return 0;

	if (data->event_channel != chan->address)
		return 0;

	if (data->comp_mode == ADS1015_CFG_COMP_MODE_TRAD &&
			comp_mode == ADS1015_CFG_COMP_MODE_WINDOW)
		return 0;

	ret = regmap_update_bits(data->regmap, ADS1015_CFG_REG,
				ADS1015_CFG_COMP_QUE_MASK,
				ADS1015_CFG_COMP_DISABLE <<
					ADS1015_CFG_COMP_QUE_SHIFT);
	if (ret)
		return ret;

	ads1015_event_channel_disable(data, chan->address);

	return ads1015_set_power_state(data, false);
}

static int ads1015_write_event_config(struct iio_dev *indio_dev,
                                      const struct iio_chan_spec *chan, enum iio_event_type type,
                                      enum iio_event_direction dir, int state)
{
    int ret;
    struct ads122_data *data = iio_priv(indio_dev);
    int comp_mode = (dir == IIO_EV_DIR_EITHER) ? ADS1015_CFG_COMP_MODE_WINDOW : ADS1015_CFG_COMP_MODE_TRAD;

    mutex_lock(&data->lock);

    /* Prevent from enabling both buffer and event at a time */
    ret = 0; //iio_device_claim_direct_mode(indio_dev);
    if (ret) {
        mutex_unlock(&data->lock);
        return ret;
    }

    if (state)
        ret = ads1015_enable_event_config(data, chan, comp_mode);
    else
        ret = ads1015_disable_event_config(data, chan, comp_mode);

    //iio_device_release_direct_mode(indio_dev);
    mutex_unlock(&data->lock);

    return ret;
}

static int ads1015_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ads122_data *data = iio_priv(indio_dev);

	/* Prevent from enabling both buffer and event at a time */
	if (ads1015_event_channel_enabled(data))
		return -EBUSY;

	return ads1015_set_power_state(iio_priv(indio_dev), true);
}

static int ads1015_buffer_postdisable(struct iio_dev *indio_dev)
{
	return ads1015_set_power_state(iio_priv(indio_dev), false);
}

static const struct iio_buffer_setup_ops ads1015_buffer_setup_ops = {
	.preenable	= ads1015_buffer_preenable,
	.postenable	= iio_triggered_buffer_postenable,
	.predisable	= iio_triggered_buffer_predisable,
	.postdisable	= ads1015_buffer_postdisable,
	.validate_scan_mask = &iio_validate_scan_mask_onehot,
};


static IIO_CONST_ATTR_NAMED(ads122_scale_available, scale_available,
	"0.1875 0.125 0.0625 0.03125 0.015625 0.007813");
static IIO_CONST_ATTR_NAMED(ads122_sampling_frequency_available,
	sampling_frequency_available, "stand: 20 45 90 175 330 660 1000\nturbo: 40 90 180 350 660 1200 2000");


static struct attribute *ads122_attributes[] = {
	&iio_const_attr_ads122_scale_available.dev_attr.attr,
	&iio_const_attr_ads122_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_tick.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads122_attribute_group = {
	.attrs = ads122_attributes,
};

static const struct iio_info ads122_info = {
	.read_raw	= ads122_read_raw,
	.write_raw	= ads122_write_raw,
	.read_event_value = ads1015_read_event,
	.write_event_value = ads1015_write_event,
	.read_event_config = ads1015_read_event_config,
	.write_event_config = ads1015_write_event_config,
	.attrs          = &ads122_attribute_group,
};

#if 0
#ifdef CONFIG_OF
static int ads1015_get_channels_config_of(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads1015_data *data = iio_priv(indio_dev);
	struct device_node *node;

	if (!client->dev.of_node ||
	    !of_get_next_child(client->dev.of_node, NULL))
		return -EINVAL;

	for_each_child_of_node(client->dev.of_node, node) {
		u32 pval;
		unsigned int channel;
		unsigned int pga = ADS1015_DEFAULT_PGA;
		unsigned int data_rate = ADS1015_DEFAULT_DATA_RATE;

		if (of_property_read_u32(node, "reg", &pval)) {
			dev_err(&client->dev, "invalid reg on %pOF\n",
				node);
			continue;
		}

		channel = pval;
		if (channel >= ADS122_CHANNELS) {
			dev_err(&client->dev,
				"invalid channel index %d on %pOF\n",
				channel, node);
			continue;
		}

		if (!of_property_read_u32(node, "ti,gain", &pval)) {
			pga = pval;
			if (pga > 6) {
				dev_err(&client->dev, "invalid gain on %pOF\n",
					node);
				of_node_put(node);
				return -EINVAL;
			}
		}

		if (!of_property_read_u32(node, "ti,datarate", &pval)) {
			data_rate = pval;
			if (data_rate > 7) {
				dev_err(&client->dev,
					"invalid data_rate on %pOF\n",
					node);
				of_node_put(node);
				return -EINVAL;
			}
		}

		data->channel_data[channel].pga = pga;
		data->channel_data[channel].data_rate = data_rate;
	}

	return 0;
}
#endif
#endif

static void ads122_get_channels_config(struct i2c_client *client)
{
	int i;

	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122_data *data = iio_priv(indio_dev);
	struct ads1015_platform_data *pdata = dev_get_platdata(&client->dev);

	/* prefer platform data */
	if (pdata) {
		memcpy(data->channel_data, pdata->channel_data, sizeof(data->channel_data));
		return;
	}

#if 0
#ifdef CONFIG_OF
	if (!ads1015_get_channels_config_of(client))
		return;
#endif
#endif

	/* fallback on default configuration */
    for (i = 0; i < ADS122_CHANNEL_NUM; i++) {
        data->channel_data[i].pga = ADS122_DEFAULT_PGA;
        data->channel_data[i].gain = ADS122_DEFAULT_GAIN;
        data->channel_data[i].mode = ADS122_DEFAULT_OP_MODE;
        data->channel_data[i].dr = 0;
    }

    data->channel_data[ADS122_PAIN1_NAIN0].enabled = true;
    data->channel_data[ADS122_PAIN1_NAIN0].tick = 0;
    data->channel_data[ADS122_PAIN1_NAIN0].interval = 0;

    data->channel_data[ADS122_PAIN3_NAIN2].enabled = true;
    data->channel_data[ADS122_PAIN3_NAIN2].tick = 0;
    data->channel_data[ADS122_PAIN3_NAIN2].interval = 0;

    data->channel_data[ADS122_TEMP].enabled = true;
    data->channel_data[ADS122_TEMP].tick = 0;
    data->channel_data[ADS122_TEMP].interval = 10;

    return;
}

static int ads122_set_conv_mode(struct ads122_data *data, int mode)
{
    return regmap_update_bits(data->regmap, ADS122_REG_1 + ADS122_CMD_WREG,
                              ADS122_CFG_MOD_MASK, mode);
}

static int ads122_ring_preenable(struct iio_dev *indio_dev)
{
    if (bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength))
        return -EINVAL;
    return 0;
}

static int ads122_ring_postenable(struct iio_dev *indio_dev)
{
    struct ads122_data *data = iio_priv(indio_dev);

    mutex_lock(&data->lock);
    data->ktime = ktime_set(data->tick / 1000000, (data->tick % 1000000) * 1000);
    mutex_unlock(&data->lock);

    hrtimer_start(&data->timer, data->ktime, HRTIMER_MODE_REL);

    return 0;
}

static int ads122_ring_postdisable(struct iio_dev *indio_dev)
{
    struct ads122_data *data = iio_priv(indio_dev);

    hrtimer_cancel(&data->timer);
    cancel_work_sync(&data->work);
    return 0;
}

static const struct iio_buffer_setup_ops ads122_ring_setup_ops =
{
    .preenable = ads122_ring_preenable,
    .postenable = ads122_ring_postenable,
    .postdisable = ads122_ring_postdisable,
};

static int ads122_register_ring(struct iio_dev *indio_dev)
{
    struct iio_buffer *ring;

    ring = iio_kfifo_allocate();
    if (!ring)
        return -ENOMEM;

    iio_device_attach_buffer(indio_dev, ring);
    indio_dev->setup_ops = &ads122_ring_setup_ops;

    return 0;
}

static void ads122_hrtimer_init(struct hrtimer *timer)
{
    hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    timer->function = ads122_hrtimer_callback;
}

struct i2c_client *ads122_1_client = NULL;
struct i2c_client *ads122_2_client = NULL;

static int ads122_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i, ret;
    struct iio_dev *indio_dev;
    struct ads122_data *data;
    enum chip_ids chip;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);
    i2c_set_clientdata(client, indio_dev);

    mutex_init(&data->lock);

    indio_dev->dev.parent = &client->dev;
    indio_dev->dev.of_node = client->dev.of_node;
    indio_dev->modes = INDIO_BUFFER_SOFTWARE | INDIO_DIRECT_MODE;

    indio_dev->channels = ads122_chnnels;
    indio_dev->num_channels = ARRAY_SIZE(ads122_chnnels);
    indio_dev->info = &ads122_info;
    data->dr_tbl = ads122_dr_tbl;
    data->client = client;
    chip = id->driver_data;
    switch (chip) {
        case ADS122_0:
            ads122_1_client = client;
            indio_dev->name = "ads122-0";
            break;
        case ADS122_1:
            ads122_2_client = client;
            indio_dev->name = "ads122-1";
            break;
        default:
            break;
    }

    data->event_channel = ADS122_CHANNEL_NUM;
    /*
     * Set default lower and upper threshold to min and max value
     * respectively.
     */
    for (i = 0; i < ADS122_CHANNEL_NUM; i++) {
        int realbits = indio_dev->channels[i].scan_type.realbits;

        data->thresh_data[i].low_thresh = -1 << (realbits - 1);
        data->thresh_data[i].high_thresh = (1 << (realbits - 1)) - 1;
    }

    /* we need to keep this ABI the same as used by hwmon ADS1015 driver */
    ads122_get_channels_config(client);

    data->regmap = devm_regmap_init_i2c(client, &ads122_regmap_config);
        if (IS_ERR(data->regmap)) {
            dev_err(&client->dev, "Failed to allocate register map\n");
            return PTR_ERR(data->regmap);
    }

    regcache_cache_bypass(data->regmap, true);

    /* reset chip for configure */
    ads122_write_cmd(client, ADS122_CMD_RESET);
    msleep(10);

    ret = ads122_set_conv_mode(data, ADS122_SINGLESHOT);
    if (ret) {
        dev_err(&client->dev, "Failed to set continuous mode\n");
        return ret;
    }
    data->conv_invalid = true;

    ret = ads122_register_ring(indio_dev);
    if (ret)
        return ret;

    ret = iio_device_register(indio_dev);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to register IIO device\n");
        return ret;
    }

    /* work quene */
    INIT_WORK(&data->work, ads122_work);
    /* hr timer init */
    ads122_hrtimer_init(&data->timer);
    data->tick = 1000000;

    return 0;
}

static int ads122_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ads122_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

#if 0
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);
#endif

	/* power down single shot mode */
	return ads122_set_conv_mode(data, ADS122_SINGLESHOT);
}

#ifdef CONFIG_PM
static int ads1015_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads122_data *data = iio_priv(indio_dev);

	return ads122_set_conv_mode(data, ADS122_SINGLESHOT);
}

static int ads122_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads122_data *data = iio_priv(indio_dev);

	return ads122_set_conv_mode(data, ADS122_SINGLESHOT);
}

static int ads1015_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads122_data *data = iio_priv(indio_dev);
	int ret;

	ret = ads122_set_conv_mode(data, ADS122_CONTINUOUS);
	if (!ret)
		data->conv_invalid = true;

	return ret;
}

static int ads122_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct ads122_data *data = iio_priv(indio_dev);
	int ret;

	ret = ads122_set_conv_mode(data, ADS122_CONTINUOUS);
	if (!ret)
		data->conv_invalid = true;

	return ret;
}
#endif

static const struct dev_pm_ops ads1015_pm_ops = {
	SET_RUNTIME_PM_OPS(ads1015_runtime_suspend,
			          ads1015_runtime_resume, NULL)
};

static const struct dev_pm_ops ads122_pm_ops = {
	SET_RUNTIME_PM_OPS(ads122_runtime_suspend,
			          ads122_runtime_resume, NULL)
};

static const struct i2c_device_id ads122_id[] = {
    {"ads122-0", ADS122_0},
    {"ads122-1", ADS122_1},
    {}
};
MODULE_DEVICE_TABLE(i2c, ads122_id);

static struct i2c_driver ads122_driver = {
	.driver = {
		.name = ADS122_DRV_NAME,
		.pm = &ads122_pm_ops,
	},
	.probe		= ads122_probe,
	.remove		= ads122_remove,
	.id_table	= ads122_id,
};

static struct i2c_board_info ads122_device[] = {
    { I2C_BOARD_INFO("ads122-0", 0x40) },
    { I2C_BOARD_INFO("ads122-1", 0x44) },
};

static int __init ads122_init(void)
{
    int i;
    struct i2c_adapter *adap;
    struct i2c_client *client;

    adap = i2c_get_adapter(1);
    if (!adap) {
        printk("i2c adapter %d\n", 1);
        return -ENODEV;
    }

    for (i = 0; i < ARRAY_SIZE(ads122_device); i++) {
        client = i2c_new_device(adap, &ads122_device[i]);
        if (!client) {
            printk("get i2c client %s @ 0x%02x fail!\n", ads122_device[i].type, ads122_device[i].addr);
            return -ENODEV;
        }
    }
    i2c_put_adapter(adap);
    i2c_add_driver(&ads122_driver);
    printk("ads122 init success!\n");
    return 0;
}

static void __exit ads122_exit(void)
{
    i2c_del_driver(&ads122_driver);
    if (NULL != ads122_1_client)
        i2c_unregister_device(ads122_1_client);
    if (NULL != ads122_2_client)
        i2c_unregister_device(ads122_2_client);
}

module_init(ads122_init);
module_exit(ads122_exit);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com>");
MODULE_DESCRIPTION("Texas Instruments ADS122 ADC driver");
MODULE_LICENSE("GPL v2");

