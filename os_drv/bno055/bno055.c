/*
 * BNO055 - Bosch 9-axis orientation sensor
 *
 * Copyright (c) 2016, Intel Corporation.
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * TODO:
 *  - buffering
 *  - interrupt support
 *  - linear and gravitational acceleration (not supported in IIO)
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>

#include <asm/irq.h>

#define BNO055_DRIVER_NAME            "bno055"

#define BNO055_REG_CHIP_ID            0x00
#define BNO055_REG_PAGE_ID            0x07

#define BNO055_REG_ACC_X_LSB     0x08
#define BNO055_REG_MAG_X_LSB     0x0E
#define BNO055_REG_GYR_X_LSB     0x14
#define BNO055_REG_EUL_HEADING_LSB    0x1A
#define BNO055_REG_LIA_ACC_X_LSB 0x28
#define BNO055_REG_QUA_W_LSB     0x20
#define BNO055_REG_TEMP               0x34

#define BNO055_REG_SYS_ERR            0x3A
#define BNO055_REG_UNIT_SEL           0x3B

#define BNO055_REG_OPR_MODE           0x3D
#define BNO055_REG_AXIS_MAP_SIGN      0x42

#define BNO055_REG_ACC_OFFSET_X_LSB   0x55
#define BNO055_REG_MAG_OFFSET_X_LSB   0x5B
#define BNO055_REG_GYR_OFFSET_X_LSB   0x61
#define BNO055_REG_MAG_RADIUS_MSB     0x6A

/*
 * The difference in address between the register that contains the
 * value and the register that contains the offset.  This applies for
 * accel, gyro and magn channels.
 */
#define BNO055_REG_OFFSET_ADDR        0x4D

#define BNO055_OPR_MODE_MASK          GENMASK(3, 0)

/* Combination of BNO055 and individual chip IDs. */
#define BNO055_CHIP_ID                0x0F32FBA0

#define BNO055_ANDROID_ORIENTATION    BIT(7)
#define BNO055_TEMP_CELSIUS           0
#define BNO055_EUL_RADIANS            BIT(2)
#define BNO055_GYR_RADIANS            BIT(1)
#define BNO055_ACC_MPSS               0

#define BNO055_MIN_INTERVAL           500           /* us */

/*
 * Operation modes.  It is important that these are listed in the order
 * they appear in the datasheet, as an index to this table is used to
 * write the actual bits in the operation config register.
 */
enum bno055_operation_mode {
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
};

enum bno055_scan
{
    BNO055_SCAN_LIA_ACCL_X,
    BNO055_SCAN_LIA_ACCL_Y,
    BNO055_SCAN_LIA_ACCL_Z,
    BNO055_SCAN_TIMESTAMP,
};

/*
 * Number of channels for each operation mode.  See Table 3-3 in the
 * datasheet for a summary of each operation mode.  Each non-config mode
 * also supports a temperature channel.
 */
static const int bno055_num_channels[] = {
	0, 5, 5, 5, 8, 8, 8, 11,
	/*
	 * In fusion modes, data from the raw sensors is still
	 * available.  Additionally, the linear and gravitational
	 * components of acceleration are available in all fusion modes,
	 * but there are currently no IIO attributes for these.
	 *
	 * Orientation is exposed both as a quaternion multi-value
	 * (meaning a single channel) and as Euler angles (3 separate
	 * channels).
	 */
	15, 15, 15, 18, 18,
};

struct bno055_data {
    struct i2c_client *client;
	struct regmap *regmap;
	enum bno055_operation_mode op_mode;
     struct work_struct work;
     struct hrtimer timer;
     ktime_t ktime;
     struct mutex mlock;
};

/*
 * Note: The BNO055 has two pages of registers.  All the addresses below
 * are page 0 addresses.  If the driver ever uses page 1 registers, it
 * is expected to manually switch between pages via the PAGE ID register
 * and make sure that no other transactions happen.  It also cannot use
 * the regmap interface for accessing registers in page 1.
 */
static const struct regmap_range bno055_writable_ranges[] = {
	regmap_reg_range(BNO055_REG_ACC_OFFSET_X_LSB, BNO055_REG_MAG_RADIUS_MSB),
	regmap_reg_range(BNO055_REG_OPR_MODE, BNO055_REG_AXIS_MAP_SIGN),
	regmap_reg_range(BNO055_REG_UNIT_SEL, BNO055_REG_UNIT_SEL),
	/* Listed as read-only in the datasheet, but probably an error. */
	regmap_reg_range(BNO055_REG_PAGE_ID, BNO055_REG_PAGE_ID),
};

static const struct regmap_access_table bno055_writable_regs = {
	.yes_ranges = bno055_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(bno055_writable_ranges),
};

/* Only reserved registers are non-readable. */
static const struct regmap_range bno055_non_readable_reg_ranges[] = {
	regmap_reg_range(BNO055_REG_AXIS_MAP_SIGN + 1, BNO055_REG_ACC_OFFSET_X_LSB - 1),
};

static const struct regmap_access_table bno055_readable_regs = {
	.no_ranges = bno055_non_readable_reg_ranges,
	.n_no_ranges = ARRAY_SIZE(bno055_non_readable_reg_ranges),
};

static const struct regmap_range bno055_volatile_reg_ranges[] = {
	regmap_reg_range(BNO055_REG_ACC_X_LSB, BNO055_REG_SYS_ERR),
};

static const struct regmap_access_table bno055_volatile_regs = {
	.yes_ranges = bno055_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(bno055_volatile_reg_ranges),
};

static const struct regmap_config bno055_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BNO055_REG_MAG_RADIUS_MSB + 1,
	.cache_type = REGCACHE_RBTREE,

	.wr_table = &bno055_writable_regs,
	.rd_table = &bno055_readable_regs,
	.volatile_table = &bno055_volatile_regs,
};

/* hr timer default 1s */
static unsigned long interval = 1000000; /* unit: us */

unsigned long long diff_tv(struct timespec start, struct timespec end)
{
    return (end.tv_sec-start.tv_sec) * 1000000000 + (end.tv_nsec - start.tv_nsec);
}

enum hrtimer_restart hrtimer_callback(struct hrtimer *hr_timer)
{
    struct bno055_data *data = container_of(hr_timer, struct bno055_data, timer);

    schedule_work(&data->work);
    hrtimer_forward_now(&data->timer, data->ktime);
    return HRTIMER_RESTART;
}

static void bno055_work(struct work_struct *_work)
{
    __u8 *buf;
    int ret;
    struct bno055_data *data = container_of(_work, struct bno055_data, work);
    struct iio_dev *indio_dev = i2c_get_clientdata(data->client);
	int64_t timestamp = 0;

    buf = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);
    if (!buf)
        return;

    if (test_bit(BNO055_SCAN_LIA_ACCL_X, indio_dev->active_scan_mask) &&
        test_bit(BNO055_SCAN_LIA_ACCL_Y, indio_dev->active_scan_mask) &&
        test_bit(BNO055_SCAN_LIA_ACCL_Z, indio_dev->active_scan_mask) &&
        indio_dev->scan_timestamp) {
		timestamp = iio_get_time_ns(indio_dev);
        ret = regmap_bulk_read(data->regmap, BNO055_REG_LIA_ACC_X_LSB, buf, 6);
		printk("time : %lld\n", iio_get_time_ns(indio_dev) - timestamp);
        if (ret >= 0)
            iio_push_to_buffers_with_timestamp(indio_dev, buf, iio_get_time_ns(indio_dev));
    }

    kfree(buf);
}

static int bno055_read_simple_chan(struct iio_dev *indio_dev,
                                   struct iio_chan_spec const *chan,
                                   int *val, int *val2, long mask)
{
	struct bno055_data *data = iio_priv(indio_dev);
	__le16 raw_val;
	int ret;

    switch (mask) {
        case IIO_CHAN_INFO_RAW:
            ret = regmap_bulk_read(data->regmap, chan->address, &raw_val, 2);
            if (ret < 0)
                return ret;
            *val = (s16)le16_to_cpu(raw_val);
            *val2 = 0;
            return IIO_VAL_INT;
        case IIO_CHAN_INFO_OFFSET:
            ret = regmap_bulk_read(data->regmap,
                                   chan->address + BNO055_REG_OFFSET_ADDR,
                                   &raw_val, 2);
            if (ret < 0)
                return ret;
            *val = (s16)le16_to_cpu(raw_val);
            *val2 = 0;
            return IIO_VAL_INT;
        case IIO_CHAN_INFO_SCALE:
            *val = 1;
            switch (chan->type) {
                case IIO_ACCEL:
                    /* Table 3-17: 1 m/s^2 = 100 LSB */
                    *val2 = 100;
                    break;
                case IIO_MAGN:
                    /*
                     * Table 3-19: 1 uT = 16 LSB. But we need
                     * Gauss: 1G = 0.1 uT.
                     */
                    *val2 = 160;
                    break;
                    case IIO_ANGL_VEL:
                    /* Table 3-22: 1 Rps = 900 LSB */
                    *val2 = 900;
			       break;
                case IIO_ROT:
                    /* Table 3-28: 1 degree = 16 LSB */
                    *val2 = 16;
                    break;
                default:
                    return -EINVAL;
            }
        return IIO_VAL_FRACTIONAL;
    default:
        return -EINVAL;
    }
}

static int bno055_read_temp_chan(struct iio_dev *indio_dev, int *val)
{
	struct bno055_data *data = iio_priv(indio_dev);
	unsigned int raw_val;
	int ret;

	ret = regmap_read(data->regmap, BNO055_REG_TEMP, &raw_val);
	if (ret < 0)
		return ret;

	/*
	 * Tables 3-36 and 3-37: one byte of data, signed, 1 LSB = 1C.
	 * ABI wants milliC.
	 */
	*val = raw_val * 1000;

	return IIO_VAL_INT;
}

static int bno055_read_quaternion(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int size, int *vals, int *val_len,
				  long mask)
{
	struct bno055_data *data = iio_priv(indio_dev);
	__le16 raw_vals[4];
	int i, ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (size < 4)
			return -EINVAL;
		ret = regmap_bulk_read(data->regmap,
				       BNO055_REG_QUA_W_LSB,
				       raw_vals, sizeof(raw_vals));
		if (ret < 0)
			return ret;
		for (i = 0; i < 4; i++)
			vals[i] = (s16)le16_to_cpu(raw_vals[i]);
		*val_len = 4;
		return IIO_VAL_INT_MULTIPLE;
	case IIO_CHAN_INFO_SCALE:
		/* Table 3-31: 1 quaternion = 2^14 LSB */
		if (size < 2)
			return -EINVAL;
		vals[0] = 1;
		vals[1] = 1 << 14;
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int bno055_read_raw_multi(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int size, int *vals, int *val_len,
				 long mask)
{
	switch (chan->type) {
	case IIO_ACCEL:
	case IIO_MAGN:
	case IIO_ANGL_VEL:
		if (size < 2)
			return -EINVAL;
		*val_len = 2;
		return bno055_read_simple_chan(indio_dev, chan,
					       &vals[0], &vals[1],
					       mask);

	case IIO_TEMP:
		*val_len = 1;
		return bno055_read_temp_chan(indio_dev, &vals[0]);

	case IIO_ROT:
		/*
		 * Rotation is exposed as either a quaternion or three
		 * Euler angles.
		 */
		if (chan->channel2 == IIO_MOD_QUATERNION)
			return bno055_read_quaternion(indio_dev, chan,
						      size, vals,
						      val_len, mask);
		if (size < 2)
			return -EINVAL;
		*val_len = 2;
		return bno055_read_simple_chan(indio_dev, chan,
					       &vals[0], &vals[1],
					       mask);
	default:
		return -EINVAL;
	}
}

static int bno055_init_chip(struct iio_dev *indio_dev)
{
    struct bno055_data *data = iio_priv(indio_dev);
    struct device *dev = regmap_get_device(data->regmap);
    u8 chip_id_bytes[6];
    u32 chip_id;
    u16 sw_rev;
    int ret;

    ret = device_property_read_u32(dev, "bosch,operation-mode", &data->op_mode);
    if (ret < 0) {
        dev_info(dev, "failed to read operation mode, falling back to MODE_NDOF\n");
        data->op_mode = BNO055_MODE_NDOF;
    }

    if (data->op_mode >= BNO055_MODE_MAX) {
        dev_err(dev, "bad operation mode %d\n", data->op_mode);
        return -EINVAL;
    }

	ret = regmap_write(data->regmap, BNO055_REG_PAGE_ID, 0);
	if (ret < 0) {
        dev_err(dev, "failed to switch to register page 0\n");
        //return ret;
	}

	/*
	 * Configure units to what we care about.  Also configure
	 * Android orientation mode.  See datasheet Section 4.3.60.
	 */
	ret = regmap_write(data->regmap, BNO055_REG_UNIT_SEL,
			          BNO055_ANDROID_ORIENTATION | BNO055_TEMP_CELSIUS |
			          BNO055_EUL_RADIANS | BNO055_GYR_RADIANS |
			          BNO055_ACC_MPSS);
	if (ret < 0) {
		dev_err(dev, "failed to set measurement units\n");
		return ret;
	}

	ret = regmap_bulk_read(data->regmap, BNO055_REG_CHIP_ID,
			       chip_id_bytes, sizeof(chip_id_bytes));
	if (ret < 0) {
		dev_err(dev, "failed to read chip id\n");
		return ret;
	}

	chip_id = le32_to_cpu(*(u32 *)chip_id_bytes);
	sw_rev = le16_to_cpu(*(u16 *)&chip_id_bytes[4]);

	if (chip_id != BNO055_CHIP_ID) {
		dev_err(dev, "bad chip id; got %08x expected %08x\n",
			chip_id, BNO055_CHIP_ID);
		return -EINVAL;
	}

	dev_info(dev, "software revision id %04x\n", sw_rev);

	ret = regmap_update_bits(data->regmap, BNO055_REG_OPR_MODE,
				 BNO055_OPR_MODE_MASK, data->op_mode);
	if (ret < 0) {
		dev_err(dev, "failed to switch operating mode\n");
		return ret;
	}

	/*
	 * Table 3-6 says transition from CONFIGMODE to any other mode
	 * takes 7ms.
	 */
	udelay(10);

	return 0;
}

static bool bno055_fusion_mode(struct bno055_data *data)
{
	return data->op_mode >= BNO055_MODE_IMU;
}

static void bno055_init_simple_channels(struct iio_chan_spec *p, enum iio_chan_type type,
					                  u8 address, const char *extend_name,
					                  bool has_offset, int scan_index)
{
    int i;
    int mask = BIT(IIO_CHAN_INFO_RAW);

    /*
     * Section 3.6.5 of the datasheet explains that in fusion modes,
     * readout from the output registers is already compensated.  In
     * non-fusion modes, the output offset is exposed separately.
    */
    if (has_offset) {
        mask |= BIT(IIO_CHAN_INFO_OFFSET);
	}

    for (i = 0; i < 3; i++) {
        p[i] = (struct iio_chan_spec) {
            .type = type,
            .info_mask_separate = mask,
            .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
            .modified = 1,
            .channel2 = IIO_MOD_X + i,
            /* Each value is stored in two registers. */
            .address = address + 2 * i,
            .extend_name = extend_name,
            .scan_index = scan_index,
            .scan_type = {
                .sign = 's',
                .realbits = 16,
                .storagebits = 16,
                .shift = 0,
                .endianness = IIO_BE,
            }
        };
        if (scan_index >= 0)
            scan_index++;
    }
}

static int bno055_init_channels(struct iio_dev *indio_dev)
{
	struct bno055_data *data = iio_priv(indio_dev);
	struct iio_chan_spec *channels, *p;
	bool has_offset = !bno055_fusion_mode(data);

	channels = kmalloc(sizeof(*channels) *
			  bno055_num_channels[data->op_mode], GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	p = channels;

	/* Refer to Table 3-3 of the datasheet for operation modes. */
	if (data->op_mode == BNO055_MODE_ACC_ONLY ||
	    data->op_mode == BNO055_MODE_ACC_MAG ||
	    data->op_mode == BNO055_MODE_ACC_GYRO ||
	    data->op_mode == BNO055_MODE_AMG ||
	    /* All fusion modes use the accelerometer. */
	    data->op_mode >= BNO055_MODE_IMU) {
		bno055_init_simple_channels(p, IIO_ACCEL, BNO055_REG_ACC_X_LSB,
					               NULL, has_offset, -1);
		p += 3;
	}

	if (data->op_mode == BNO055_MODE_MAG_ONLY ||
	    data->op_mode == BNO055_MODE_ACC_MAG ||
	    data->op_mode == BNO055_MODE_MAG_GYRO ||
	    data->op_mode == BNO055_MODE_AMG ||
	    data->op_mode >= BNO055_MODE_COMPASS) {
		bno055_init_simple_channels(p, IIO_MAGN, BNO055_REG_MAG_X_LSB,
					               NULL, has_offset, -1);
		p += 3;
	}

	if (data->op_mode == BNO055_MODE_GYRO_ONLY ||
	    data->op_mode == BNO055_MODE_ACC_GYRO ||
	    data->op_mode == BNO055_MODE_MAG_GYRO ||
	    data->op_mode == BNO055_MODE_AMG ||
	    data->op_mode == BNO055_MODE_IMU ||
	    data->op_mode == BNO055_MODE_NDOF_FMC_OFF ||
	    data->op_mode == BNO055_MODE_NDOF) {
		bno055_init_simple_channels(p, IIO_ANGL_VEL, BNO055_REG_GYR_X_LSB,
					               NULL, has_offset, -1);
		p += 3;
	}

	if (bno055_fusion_mode(data)) {
		/* Euler angles. */
		bno055_init_simple_channels(p, IIO_ROT, BNO055_REG_EUL_HEADING_LSB,
					               NULL, false, -1);
		p += 3;

         /* Add linear accel channel with scan */
		bno055_init_simple_channels(p, IIO_ACCEL, BNO055_REG_LIA_ACC_X_LSB,
					               "linear", false, BNO055_SCAN_LIA_ACCL_X);
		p += 3;

		/* Add quaternion orientation channel. */
		*p = (struct iio_chan_spec) {
			.type = IIO_ROT,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
			.modified = 1,
			.channel2 = IIO_MOD_QUATERNION,
			.scan_index = -1,
		};
		p++;
	}

     /* Add timestamp channel */
     *p = (struct iio_chan_spec) IIO_CHAN_SOFT_TIMESTAMP(4);
     p++;

	/* Finally, all modes have a temperature channel. */
	*p = (struct iio_chan_spec) {
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_index = -1,
	};

	indio_dev->channels = channels;
	indio_dev->num_channels = bno055_num_channels[data->op_mode];

	return 0;
}

static ssize_t bno055_show_interval(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
    return sprintf(buf, "%lu\n", interval);
}

static ssize_t bno055_store_interval(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf,
                                      size_t len)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct bno055_data *data = iio_priv(indio_dev);

    int ret;

    ret = kstrtoul(buf, 10, &interval);
    if (ret)
        return ret;

    if (interval < BNO055_MIN_INTERVAL)
        return -EINVAL;

    //mutex_lock(&data->mlock);
    data->ktime = ktime_set(interval / 1000000, (interval % 1000000) * 1000);
    //mutex_unlock(&data->mlock);

    return len;
}

static IIO_DEVICE_ATTR( interval, S_IRUGO | S_IWUSR,
                        bno055_show_interval,
                        bno055_store_interval,
                        0);

static struct attribute *ad5933_attributes[] = {
        &iio_dev_attr_interval.dev_attr.attr,
        NULL
};

static const struct attribute_group bno055_attribute_group = {
        .attrs = ad5933_attributes,
};

static const struct iio_info bno055_info = {
	.driver_module = THIS_MODULE,
	.read_raw_multi = &bno055_read_raw_multi,
	.attrs = &bno055_attribute_group,
};

static int bno055_ring_preenable(struct iio_dev *indio_dev)
{
    	if (bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength))
            return -EINVAL;
        printk("bno055 ring preenable\n");

    return 0;
}

static int bno055_ring_postenable(struct iio_dev *indio_dev)
{
    struct bno055_data *data = iio_priv(indio_dev);
    hrtimer_start(&data->timer, data->ktime, HRTIMER_MODE_REL);
    printk("bno055 ring enable\n");
    return 0;
}

static int bno055_ring_postdisable(struct iio_dev *indio_dev)
{
    struct bno055_data *data = iio_priv(indio_dev);
    int ret;

    cancel_work_sync(&data->work);

    ret = hrtimer_cancel(&data->timer);
    if (ret)
        printk("The timer was still in use...\n");

    printk("hr Timer module uninstalling\n");

    return 0;
}

static const struct iio_buffer_setup_ops bno055_ring_setup_ops = {
    .preenable = &bno055_ring_preenable,
    .postenable = &bno055_ring_postenable,
    .postdisable = &bno055_ring_postdisable,
};

static int bno055_register_ring(struct iio_dev *indio_dev)
{
    struct iio_buffer *ring;

    ring = iio_kfifo_allocate();
    if (!ring)
        return -ENOMEM;

    iio_device_attach_buffer(indio_dev, ring);
    indio_dev->setup_ops = &bno055_ring_setup_ops;

    return 0;
}

static int bno055_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct iio_dev *indio_dev;
    struct bno055_data *data;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);

    data->regmap = devm_regmap_init_i2c(client, &bno055_regmap_config);
    if (IS_ERR(data->regmap))
        return PTR_ERR(data->regmap);

    indio_dev->dev.parent = &client->dev;
    indio_dev->name = BNO055_DRIVER_NAME;

    ret = bno055_init_chip(indio_dev);
    if (ret)
        return ret;

    ret = bno055_init_channels(indio_dev);
    if (ret)
        return ret;

    ret = bno055_register_ring(indio_dev);
    if (ret)
    return ret;

    indio_dev->info = &bno055_info;
    indio_dev->modes = INDIO_BUFFER_SOFTWARE | INDIO_DIRECT_MODE;
    i2c_set_clientdata(client, indio_dev);
    data->client = client;

    ret = devm_iio_device_register(&client->dev, indio_dev);
    if (ret < 0) {
        dev_err(&client->dev, "could not register iio device\n");
        return ret;
    }

    INIT_WORK(&data->work, bno055_work);
    hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    data->timer.function = hrtimer_callback;
    data->ktime = ktime_set(interval / 1000000, (interval % 1000000) * 1000);

    return 0;
}

static const struct of_device_id i2c_of_match[] = {
    { .compatible = "bosch,bno055" },
    {}
};
MODULE_DEVICE_TABLE(of, i2c_of_match);

static const struct i2c_device_id bno055_id[] = {
	{"bno055", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bno055_id);

static struct i2c_driver bno055_driver = {
	.driver = {
		.name	= "bno055",
         .of_match_table = i2c_of_match,
         .owner	= THIS_MODULE,
	},
	.probe		= bno055_probe,
	.id_table	= bno055_id,
};

module_i2c_driver(bno055_driver);

MODULE_AUTHOR("wuhaibin");
MODULE_DESCRIPTION("Driver for Bosch BNO055 9-axis orientation sensor");
MODULE_LICENSE("GPL v2");
