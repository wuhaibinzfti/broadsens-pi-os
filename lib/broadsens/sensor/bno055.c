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

#include <string.h>
#include <stdio.h>
#include <iio.h>
#include "bno055.h"
#include "common.h"

#define ASSERT(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

#define BNO055_CHNNEL_NUM 6
struct bno055_device {
    int dev_no;
    struct iio_device *indio_dev;
    int channel_count;
    struct iio_channel *channels[BNO055_CHNNEL_NUM];
    struct iio_buffer  *rxbuf;
};

static struct bno055_device bno055_dev;

static int bno055_iio_register(struct iio_context *ctx, struct bno055_device *device)
{
    int chan_nbr = 0;
    struct iio_buffer  *rxbuf = NULL;
    static struct iio_device *dev;

    dev = iio_context_find_device(ctx, "bno055");
    if (!dev) {
        PRINT_ERROR("could not find iio_dev:bno055");
        return -1;
    }

    for (int i = 0; i < iio_device_get_channels_count(dev); ++i) {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn)) {
            if (chan_nbr < BNO055_CHNNEL_NUM) {
                iio_channel_enable(chn);
                device->channels[chan_nbr++] = chn;
            } else {
                PRINT_ERROR("channel no space");
                return -1;
            }
        }
    }
    if (0 == chan_nbr) {
        PRINT_ERROR("No scan elements found");
    }

    iio_device_attr_write(dev, "interval", "1000000");

    device->rxbuf = iio_device_create_buffer(dev, 1, false);
    if (!device->rxbuf) {
        PRINT_ERROR("Could not create rx buffer");
    }

    if (iio_buffer_set_blocking_mode(device->rxbuf, false) < 0) {
        PRINT_ERROR("Could not set rx buffer no block!\n");
        return -1;
    }

    device->indio_dev = dev;
    device->channel_count = chan_nbr;
    return 0;
}

int bno055_buffer_poll(float *buffer, struct timeval *tv)
{
    char buf[512];
    int i = 0;
    ssize_t nbytes_rx;
    size_t sample_size, bytes;
    const struct iio_data_format *fmt;
    struct bno055_device *dev = &bno055_dev;

    nbytes_rx = iio_buffer_refill(dev->rxbuf);

    for (i = 0; i < dev->channel_count; i++) {
        struct iio_channel *chn = dev->channels[i];
        fmt = iio_channel_get_data_format(chn);
        sample_size = fmt->length / 8;
        bytes = iio_channel_read(chn, dev->rxbuf, buf, sample_size);
        if (sample_size != bytes) {
            return -1;
        }
        if (2 == sample_size) {
            buffer[i] = (float)(((int16_t *)buf)[0]);
            iio_channel_attr_read(chn, "scale", buf, sizeof(buf));
            buffer[i] *= strtof(buf, NULL);
        } else if (8 == sample_size) {
            tv->tv_sec = ((int64_t *)buf)[0] / 1000000000;
            tv->tv_usec = ((int64_t *)buf)[0] % 1000000 / 1000;
        }
    }

    return 0;
}

int bno055_sensor_init(struct iio_context *ctx)
{
    bno055_iio_register(ctx, &bno055_dev);
    return 0;
}
