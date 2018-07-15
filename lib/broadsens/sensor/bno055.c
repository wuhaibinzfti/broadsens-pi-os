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

#define ASSERT(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

static struct iio_device *bno055_dev;
static struct iio_buffer  *rxbuf = NULL;
static int channel_count;
static struct iio_channel **channels;

int read_bno055_sensor_buffer(float *buffer, struct timeval *tv)
{
    char buf[512];
    int i = 0;
    ssize_t nbytes_rx;
    size_t sample_size, bytes;
    const struct iio_data_format *fmt;

    nbytes_rx = iio_buffer_refill(rxbuf);

    for (i = 0; i < channel_count - 1; i++) {
        fmt = iio_channel_get_data_format(channels[i]);
        sample_size = fmt->length / 8;
        bytes = iio_channel_read(channels[i], rxbuf, buf, sample_size);
        if (sample_size == bytes) {
            buffer[i] = (float)(((int16_t *)buf)[0]);
            iio_channel_attr_read(channels[i], "scale", buf, sizeof(buf));
            buffer[i] *= strtof(buf, NULL);
        }
    }

    /* timestamp */
    fmt = iio_channel_get_data_format(channels[i]);
    sample_size = fmt->length / 8;
    bytes = iio_channel_read(channels[i], rxbuf, buf, sample_size);
    if (sample_size == bytes) {
        tv->tv_sec = ((int64_t *)buf)[0] / 1000000000;
        tv->tv_usec = ((int64_t *)buf)[0] % 1000000 / 1000;

        return 0;
    }

    return -1;
}

int bno055_sensor_init(struct iio_context *ctx)
{
    static struct iio_device *dev;

    ASSERT((dev = iio_context_find_device(ctx, "bno055")) && "No bno055");

    for (int i = 0; i < iio_device_get_channels_count(dev); ++i) {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn)) {
            printf("%s\n", iio_channel_get_id(chn));
            channel_count++;
        }
    }
    if (0 == channel_count) {
        printf("No scan elements found \n");
    }

    channels = (iio_channel **)calloc(channel_count, sizeof *channels);
    if (!channels) {
        printf("Channel array allocation failed");
    }
    for (int i = 0; i < channel_count; ++i) {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn)) {
            channels[i] = chn;
        }
    }

    printf("Enabling iio streaming channels for buffered capture\n");
    for (int i = 0; i < channel_count; ++i) {
        iio_channel_enable(channels[i]);
    }

    rxbuf = iio_device_create_buffer(dev, 4, false);
    if (!rxbuf) {
        printf("Could not create rx buffer");
    }

    bno055_dev = dev;
    return 0;
}
