#include <string.h>
#include <stdio.h>
#include <iio.h>
#include "ads122.h"
#include "common.h"

#define ADS122_CHNNEL_NUM 4
#define ADS122_IIO_DEVICE_NAME "ads122"

struct ads122_device {
    int dev_no;
    struct iio_device *indio_dev;
    int channel_count;
    struct iio_channel *channels[ADS122_CHNNEL_NUM];
    struct iio_buffer  *rxbuf;
};

static struct ads122_device ads122_device[ADS122_DEV_NUM];

static void ads122_iio_shutdown(struct iio_context *ctx, struct ads122_device *device)
{
    if (device->rxbuf) {
        iio_buffer_destroy(device->rxbuf);
    }

    for (int i = 0; i < ADS122_CHNNEL_NUM; i++) {
        if (device->channels[i]) {
            iio_channel_disable(device->channels[i]);
        }
    }
    return;
}

static int ads122_iio_register(struct iio_context *ctx, struct ads122_device *device)
{
    int chan = 0;
    char buf[20];
    struct iio_device *dev;
    struct iio_buffer *rxbuf;

    if (device->dev_no < 0) {
        sprintf(buf, "%s", ADS122_IIO_DEVICE_NAME);
    } else {
        sprintf(buf, ""ADS122_IIO_DEVICE_NAME"-%d", device->dev_no);
    }

    dev = iio_context_find_device(ctx, buf);
    if (!dev) {
        PRINT_ERROR("could not find iio_dev:ads122");
        return -1;
    }

    for (int i = 0; i < iio_device_get_channels_count(dev); i++) {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn)) {
            if (chan < ADS122_CHNNEL_NUM) {
                iio_channel_enable(chn);
                device->channels[chan++] = chn;
            } else {
                PRINT_ERROR("channel no space");
                ads122_iio_shutdown(ctx, device);
                return -1;
            }
        }
    }

    if (0 == chan) {
        PRINT_ERROR("No scan elements found");
        ads122_iio_shutdown(ctx, device);
        return -1;
    }

    device->rxbuf = iio_device_create_buffer(dev, 1, false);
    if (!device->rxbuf) {
        PRINT_ERROR("Could not create rx buffer\n");
        ads122_iio_shutdown(ctx, device);
        return -1;
    }

    if (iio_buffer_set_blocking_mode(device->rxbuf, false) < 0) {
        PRINT_ERROR("Could not set rx buffer no block!\n");
        ads122_iio_shutdown(ctx, device);
        return -1;
    }

    device->indio_dev = dev;
    device->channel_count = chan;

    return 0;
}

int ads122_buffer_poll(enum ads122_dev_no dev_no, float *value, struct timeval *tv)
{
    char buf[128];
    int chn_no;
    uint32_t tmp;
    const struct iio_data_format *fmt;
    struct ads122_device *dev = &ads122_device[dev_no];

    if (iio_buffer_refill(dev->rxbuf) <= 0) return -1;

    for (int i = 0; i < dev->channel_count; i++) {
        struct iio_channel *chn = dev->channels[i];
        fmt = iio_channel_get_data_format(chn);
        size_t sample_size = fmt->length / 8;
        size_t bytes = iio_channel_read_raw(chn, dev->rxbuf, buf, sample_size);
        if (sample_size != bytes) {
            return -1;
        }
        if (4 == sample_size) {
            tmp = *((uint32_t *)buf);
            chn_no = (int)(tmp >> 24);
            if (chn_no >= ADS122_CHANNEL_NUM) {
                return -1;
            }
            if (ADS122_TEMP == chn_no) {
                *value = (float)(((int32_t)(tmp << 8)) >> 18) / 32;
            } else {
                *value = (float)(((int32_t)(tmp << 8)) >> 8) / 4096; /* raw * 2048 / (2 ^ 23) */
            }
        } else if (8 == sample_size) {
            tv->tv_sec = ((int64_t *)buf)[0] / 1000000000;
            tv->tv_usec = ((int64_t *)buf)[0] % 1000000000 / 1000;
        }
    }

    return chn_no;
}

int ads122_set_interval(enum ads122_dev_no dev_no, enum ads122_channels chn,
                        unsigned long tick)
{
    char buf[20];
    struct iio_device *dev = ads122_device[dev_no].indio_dev;

    sprintf(buf, "%ld", tick);

    if (iio_device_attr_write(dev, "tick", buf) < 0) {
        return -1;
    }

    return 0;
}

int ads122_sensor_init(enum ads122_dev_no dev_no, struct iio_context *ctx)
{
    int ret;
    struct ads122_device *device = &ads122_device[dev_no];
    device->dev_no = dev_no;

    ret = ads122_iio_register(ctx, device);
    if (ret < 0) {
        return -1;
    }
    return 0;
}

