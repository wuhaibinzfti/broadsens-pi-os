
#include <string.h>
#include <stdio.h>
#include <iio.h>
#include "ads122.h"

#define ADS122_CHNNEL_NUM 4

struct ads122_device {
    int dev_no;
    struct iio_device *indio_dev;
    int channel_count;
    struct iio_channel *channels[ADS122_CHNNEL_NUM];
    struct iio_buffer  *rxbuf;
};

static struct ads122_device ads122_device[ADS122_DEV_NUM];

int ads122_buffer_minitor(int *channel, float *value, struct timeval *tv)
{
    char buf[512];
    int i = 0;
    ssize_t nbytes_rx;
    size_t sample_size, bytes;
    const struct iio_data_format *fmt;
    uint32_t tmp;
#if 0
    nbytes_rx = iio_buffer_refill(rxbuf);
    if (nbytes_rx <= 0) {
        return -1;
    }

    for (i = 0; i < channel_count; i++) {
        fmt = iio_channel_get_data_format(channels[i]);
        sample_size = fmt->length / 8;
        bytes = iio_channel_read_raw(channels[i], rxbuf, buf, sample_size);
        if (sample_size != bytes) {
            return -1;
        }
        if (4 == sample_size) {
            tmp = *((uint32_t *)buf);
            *channel = (int)(tmp >> 24);
            if (*channel >= ADS122_CHANNEL_NUM) {
                continue;
            }
            if (ADS122_TEMP == *channel) {
                *value = (float)(((int32_t)(tmp << 8)) >> 18) / 32;
            } else {
                *value = (float)(((int32_t)(tmp << 8)) >> 8) / 4096;
            }
        } else if (8 == sample_size) {
            tv->tv_sec = ((int64_t *)buf)[0] / 1000000000;
            tv->tv_usec = ((int64_t *)buf)[0] % 1000000000 / 1000;
        }
    }
#endif
    return 0;
}

static int ads122_iio_register(struct iio_context *ctx, struct ads122_device *device)
{
    int chan = 0;
    char buf[20];
    const char *name;
    struct iio_device *dev;
    struct iio_buffer *rxbuf;

    sprintf(buf, "ads122-%d", device->dev_no);

    dev = iio_context_find_device(ctx, buf);
    if (!dev) {
        printf("could not find iio_dev:ads122\n");
        return -1;
    }

    for (int i = 0; i < iio_device_get_channels_count(dev); i++) {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn)) {
            name = iio_channel_get_id(chn);
            if (!strcmp("voltage", name) || !strcmp("timestamp", name)) {
                if (chan < ADS122_CHNNEL_NUM) {
                    iio_channel_enable(chn);
                    device->channels[chan++] = chn;
                    printf("%s\n", name);
                } else {
                    printf("channel space too small!\n");
                }
            }
        }
    }

    if (0 == chan) {
        printf("No scan elements found \n");
        return -1;
    }

    device->rxbuf = iio_device_create_buffer(dev, 1, false);
    if (!device->rxbuf) {
        printf("Could not create rx buffer\n");
    }

    iio_buffer_set_blocking_mode(device->rxbuf, false);

    device->indio_dev = dev;
    device->channel_count = chan;

    return 0;
}

int ads122_sensor_init(enum ads122_dev_no dev_no, struct iio_context *ctx)
{
    struct ads122_device *device = &ads122_device[dev_no];
    device->dev_no = dev_no;
    ads122_iio_register(ctx, device);
    return 0;
}

