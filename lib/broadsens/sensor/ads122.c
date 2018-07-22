
#include <string.h>
#include <stdio.h>
#include <iio.h>
#include "ads122.h"

#define ADS122_CHNNEL_NUM 4

static struct iio_device *ads122_dev;
static int channel_count;
static struct iio_channel *channels[ADS122_CHNNEL_NUM];
static struct iio_buffer  *rxbuf = NULL;

static bool channel_has_attr(struct iio_channel *chn, const char *attr)
{
	unsigned int i, nb = iio_channel_get_attrs_count(chn);
	for (i = 0; i < nb; i++)
		if (!strcmp(attr, iio_channel_get_attr(chn, i)))
			return true;
	return false;
}

int ads122_read_channel(int chn, float *buffer, struct timeval *tv)
{
    char buf[512];
    int i = 0;
    ssize_t nbytes_rx;
    size_t sample_size, bytes;
    const struct iio_data_format *fmt;
    float val = 0.0;

    nbytes_rx = iio_buffer_refill(rxbuf);
    if (nbytes_rx <= 0) {
        return -1;
    }

    for (i = 0; i < channel_count - 1; i++) {
        fmt = iio_channel_get_data_format(channels[i]);
        sample_size = fmt->length / 8;
        bytes = iio_channel_read(channels[i], rxbuf, buf, sample_size);
        if (bytes < 0) {
            return -1;
        }

        if (2 == sample_size) {
            val = (float)((int16_t *)buf)[0];
        } else if (4 == sample_size) {
            val = (float)(((int32_t *)buf)[0]);
        }

        if (channel_has_attr(channels[i], "scale")) {
            iio_channel_attr_read(channels[i], "scale", buf, sizeof(buf));
            val *= strtof(buf, NULL);
        }
        buffer[i] = val;
    }

    /* timestamp */
    fmt = iio_channel_get_data_format(channels[i]);
    sample_size = fmt->length / 8;
    bytes = iio_channel_read(channels[i], rxbuf, buf, sample_size);
    if (sample_size != bytes) {
        return -1;
    }
    tv->tv_sec = ((int64_t *)buf)[0] / 1000000000;
    tv->tv_usec = ((int64_t *)buf)[0] % 1000000000 / 1000;

    return 0;
}

int ads122_sensor_init(struct iio_context *ctx)
{
    static struct iio_device *dev;
    const char *name;

    dev = iio_context_find_device(ctx, "ads122-0");
    if (!dev) {
        printf("could not find iio_dev:ads122\n");
        return -1;
    }

    for (int i = 0; i < iio_device_get_channels_count(dev); i++) {
        struct iio_channel *chn = iio_device_get_channel(dev, i);
        if (iio_channel_is_scan_element(chn)) {
            name = iio_channel_get_id(chn);
            if (!strcmp("voltage1-voltage0", name) || !strcmp("voltage3-voltage2", name) ||
                !strcmp("temp0", name) || !strcmp("timestamp", name)) {
                if (channel_count < ADS122_CHNNEL_NUM) {
                    iio_channel_enable(chn);
                    channels[channel_count] = chn;
                    channel_count++;
                } else {
                    printf("channel space too small!\n");
                }
            }
        }
    }
    if (0 == channel_count) {
        printf("No scan elements found \n");
        return -1;
    }

    rxbuf = iio_device_create_buffer(dev, 1, false);
    if (!rxbuf) {
        printf("Could not create rx buffer\n");
    }

    iio_buffer_set_blocking_mode(rxbuf, false);
    ads122_dev = dev;
    return 0;
}

