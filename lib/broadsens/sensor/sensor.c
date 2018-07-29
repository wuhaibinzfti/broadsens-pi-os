#include <stdio.h>
#include <iio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

/* sensor api */
#include "bno055.h"
#include "ads122.h"

#include "sensor.h"

#define ASSERT(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

static int channel_count = 0;

static struct iio_channel *accel_x = NULL;
static struct iio_channel *accel_y = NULL;
static struct iio_channel *accel_z = NULL;
static struct iio_channel *timestamp = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_channel *rx0_i = NULL;

static struct iio_context *iio_ctx = NULL;

static inline int strain_gauge_chan_to_port(int chan)
{
    switch (chan) {
        case 3: return STRAIN_PORT2;
        case 7: return STRAIN_PORT1;
        case 16: return STRAIN_PORT_TEMP;
        default: return STRAIN_PORT_UNDEF;
    }
}

/* static scratch mem for strings */
static char tmpstr[64];
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

static bool channel_has_attr(struct iio_channel *chn, const char *attr)
{
	unsigned int i, nb = iio_channel_get_attrs_count(chn);
	for (i = 0; i < nb; i++)
		if (!strcmp(attr, iio_channel_get_attr(chn, i)))
			return true;
	return false;
}

static double get_channel_value(struct iio_channel *chn)
{
	char buf[1024];
	double val;

	if (channel_has_attr(chn, "input")) {
		iio_channel_attr_read(chn, "input", buf, sizeof(buf));
		val = strtod(buf, NULL);
	} else {
		iio_channel_attr_read(chn, "raw", buf, sizeof(buf));
		val = strtod(buf, NULL);

		if (channel_has_attr(chn, "offset")) {
			iio_channel_attr_read(chn, "offset", buf, sizeof(buf));
			val += strtod(buf, NULL);
		}

		if (channel_has_attr(chn, "scale")) {
			iio_channel_attr_read(chn, "scale", buf, sizeof(buf));
			val *= strtod(buf, NULL);
		}
	}

	return val;
}

int iio_context_init(void)
{
    struct iio_context *ctx;
    ctx = iio_create_local_context();
    if (ctx) {
        return -1;
    }
    if (iio_context_get_devices_count(ctx) <= 0) {
        return -1;
    }
    iio_ctx = ctx;
    return 0;
}

int strain_gauge_sensor_init(struct iio_context *ctx)
{
    if (ads122_sensor_init(ADS122_DEV_0, ctx) < 0) {
        return -1;
    }
    if (ads122_sensor_init(ADS122_DEV_1, ctx) < 0) {
        return -1;
    }

    ads122_set_interval(ADS122_DEV_0, ADS122_PAIN3_NAIN2, 50000);
    ads122_set_interval(ADS122_DEV_1, ADS122_PAIN3_NAIN2, 50000);
    return 0;
}

int strain_gauge_data_poll(int offset, float *data, struct timeval *tv)
{
    int chan;
    enum ads122_dev_no dev_no;

    switch (offset) {
        case 0: dev_no = ADS122_DEV_0; break;
        case 1: dev_no = ADS122_DEV_1; break;
        default: return -1;
    }

    chan = ads122_buffer_poll(dev_no, data, tv);
    return strain_gauge_chan_to_port(chan);
}

int iio_sensors_init(void)
{
    struct iio_context *ctx;

    ASSERT((ctx = iio_create_local_context()) && "No context");
    if (iio_context_get_devices_count(ctx) < 0) {
        iio_context_destroy(ctx);
        return -1;
    }

    if (iio_context_set_timeout(ctx, 0) < 0) {
        return -1;
    }

    if (bno055_sensor_init(ctx) < 0) {

    }
    if (strain_gauge_sensor_init(ctx) < 0) {
        return -1;
    }

    return 0;
}

int main(int argc, char **argv)
{
    int ret, port, offset;
    unsigned long tick = 0, pr_ctrl = 0;
    float value = 0.0;
    struct timespec time;
    struct timeval tv;
    static float port_data1[3];
    static float port_data2[3];
    float buffer[20];

    time.tv_sec = 0;
    time.tv_nsec = 1000;

    if (iio_sensors_init() < 0)
        return -1;

    while (1) {
        #if 1
        ret = bno055_buffer_poll(buffer, &tv);
        if (ret >= 0) {
            printf("%.2f  %.2f  %.2f  [%ld.%ld]\n", buffer[0], buffer[1], buffer[2],
                                                    tv.tv_sec, tv.tv_usec);
        }
        #endif

        port = strain_gauge_data_poll(0, &value, &tv);
        if (port >= 0) {
            port_data1[port] = value;
        }
        if (1 == port) {
            printf("dev[0]: %f %f %f\n", port_data1[0], port_data1[1], port_data1[2]);
        }

        port = strain_gauge_data_poll(1, &value, &tv);
        if (port >= 0) {
            port_data2[port] = value;
        }
        if (1 == port) {
            printf("dev[1]: %f %f %f\n", port_data2[0], port_data2[1], port_data2[2]);
        }

        nanosleep(&time, NULL);
    }

    return 0;
}