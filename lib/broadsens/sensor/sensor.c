#include <stdio.h>
#include <iio.h>
#include <string.h>
#include "bno055.h"

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

int sensor_init(void)
{
    struct iio_context *ctx;
    struct iio_device *dev;
    double ch_val;
    char *p_dat, *p_end;
    ptrdiff_t p_inc;
    char buf[512];
    const struct iio_data_format *fmt;
    size_t sample_size;
    size_t bytes;
    ssize_t nbytes_rx;
    int sample;
    static int buffer_length = 4;

    ASSERT((ctx = iio_create_local_context()) && "No context");
    ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

    bno055_sensor_init(ctx);

    return 0;
}

int main(int argc, char **argv)
{
    struct timeval tv;
    float buffer[10];
    int ret;

    sensor_init();

    while (1) {
        ret = read_bno055_sensor_buffer(buffer, &tv);
        if (ret >= 0) {
            printf("%.2f  %.2f  %.2f  [%ld.%ld]\n", buffer[0], buffer[1], buffer[2],
                                                  tv.tv_sec, tv.tv_usec);
        }
    }
    return 0;
}