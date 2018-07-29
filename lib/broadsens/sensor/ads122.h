#ifndef __ADS122_H__
#define __ADS122_H__

enum ads122_dev_no {
    ADS122_DEV_0 = 0,
    ADS122_DEV_1,
    ADS122_DEV_NUM,
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

int ads122_sensor_init(enum ads122_dev_no dev_no, struct iio_context *ctx);
int ads122_buffer_poll(enum ads122_dev_no dev_no, float *value, struct timeval *tv);
int ads122_set_interval(enum ads122_dev_no dev_no, enum ads122_channels chn, unsigned long tick);

#endif  /* BNO055_H */