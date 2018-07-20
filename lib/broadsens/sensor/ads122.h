#ifndef __ADS122_H__
#define __ADS122_H__

int ads122_sensor_init(struct iio_context *ctx);
int ads122_read_channel(int chn, float *value, struct timeval *tv);

#endif  /* BNO055_H */