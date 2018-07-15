
#ifndef __BNO055_H__
#define __BNO055_H__

int bno055_sensor_init(struct iio_context *ctx);
int read_bno055_sensor_buffer(float *buffer, struct timeval *timestamp);

#endif  /* BNO055_H */