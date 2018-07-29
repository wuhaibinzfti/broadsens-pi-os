
#ifndef __BNO055_H__
#define __BNO055_H__

int bno055_sensor_init(struct iio_context *ctx);
int bno055_buffer_poll(float *buffer, struct timeval *tv);

#endif  /* BNO055_H */