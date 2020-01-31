#ifndef SENSOR_H_
#define SENSOR_H_

#include "main.h"
#include "eth.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "stdio.h"
#include "bmp280.h"
#include "bmp280_defs.h"

int8_t spi_reg_read ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t length );
int8_t spi_reg_write ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t length );

#endif
