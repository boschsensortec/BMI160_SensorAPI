# BMI160 sensor API
## Introduction
This package contains the Bosch Sensortec's BMI160 sensor driver (sensor API)

The sensor driver package includes bmi160.h, bmi160.c and bmi160_defs.h files

## Version
File | Version | Date
-----|---------|-----
bmi160.c |   3.0.0    |   Dec 1, 2016
bmi160.h |   3.0.0   |    Dec 1, 2016
bmi160_defs.h |   3.0.0    |   Dec 1, 2016

## Integration details
* Integrate bmi160.h, bmi160_defs.h and bmi160.c file in to your project.
* Include the bmi160.h file in your code like below.
``` c
#include "bmi160.h"
```

## File information
* bmi160_defs.h : This header file has the constants, macros and datatype declarations.
* bmi160.h : This header file contains the declarations of the sensor driver APIs.
* bmi160.c : This source file contains the definitions of the sensor driver APIs.

## Supported sensor interface
* SPI 4-wire
* I2C

## Usage guide
### Initializing the sensor
To initialize the sensor, you will first need to create a device structure. You 
can do this by creating an instance of the structure bmi160_dev. Then go on to 
fill in the various parameters as shown below.

#### Example for SPI 4-Wire
``` c
struct bmi160_dev sensor;

/* You may assign a chip select identifier to be handled later */
sensor.id = 0;
sensor.interface = BMI160_SPI_INTF;
sensor.read = user_spi_read;
sensor.write = user_spi_write;
sensor.delay_ms = user_delay_ms;


int8_t rslt = BMI160_OK;
rslt = bmi160_init(&sensor);
/* After the above function call, accel_cfg and gyro_cfg parameters in the device 
structure are set with default values, found in the datasheet of the sensor */
```

#### Example for I2C
``` c
struct bmi160_dev sensor;

sensor.id = BMI160_I2C_ADDR;
sensor.interface = BMI160_I2C_INTF;
sensor.read = user_i2c_read;
sensor.write = user_i2c_write;
sensor.delay_ms = user_delay_ms;

int8_t rslt = BMI160_OK;
rslt = bmi160_init(&sensor);
/* After the above function call, accel and gyro parameters in the device structure 
are set with default values, found in the datasheet of the sensor */
```

### Configuring accel and gyro sensor
#### Example for configuring accel and gyro sensors in normal mode
``` c

int8_t rslt = BMI160_OK;

/* Select the Output data rate, range of accelerometer sensor */
sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

/* Select the power mode of accelerometer sensor */
sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

/* Select the Output data rate, range of Gyroscope sensor */
sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

/* Select the power mode of Gyroscope sensor */
sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

/* Set the sensor configuration */
rslt = bmi160_set_sens_conf(&sensor);
```

### Reading sensor data
#### Example for reading sensor data
``` c

int8_t rslt = BMI160_OK;
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;

/* To read only Accel data */
rslt = bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &accel, NULL, &sensor);

/* To read only Gyro data */
rslt = bmi160_get_sensor_data(BMI160_GYRO_ONLY, NULL, &gyro, &sensor);

/* To read both Accel and Gyro data */
bmi160_get_sensor_data(BMI160_BOTH_ACCEL_AND_GYRO, &accel, &gyro, &sensor);
```

### Setting the power mode of sensors
#### Example for setting power mode of accel and gyro
``` c

int8_t rslt = BMI160_OK;

/* Select the power mode */
sensor.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE; 
sensor.gyro_cfg.power = BMI160_GYRO_FASTSTARTUP_MODE; 

/*  Set the Power mode  */
rslt = bmi160_set_power_mode(&sensor);

/* Select the power mode */
sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

/*  Set the Power mode  */
rslt = bmi160_set_power_mode(&sensor);

```

### Reading sensor data register
#### Example for reading Chip Address
``` c

int8_t rslt = BMI160_OK;
uint8_t reg_addr = BMI160_CHIP_ID_ADDR;
uint8_t data;
uint16_t len = 1;
rslt = bmi160_get_regs(reg_addr, &data, len, &sensor);
```


### Writing to sensor data register
#### Example for writing data to any motion threshold register
``` c

int8_t rslt = BMI160_OK;
uint8_t reg_addr = BMI160_INT_MOTION_1_ADDR;
uint8_t data = 20;
uint16_t len = 1;
rslt = bmi160_set_regs(reg_addr, &data, len, &sensor);
```

### Resetting the device using soft-reset
#### Example for writing soft-reset command to command register
``` c

int8_t rslt = BMI160_OK;
rslt = bmi160_soft_reset(&sensor);
```


### Configuring interrupts for sensors
To configure the sensor interrupts, you will first need to create an interrupt 
structure. You can do this by creating an instance of the structure bmi160_intr_sett.
Then go on to fill in the various parameters as shown below


### Configuring Slope/Any-motion Interrupt
#### Example for configuring Slope/Any-motion Interrupt
``` c

struct bmi160_intr_sett int_config;

/* Select the Interrupt channel/pin */
int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

/* Select the Interrupt type */
int_config.int_type = BMI160_ACC_SLOPE_INT;// Choosing Slope/Any motion interrupt

/* Select the interrupt channel/pin settings */
int_config.int_pin_sett.output_en = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
int_config.int_pin_sett.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
int_config.int_pin_sett.output_type = BMI160_DISABLE;// Choosing active low output
int_config.int_pin_sett.edge_ctrl = BMI160_ENABLE;// Choosing edge triggered output
int_config.int_pin_sett.input_en = BMI160_DISABLE;// Disabling interrupt pin to act as input
int_config.int_pin_sett.latch_dur = BMI160_LATCH_DUR_NONE;// non-latched output

/* Select the Slope/Any-motion interrupt parameters */
int_config.int_type_cfg.acc_slope_int.slope_x = BMI160_ENABLE;// Enabling x-axis for slope/any motion interrupt
int_config.int_type_cfg.acc_slope_int.slope_y = BMI160_ENABLE;// Enabling y-axis for slope/any motion interrupt
int_config.int_type_cfg.acc_slope_int.slope_z = BMI160_ENABLE;// Enabling z-axis for slope/any motion interrupt
int_config.int_type_cfg.acc_slope_int.slope_dur = 0;// Slope duration
int_config.int_type_cfg.acc_slope_int.slope_thr = 20;// (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg, (8-g range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg 

/* Set the Slope/Any-motion interrupt */
bmi160_set_intr_config(&int_config, &sensor); /* sensor is an instance of the structure bmi160_dev  */

```
### Configuring Flat Interrupt
#### Example for configuring Flat Interrupt
``` c

struct bmi160_intr_sett int_config;

/* Select the Interrupt channel/pin */
int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

/* Select the Interrupt type */
int_config.int_type = BMI160_ACC_FLAT_INT;// Choosing flat interrupt

/* Select the interrupt channel/pin settings */
int_config.int_pin_sett.output_en = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
int_config.int_pin_sett.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
int_config.int_pin_sett.output_type = BMI160_DISABLE;// Choosing active low output
int_config.int_pin_sett.edge_ctrl = BMI160_ENABLE;// Choosing edge triggered output
int_config.int_pin_sett.input_en = BMI160_DISABLE;// Disabling interrupt pin to act as input
int_config.int_pin_sett.latch_dur = BMI160_LATCH_DUR_NONE;// non-latched output

/* Select the Flat interrupt parameters */
int_config.int_type_cfg.acc_flat_int.flat_theta = 8;// threshold for detection of flat position in range from 0° to 44.8°.
int_config.int_type_cfg.acc_flat_int.flat_hy = 1;// Flat hysteresis
int_config.int_type_cfg.acc_flat_int.flat_hold_time = 1;// Flat hold time (0 -> 0 ms, 1 -> 640 ms, 2 -> 1280 ms, 3 -> 2560 ms)

/* Set the Flat interrupt */
bmi160_set_intr_config(&int_config, &sensor); /* sensor is an instance of the structure bmi160_dev */

```
## Copyright (C) 2015 - 2016 Bosch Sensortec GmbH