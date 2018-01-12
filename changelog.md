# Change Log
All notable changes to bmi160 Sensor API will be documented in this file.

## v3.7.5, 11 Jan 2018
#### Added
	- "bmi160_get_power_mode" API added
#### Changed
	- bmi160_init function github comment fixed


## v3.7.4, 24 Nov 2017
#### Added
	- Linux compatibility issue fixed
	- FIFO Sensortime access is updated
	- Try concept used in bmi160_init API

## v3.7.3, 20 Nov 2017
#### Added
	- Provided support for FIFO tagging feature by adding 
	  "BMI160_FIFO_TAG_INT_PIN" case to "bmi160_set_int_config" API

## v3.7.2, 16 Oct 2017
#### Added
	- Aux FIFO support added
	- Self-test issue fixed

## v3.7.1, 10 Oct 2017
#### Added
	- Support for mapping and unmapping interrupt pin for channel 1, 2 and both
	
## v3.7.0, 05 Oct 2017
#### Added
	- Support for reading interrupt status
	- Support for mapping and unmapping interrupt pin for channel 1 & 2
	
## v3.6.1, 23 Aug 2017
#### Changed
* SPI support for FIFO reading and parsing logic update

## v3.6.0, 04 Aug 2017
#### Added
* Added interfaces for the following features 
     - FOC
     - Manual Offset compensation
     - Offset compenation value update to NVM

## v3.5.0, 13 Apr 2017

#### Added
* Self-test feature for accel and gyro added

## v3.4.0, 31 Mar 2017

#### Added
* Auxiliary sensor interface in auto-mode(data-mode) support is implemented

## v3.3.0, 31 Mar 2017

#### Added
* Extracting of gyro data from FIFO is implemented.

## v3.2.1, 15 Mar 2017

#### Changed
* Aux init made compatible for all auxillary sensors.

## v3.2.0, 09 Mar 2017

#### Added
* Reading FIFO data and extracting of accel data from FIFO implemented.
* FIFO FULL Interrupt implemented only for Accel data.

## v3.1.0, 15 Feb 2017

#### Changed
* Condition for gyro BW corrected in order to set for all BW modes.
* Interrupt Active High level setting handled.
* Existing step detector code corrected for recommended settings.
* Disabling of step detector and step counter has been removed in low-g interrupt.

#### Added
* Error code implemented, if input parameter is out of range or invalid.
* Source of data (filter & prefilter) setting handled for slope, no-motion, tap, sig, high-g & Low-g interrupt.
* Error handling of Pre-filter data & Interrupt is done in low power mode. 
* Burst write handled for low & suspended power mode.
* Auxiliary read & write implemented in order to read the BMM150 data.
* Interrupt disable mechanism added.


