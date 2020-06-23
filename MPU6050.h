/**
  ******************************************************************************
  * @file           : MPU6050.h
  * @brief          : Header for MPU6050.c file.
  *                   This file contains the common defines to use the MPU6050.
  * @author			: Lawrence Stanton
  * @revised		: May 2020
  ******************************************************************************
  * @attention		© University of Cape Town 2019
  *
  * This file and its associates are under license. Please refer to the accompanying
  * LICENCE file for details. Use outside of the license is prohibited except for
  * exclusive use by the University of Cape Town or the SHARC Buoy Research Project.
  *
  ******************************************************************************
  */
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f0xx_hal.h"	// Use the HAL methods and data types


/* Begin public typedefs. */

/*
 * @brief	Valid I2C Address values for the MPU6050
 */
typedef enum{
	/* Two possible I2C addresses for the MPU6050 depending on pin AD0 */
	MPU6050_I2C_ADDR_GND_AD0 = 0b1101000U,
	MPU6050_I2C_ADDR_VCC_AD0 = 0b1101001U
} MPU6050_AddressTypedef;

/*
 * @bief	Reset bits for each of the three sensors or all.
 * @note	To be used in resetMPU6050SignalPaths
 */
typedef enum{
	/* Defined path reset parameters. Ref. Register Map Sec. 4.26. */
	MPU6050_PATH_RESET_GYROSCOPE		= 0x04U,
	MPU6050_PATH_RESET_ACCELEROMETER	= 0x02U,
	MPU6050_PATH_RESET_TEMPERATURE		= 0x01U,
	MPU6050_PATH_RESET_ALL				= 0x07U
} MPU6050_PathResetTypedef;

/*
 * @brief	Clock Source Selection bits.
 * @note	To be used in setMPU6050ClockSource.
 * @note	The output of a gyroscope being used as the clock source is undefined.
 */
typedef enum{
	/* Defined values for clock sources. Ref. Register Map Sec. 4.28 */
	MPU6050_CLOCK_8MHz_HSI		= 0U,
	MPU6050_CLOCK_X_GYRO		= 1U,
	MPU6050_CLOCK_Y_GYRO		= 2U,
	MPU6050_CLOCK_Z_GYRO		= 3U,
	/* External clocks banned. Can be uncommented if these are made available */
	//MPU6050_CLOCK_32_768kHz_EXT = 4UU,
	//MPU6050_CLOCK_19_2MHz_EXT	= 5U,
	MPU6050_CLOCK_STOP			= 7U	// Passing this value into setMPU6050ClockSource will not technically set a source per say but is allowed.
} MMPU6050_ClockSourceTypedef;

/*
 * @brief 	Digital Low Pass filter configuration bits.
 * @note	To be used in setMPU6050DLPF.
 */
typedef enum{
	/* Possible Digital Low Pass Filter Values */
	MPU6050_DLPF_A260_G256_Hz		= 0x00U,	// Bandwidth (Accelerometer, Gyroscope) = (260 , 256)Hz. Delay (Accelerometer, Gyroscope) = (0,0  ,  0,0)ms
	MPU6050_DLPF_A184_G188_Hz		= 0x01U,	// Bandwidth (Accelerometer, Gyroscope) = (184 , 188)Hz. Delay (Accelerometer, Gyroscope) = (2.0  ,  1.9)ms
	MPU6050_DLPF_A94_G98_Hz			= 0x02U,	// Bandwidth (Accelerometer, Gyroscope) = (94  ,  98)Hz. Delay (Accelerometer, Gyroscope) = (3.0  ,  2.8)ms
	MPU6050_DLPF_A44_G42_Hz			= 0x03U,	// Bandwidth (Accelerometer, Gyroscope) = (44  ,  42)Hz. Delay (Accelerometer, Gyroscope) = (4.9  ,  4.8)ms
	MPU6050_DLPF_A21_G20_Hz			= 0x04U,	// Bandwidth (Accelerometer, Gyroscope) = (21  ,  20)Hz. Delay (Accelerometer, Gyroscope) = (8.5  ,  8.3)ms
	MPU6050_DLPF_A10_G10_Hz			= 0x05U,	// Bandwidth (Accelerometer, Gyroscope) = (10  ,  10)Hz. Delay (Accelerometer, Gyroscope) = (13.8 , 13.4)ms
	MPU6050_DLPF_A5_G5_Hz			= 0x06U		// Bandwidth (Accelerometer, Gyroscope) = (5   ,   5)Hz. Delay (Accelerometer, Gyroscope) = (19.0 , 18.6)ms
} MPU6050_DLPFScaleTypedef;

/*
 * @brief 	Gyroscope Scale configuration bits.
 * @note	To be used in setMPU6050GyroscopeScale.
 */
typedef enum{
	/* Possible scales for gyroscope. Ref. Register Map Sec. 4.4 */
	MPU6050_GYROSCOPE_SCALE_250		= 0x00U,	// ± 250  degrees per second
	MPU6050_GYROSCOPE_SCALE_500		= 0x08U,	// ± 500  degrees per second
	MPU6050_GYROSCOPE_SCALE_1000	= 0x10U,	// ± 1000 degrees per second
	MPU6050_GYROSCOPE_SCALE_2000	= 0x18U		// ± 2000 degrees per second
} MPU6050_GyroscopeScaleTypedef;

/*
 * @brief	Accelerometer Scale configuration bits.
 * @note	To be used in setMPU6050AccelerometerScale.
 */
typedef enum{
	/* Possible scales for accelerometer. Ref. Register Map Sec. 4.5 */
	MPU6050_ACCELEROMETER_SCALE_2G	= 0x00U,	// ± 2  g
	MPU6050_ACCELEROMETER_SCALE_4G	= 0x08U,	// ± 4  g
	MPU6050_ACCELEROMETER_SCALE_8G	= 0x10U,	// ± 8  g
	MPU6050_ACCELEROMETER_SCALE_16G	= 0x18U		// ± 16 g
} MPU6050_AccelerometerScaleTypedef;

/*
 * @brief	FIFO Enable configuration bits.
 * @note 	To be used in setMPU6050FIFOEnable.
 */
typedef enum{
	MPU6050_FIFO_EN_TEMP = 0x80U,				// Temperature 			(Register 65 - 66)
	MPU6050_FIFO_EN_XG   = 0x40U,               // Gyroscope X			(Register 67 - 68)
	MPU6050_FIFO_EN_YG   = 0x20U,               // Gyroscope Y			(Register 69 - 70)
	MPU6050_FIFO_EN_ZG   = 0x10U,               // Gyroscope Z			(Register 71 - 72)
	MPU6050_FIFO_EN_A    = 0x08U,               // All Accelerometers	(Register 59 - 64)
	MPU6050_FIFO_EN_ALL_G= 0xF0U,               // All Gyroscopes
	MPU6050_FIFO_EN_ALL  = 0xF8U                // All Accelerometer and Gyroscopes
} MPU6050_FIFOEnableTypedef;

/*
 * @brief	Structure enum used in MPU6050_INTPinConfigTypedef
 */
typedef enum{
	MPU6050_INT_LEVEL_HIGH = 0x80,					// Active HIGH Interrupt.
	MPU6050_INT_LEVEL_LOW  = 0x00 					// Active LOW Interrupt.
} MPU6050_INTLevelTypedef;

/*
 * @brief	Structure enum used in MPU6050_INTPinConfigTypedef
 */
typedef enum{
	MPU6050_INT_OD_PUSHPULL  = 0x00,
	MPU6050_INT_OD_OPENDRAIN = 0x40
} MPU6050_INTODTypedef;

/*
 * @brief	Structure enum used in MPU6050_INTPinConfigTypedef
 */
typedef enum{
	MPU6050_INT_LATCH_PULSE	= 0x20,					// Pulse for 50us.
	MPU6050_INT_LATCH_HOLD  = 0x00					// Hold until clear.
} MPU6050_INTLatchTypedef;

/*
 * @brief	Structure enum used in MPU6050_INTPinConfigTypedef
 */
typedef enum{
	MPU6050_INT_READCLEAR_STATUSREG = 0x00,			// Clear by reading INT_STATUS.
	MPU6050_INT_READCLEAR_READANY 	= 0x08			// Clear by reading anything.
} MPU6050_INTReadClearTypedef;

/*
 * @brief	Configuration structure used to set INT Pin behavior.
 */
typedef struct{
	MPU6050_INTLevelTypedef LEVEL;
	MPU6050_INTODTypedef TYPE;
	MPU6050_INTLatchTypedef LATCH;
	MPU6050_INTReadClearTypedef CLEAR;
} MPU6050_INTPinConfigTypedef;

/*
 * @brief	Interrupt Enable configuration bits.
 */
typedef enum{
	MPU6050_INT_ENABLE_FIFO_OVERFLOW	 = 0x10,
	MPU6050_INT_ENABLE_I2C_MST_IN		 = 0x80,
	MPU6050_INT_ENABLE_DATA_READY		 = 0x01
} MPU6050_INTEnableTypedef;

/*
 * @brief	Power Cycle Rate configuration bits.
 * @note 	To be used in setMPU6050PowerCycle.
 */
typedef enum{
	/* Possible rates of power cycling. Ref. Register Map Sec. 4.29 */
	MPU6050_PWR_CYCLE_1_25Hz		= 0x00U,	// 1.25Hz
	MPU6050_PWR_CYCLE_5Hz			= 0x40U,	// 5Hz
	MPU6050_PWR_CYCLE_20Hz			= 0x80U,	// 20Hz
	MPU6050_PWR_CYCLE_40Hz			= 0xC0U		// 40Hz
} MPU6050_PowerCycleTypedef;

/*
 * @brief	Accelerometer axis enabling bits.
 * @note	To be used in setMPU6050Accelerometers.
 */
typedef enum{
	/* Defined parameters to set the state of accelerometer axes. Ref. Register Map Sec. 4.29 */
	MPU6050_ACCELEROMETER_X_SET		= 0x20U,
	MPU6050_ACCELEROMETER_Y_SET		= 0x10U,
	MPU6050_ACCELEROMETER_Z_SET		= 0x08U,
	MPU6050_ACCELEROMETER_ALL_SET	= 0x38U
} MPU6050_AccelerometerSetTypedef;

/*
 * @brief	Gyroscope axis enabling bits.
 * @note	To be used in setMPU6050Gyroscopes.
 */
typedef enum{
	/* Defined parameters to set the state of gyroscope axes. Ref. Register Map Sec. 4.29 */
	MPU6050_GYROSCOPE_X_SET			= 0x04U,
	MPU6050_GYROSCOPE_Y_SET			= 0x02U,
	MPU6050_GYROSCOPE_Z_SET			= 0x01U,
	MPU6050_GYROSCOPE_ALL_SET		= 0x07U
} MPU6050_GyroscopeSetTypedef;

/*
 * @brief	Switching values used by setter methods to determine the direction of the action.
 */
typedef enum{
	/* Unique parameters to enable / disable setters. */
	MPU6050_ENABLE	= 1,
	MPU6050_DISABLE	= 0
} MPU050_SetterTypedef;

/*
 * @brief	Standard return values for the outcomes of all methods in this file.
 * @note	Any computed data is returned by reference.
 */
typedef enum{
	/* Defined status return values for all methods. */
	MPU6050_OK 							=  1,
	MPU6050_FAIL						= -1,
	MPU6050_FAIL_INVALID_PARAMETER		= -2,
	MPU6050_FAIL_HAL					= -3,
	MPU6050_FAIL_DEVICE_NOT_FOUND		= -4,
	MPU6050_FAIL_SELF_TEST				= -5,
	MPU6050_WARNING						=  0
} MPU6050_ReturnTypedef;

/* End public typedefs. */

/* Begin public function prototypes. */

MPU6050_ReturnTypedef detectMPU6050(I2C_HandleTypeDef *);
MPU6050_ReturnTypedef selfTestMPU6050(I2C_HandleTypeDef *, MPU6050_AddressTypedef);

/* Resetters */
MPU6050_ReturnTypedef resetMPU6050(I2C_HandleTypeDef *, uint8_t);
MPU6050_ReturnTypedef resetMPU6050SignalCondition(I2C_HandleTypeDef *, uint8_t);
MPU6050_ReturnTypedef resetMPU6050SignalPaths(I2C_HandleTypeDef *, uint8_t, uint8_t);
MPU6050_ReturnTypedef resetMPU6050FIFO(I2C_HandleTypeDef *, uint8_t);

/* Setters */
MPU6050_ReturnTypedef setMPU6050(I2C_HandleTypeDef *, uint8_t, uint8_t);
MPU6050_ReturnTypedef setMPU6050SampleRate(I2C_HandleTypeDef *, uint8_t, uint8_t);

MPU6050_ReturnTypedef setMPU6050FIFOEnable(I2C_HandleTypeDef *, uint8_t, uint8_t);
MPU6050_ReturnTypedef setMPU6050FIFO(I2C_HandleTypeDef *, uint8_t, MPU050_SetterTypedef);

MPU6050_ReturnTypedef setMPU6050INTPinConfig(I2C_HandleTypeDef *, MPU6050_AddressTypedef, MPU6050_INTPinConfigTypedef);
MPU6050_ReturnTypedef setMPU6050INTEnable(I2C_HandleTypeDef *, MPU6050_AddressTypedef, MPU6050_INTEnableTypedef);

MPU6050_ReturnTypedef setMPU6050PowerCycle(I2C_HandleTypeDef *, uint8_t, uint8_t);
MPU6050_ReturnTypedef setMPU6050PowerCycleRate(I2C_HandleTypeDef *, uint8_t, uint8_t);

MPU6050_ReturnTypedef setMPU6050DLPF(I2C_HandleTypeDef *, uint8_t, uint8_t);

MPU6050_ReturnTypedef setMPU6050Gyroscopes(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t);
MPU6050_ReturnTypedef setMPU6050Accelerometers(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t);

MPU6050_ReturnTypedef setMPU6050GyroscopeScale(I2C_HandleTypeDef *, uint8_t, uint8_t);
MPU6050_ReturnTypedef setMPU6050AccelerometerScale(I2C_HandleTypeDef *, uint8_t, uint8_t);

MPU6050_ReturnTypedef setMPU6050TemperatureSensor(I2C_HandleTypeDef *, uint8_t, uint8_t);
MPU6050_ReturnTypedef setMPU6050ClockSource(I2C_HandleTypeDef *, uint8_t, uint8_t);

/* Getters */
MPU6050_ReturnTypedef getMPU6050AccelerometerMeasurements(I2C_HandleTypeDef *, uint8_t, float [3]);
MPU6050_ReturnTypedef getMPU6050AccelerometerData(I2C_HandleTypeDef *, uint8_t, int16_t [3]);

MPU6050_ReturnTypedef getMPU6050GyroscopeMeasurements(I2C_HandleTypeDef *, uint8_t, float [3]);

MPU6050_ReturnTypedef getMPU6050Temperature(I2C_HandleTypeDef *, uint8_t, float *);

MPU6050_ReturnTypedef getMPU6050FIFOCount(I2C_HandleTypeDef *, uint8_t, uint16_t *);
MPU6050_ReturnTypedef getMPU6050FIFO(I2C_HandleTypeDef *, uint8_t, uint16_t, uint8_t *);

/* End public function prototypes */

#endif /* INC_MPU6050_H_ */
