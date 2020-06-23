/**
  * @file           : MPU6050.h
  * @brief          : Body of methods to use the MPU6050.
  * @author			: Lawrence Stanton
  * @revised		: May 2020
  ******************************************************************************
  ******************************************************************************
  * @attention		© University of Cape Town 2019
  *
  * This file and its associates are under license. Please refer to the accompanying
  * LICENCE file for details. Use outside of the license is prohibited except for
  * exclusive use by the University of Cape Town or the SHARC Buoy Research Project.
  *
  * @instructions
  *
  ******************************************************************************
  */
/* Main includes */


/* math.h is necessary for a floating point exponent function and cannot be avoided without discarding the self test.
 * Consequently, the program immediately occupies >32KB, hence the need for a STM32F051C8T6 or some other chip with large flash capacity. */
#include <math.h>
#include "MPU6050.h"

/* Where a definition is referred to a header defined value it is because that value is intended to be passed into some method. */

#define SELF_TEST_X						0x0D
#define SELF_TEST_Y						0x0E
#define SELF_TEST_Z						0x0F
#define SELF_TEST_A						0x10
#define SELF_TEST_TOLORANCE				0.50
//#define SELF_TEST_TOLORANCE				0.14	// 14% Tolerance as per Datasheet Sec. 6.2

#define SMPLRT_DIV_ADDR					0x19

#define CONFIG_ADDR						0x1A
#define CONFIG_EXT_SYNC_SET				0x38
#define CONFIG_DLPF_CFG					0x07
#define CONFIG_DLPF_CFG_0				MPU6050_DLPF_A260_G256_Hz
#define CONFIG_DLPF_CFG_1				MPU6050_DLPF_A184_G188_Hz
#define CONFIG_DLPF_CFG_2				MPU6050_DLPF_A94_G98_Hz
#define CONFIG_DLPF_CFG_3				MPU6050_DLPF_A44_G42_Hz
#define CONFIG_DLPF_CFG_4				MPU6050_DLPF_A21_G20_Hz
#define CONFIG_DLPF_CFG_5				MPU6050_DLPF_A10_G10_Hz
#define CONFIG_DLPF_CFG_6				MPU6050_DLPF_A5_G5_Hz

#define GYRO_CONFIG_ADDR				0x1B
#define GYRO_CONFIG_XG_ST				0x80
#define GYRO_CONFIG_YG_ST				0x40
#define GYRO_CONFIG_ZG_ST				0x20
#define GYRO_CONFIG_FS_SEL				0x18
#define GYRO_CONFIG_FS_SEL_250			MPU6050_GYROSCOPE_SCALE_250
#define GYRO_CONFIG_FS_SEL_500			MPU6050_GYROSCOPE_SCALE_500
#define GYRO_CONFIG_FS_SEL_1000			MPU6050_GYROSCOPE_SCALE_1000
#define GYRO_CONFIG_FS_SEL_2000			MPU6050_GYROSCOPE_SCALE_2000

#define ACCEL_CONFIG_ADDR				0x1C
#define ACCEL_CONFIG_XA_ST				0x80
#define ACCEL_CONFIG_YA_ST				0x40
#define ACCEL_CONFIG_ZA_ST				0x20
#define ACCEL_CONFIG_AFS_SEL			0x18
#define ACCEL_CONFIG_AFS_SEL_2G			MPU6050_ACCELEROMETER_SCALE_2G
#define ACCEL_CONFIG_AFS_SEL_4G			MPU6050_ACCELEROMETER_SCALE_4G
#define ACCEL_CONFIG_AFS_SEL_8G			MPU6050_ACCELEROMETER_SCALE_8G
#define ACCEL_CONFIG_AFS_SEL_16G		MPU6050_ACCELEROMETER_SCALE_16G

#define FIFO_EN_ADDR					0x23
#define FIFO_EN_ALL						MPU6050_FIFO_EN_ALL

#define INT_PIN_CFG_ADDR				0x37
#define INT_PIN_CFG_ALL_INT				0xF0

#define INT_ENABLE_ADDR					0x38
#define INT_ENABLE_FIFO_OVERFLOW_EN		0x10
//#define INT_ENABLE_I2C_MST_INT_EN		0x80
#define INT_ENABLE_DATA_READY_EN		0x01

#define ACCEL_XOUT_ADDR1				0x3B	// ACCEL_XOUT[15:8]
#define ACCEL_XOUT_ADDR2				0x3C    // ACCEL_XOUT[7:0]
#define ACCEL_YOUT_ADDR1				0x3D    // ACCEL_YOUT[15:8]
#define ACCEL_YOUT_ADDR2				0x3E    // ACCEL_YOUT[7:0]
#define ACCEL_ZOUT_ADDR1				0x3F    // ACCEL_ZOUT[15:8]
#define ACCEL_ZOUT_ADDR2				0x40    // ACCEL_ZOUT[7:0]

#define TEMP_OUT_ADDR1					0x41	// TEMP_OUT[15:8]
#define TEMP_OUT_ADDR2					0x42	// TEMP_OUT[7:0]

#define GYRO_XOUT_ADDR1					0x43	// GYRO_XOUT[15:8]
#define GYRO_XOUT_ADDR2					0x44    // GYRO_XOUT[7:0]
#define GYRO_YOUT_ADDR1					0x45    // GYRO_YOUT[15:8]
#define GYRO_YOUT_ADDR2					0x46    // GYRO_YOUT[7:0]
#define GYRO_ZOUT_ADDR1					0x47    // GYRO_ZOUT[15:8]
#define GYRO_ZOUT_ADDR2					0x48    // GYRO_ZOUT[7:0]

#define SIGNAL_PATH_RESET_ADDR			0x68
#define SIGNAL_PATH_GYRO				MPU6050_PATH_RESET_GYROSCOPE
#define SIGNAL_PATH_ACCEL				MPU6050_PATH_RESET_ACCELEROMETER
#define SIGNAL_PATH_TEMP				MPU6050_PATH_RESET_TEMPERATURE
#define SIGNAL_PATH_ALL					MPU6050_PATH_RESET_ALL

#define USER_CONTROL_ADDR				0x6A
#define USER_CONTROL_FIFO_EN			0x40
#define USER_CONTROL_FIFO_RESET			0x04	// Currently unused.
#define USER_CONTROL_SIGNAL_COND_RESET	0x01

#define PWR_MGMT_1_ADDR					0x6B
#define PWR_MGMT_1_CLK_8MHz_HSI			MPU6050_CLOCK_8MHz_HSI
#define PWR_MGMT_1_CLK_X_GYRO			MPU6050_CLOCK_X_GYRO
#define PWR_MGMT_1_CLK_Y_GYRO			MPU6050_CLOCK_Y_GYRO
#define PWR_MGMT_1_CLK_Z_GYRO			MPU6050_CLOCK_Z_GYRO
#define PWR_MGMT_1_CLK_STOP				0x07
#define PWR_MGMT_1_TEMP_DIS	    		0x08
#define PWR_MGMT_1_CYCLE	    		0x20
#define PWR_MGMT_1_SLEEP	    		0x40
#define PWR_MGMT_1_RESET	    		0x80

#define PWR_MGMT_2_ADDR					0x6C
#define PWR_MGMT_2_LP_WAKE_1_25Hz		MPU6050_PWR_CYCLE_1_25Hz
#define PWR_MGMT_2_LP_WAKE_5Hz			MPU6050_PWR_CYCLE_5Hz
#define PWR_MGMT_2_LP_WAKE_20Hz			MPU6050_PWR_CYCLE_20Hz
#define PWR_MGMT_2_LP_WAKE_40Hz			MPU6050_PWR_CYCLE_40Hz
#define PWR_MGMT_2_STBY_XA				MPU6050_ACCELEROMETER_X_SET
#define PWR_MGMT_2_STBY_YA				MPU6050_ACCELEROMETER_Y_SET
#define PWR_MGMT_2_STBY_ZA				MPU6050_ACCELEROMETER_Z_SET
#define PWR_MGMT_2_STBY_XG				MPU6050_GYROSCOPE_X_SET
#define PWR_MGMT_2_STBY_YG				MPU6050_GYROSCOPE_Y_SET
#define PWR_MGMT_2_STBY_ZG				MPU6050_GYROSCOPE_Z_SET

#define FIFO_COUNT_H_ADDR				0x72
#define	FIFO_COUNT_L_ADDR				0x73

#define FIFO_R_W_ADDR					0x74

#define MPU6050_WHO_AM_I_REG_ADDR		0x75
#define MPU6050_WHO_AM_I_GND_AD0		MPU6050_I2C_ADDR_GND_AD0
#define MPU6050_WHO_AM_I_VCC_AD0		MPU6050_I2C_ADDR_VCC_AD0

//static MPU6050_ReturnTypedef setAG_Test(I2C_HandleTypeDef *, uint8_t, uint8_t [3], uint8_t [3]);
static MPU6050_ReturnTypedef getAG_Test(I2C_HandleTypeDef *, uint8_t, uint8_t [3], uint8_t [3]);
static MPU6050_ReturnTypedef setSelfTest(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t, uint8_t);

static MPU6050_ReturnTypedef setMPU6050SampleRateDivider(I2C_HandleTypeDef *, uint8_t, uint8_t);

static MPU6050_ReturnTypedef getAccelerometerScale(I2C_HandleTypeDef *, uint8_t, uint16_t *);

static MPU6050_ReturnTypedef getGyroscopeData(I2C_HandleTypeDef *, uint8_t, int16_t [3]);
static MPU6050_ReturnTypedef getGyroscopeScale(I2C_HandleTypeDef *, uint8_t, uint16_t *);

static MPU6050_ReturnTypedef get3AxisData(I2C_HandleTypeDef *, uint8_t, uint8_t, int16_t [3]);

static MPU6050_ReturnTypedef getRegister(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t*);
static MPU6050_ReturnTypedef getRegisters(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t*, uint8_t);

static MPU6050_ReturnTypedef setScaleSel(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t);

static MPU6050_ReturnTypedef setRegister(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t*);
static MPU6050_ReturnTypedef setRegisters(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t*, uint8_t);

static MPU6050_ReturnTypedef setBits(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t);
static MPU6050_ReturnTypedef clearBits(I2C_HandleTypeDef *, uint8_t, uint8_t, uint8_t);

static MPU6050_ReturnTypedef validAddress(uint8_t);
static MPU6050_ReturnTypedef readableRegister(uint8_t);
static MPU6050_ReturnTypedef writableRegister(uint8_t);

/**
  * @brief 	Test connection to MPU6050 by reading the value of the WHO_AM_I register (0x75)
  * @note	If 2 MPU6050s are on the same I2C line, this method will always find the AD0 grounded one first and ignore the other.
  * @param *hi2c:	The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef detectMPU6050(I2C_HandleTypeDef *hi2c){
	uint8_t Who_Am_I[1];																		// Byte to store value of register

	if( (getRegister(hi2c, MPU6050_I2C_ADDR_GND_AD0, MPU6050_WHO_AM_I_REG_ADDR, Who_Am_I) == MPU6050_OK) ||
		(getRegister(hi2c, MPU6050_I2C_ADDR_VCC_AD0, MPU6050_WHO_AM_I_REG_ADDR, Who_Am_I) == MPU6050_OK) ){	// If either of the addresses yields a response

			 if(Who_Am_I[0] == MPU6050_WHO_AM_I_GND_AD0) return MPU6050_OK;		// If the ID ends with 0 (GND on AD0)
		else if(Who_Am_I[0] == MPU6050_WHO_AM_I_VCC_AD0) return MPU6050_OK;		// Else if the ID ends with 1 (VCC on AD0)
		else 											 return MPU6050_FAIL_DEVICE_NOT_FOUND;	// Invalid response deemed to be not found (unlikely for this to execute)
	}
	else return MPU6050_FAIL_DEVICE_NOT_FOUND;													// No response, not found
}

MPU6050_ReturnTypedef selfTestMPU6050(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr){
	int status;

	/* Soft reset the MPU6050. */
	status = resetMPU6050(hi2c, MPU6050_Addr);
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;													// Return if error

	/* Enable the MPU6050 since it was reset. */
	status = setMPU6050(hi2c, MPU6050_Addr, MPU6050_ENABLE);
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;													// Return if error

	/* Set the accelerometer and gyroscope to 250dps and 8g respectively. */
	status = setMPU6050GyroscopeScale(hi2c, MPU6050_Addr, GYRO_CONFIG_FS_SEL_250);			// Not strictly needed as this is the default after reset
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;													// Return if error
	status = setMPU6050AccelerometerScale(hi2c, MPU6050_Addr, ACCEL_CONFIG_AFS_SEL_8G);		// Strictly needed as this is not the default after reset
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;													// Return if error

	HAL_Delay(10);																			// Delay to allow for refresh.

	/* All arrays follow X, Y, Z -> 0, 1, 2 indices */
	int16_t acel[2][3], gyro[2][3];		// Accelerometer and Gyroscope values with Test Off column [0] and Test On column [1].
	int16_t STR_A[3], STR_G[3];			// Self Test response values.
	uint8_t  A_Test[3], G_Test[3];		// Self Test parameters (Reg 0x0D - 0x10)
	float FT_A[3], FT_G[3];				// Factory Trim Values
	float  D_A[3],  D_G[3];				// Change from factory trim values

	/* Get the non-excited sensor data. */
	status = getMPU6050AccelerometerData(hi2c, MPU6050_Addr, acel[0]);
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;												// Return if error
	status = getGyroscopeData(hi2c, MPU6050_Addr, gyro[0]);
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;												// Return if error

	/* Enable the Self Test on all axes for both Gyroscope and Accelerometer */
	setSelfTest(hi2c, MPU6050_Addr, GYRO_CONFIG_ADDR,  GYRO_CONFIG_XG_ST  | GYRO_CONFIG_YG_ST  | GYRO_CONFIG_ZG_ST,  MPU6050_ENABLE);
	setSelfTest(hi2c, MPU6050_Addr, ACCEL_CONFIG_ADDR, ACCEL_CONFIG_XA_ST | ACCEL_CONFIG_YA_ST | ACCEL_CONFIG_ZA_ST, MPU6050_ENABLE);

	HAL_Delay(2);	// Guarantees 1 IMU clock cycle

	/* Get the excited sensor data. */
	status = getMPU6050AccelerometerData(hi2c, MPU6050_Addr, acel[1]);
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;												// Return if error
	status = getGyroscopeData(hi2c, MPU6050_Addr, gyro[1]);
	if(status != MPU6050_OK) return MPU6050_FAIL_HAL;												// Return if error

	/* Disable the Self Test on all axes for both Gyroscope and Accelerometer */
	setSelfTest(hi2c, MPU6050_Addr, GYRO_CONFIG_ADDR,  GYRO_CONFIG_XG_ST  | GYRO_CONFIG_YG_ST  | GYRO_CONFIG_ZG_ST,  MPU6050_DISABLE);
	setSelfTest(hi2c, MPU6050_Addr, ACCEL_CONFIG_ADDR, ACCEL_CONFIG_XA_ST | ACCEL_CONFIG_YA_ST | ACCEL_CONFIG_ZA_ST, MPU6050_DISABLE);

	/* Get the test parameters */
	getAG_Test(hi2c, MPU6050_Addr, A_Test, G_Test);

	/* For each axis */
	for(int j = 0; j < 3; j++){
		/* Calculate the self test response (STR) */
		STR_A[j] = acel[1][j] - acel[0][j];
		STR_G[j] = gyro[1][j] - gyro[0][j];

		/* Calculate the Factory Trim (FT) values based on the test parameters. Ref Register Map Sec. 4.1 */
		FT_A[j] = A_Test[j] == 0 ? 0.0 : 4096.0 * 0.34 * powf((0.92 / 0.34), ((float)A_Test[j] - 1.0) / ( (float)(1 << 5) - 2.0 ) );
		FT_G[j] = G_Test[j] == 0 ? 0.0 :  25.0 * 131.0 * powf(1.046, (float)G_Test[j] - 1.0 );
		if(j == 1) FT_G[j] *= -1.0;

		/* Determine the change from factory trim. */
		D_A[j] = ((float)STR_A[j] - FT_A[j]) / FT_A[j];
		D_G[j] = ((float)STR_G[j] - FT_G[j]) / FT_G[j];
	}

	HAL_Delay(1);

	for(int j = 0; j < 3; j++){
		/* Check the Tolerance */
		if( (D_A[j] > SELF_TEST_TOLORANCE ) || (D_A[j] < -SELF_TEST_TOLORANCE ) )  return MPU6050_FAIL_SELF_TEST;	// If fail in Accelerometer
		if( (D_G[j] > SELF_TEST_TOLORANCE ) || (D_G[j] < -SELF_TEST_TOLORANCE ) )  return MPU6050_FAIL_SELF_TEST;	// If fail in Gyroscope
	}

	return MPU6050_OK;					// If no failure detected return success
}

/**
  * @brief	Sets the weirdly ordered X, Y and Z test registers for both the gyroscope and accelerometer self test.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  A_Test[3]	The size 3 array of 5-bit test values for the accelerometer.
  * @param  G_Test[3]	The size 3 array of 5-bit test values for the gyroscope.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
/*static MPU6050_ReturnTypedef setAG_Test(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t A_Test[3], uint8_t G_Test[3]){
	uint8_t regVals[4];
	regVals[3] = 0;

	 Build the weird data structure. Ref. Register Map Sec. 4.1
	for(int i = 0; i < 3; i++) regVals[i] = ( (A_Test[i] & 0x1C) << 3) |  G_Test[i];		// Concatenate the first 3 registers
	for(int i = 0; i < 3; i++) regVals[3] |= ( (A_Test[i] & 0x03) << (4 - 2*i) );			// Build the last register

	return setRegisters(hi2c, MPU6050_Addr, SELF_TEST_X, regVals, 4);						// Write the registers and return the outcome
}*/

/**
  * @brief	Gets the weirdly ordered X, Y and Z test registers for both the gyroscope and accelerometer self test.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  A_Test[3]	The size 3 array for 5-bit test values of the accelerometer.
  * @param  G_Test[3]	The size 3 array for 5-bit test values of the gyroscope.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef getAG_Test(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t A_Test[3], uint8_t G_Test[3]){
	uint8_t regVals[4];
	int status = getRegisters(hi2c, MPU6050_Addr, SELF_TEST_X, regVals, 4);
	if(status != MPU6050_OK) return status;

	for(int i = 0; i < 3; i++){
		A_Test[i] = 0x00; G_Test[i] = 0x00;						// Zero all values

		A_Test[i] = (regVals[i] & 0xE0) >> 3;					// Get bits 2-4 and shift back by 3.
		A_Test[i] |= ((regVals[3] & 0x3F) >> (4-i*2)) & 0x03;	// Filter to [0:5], shift to place desired on [0:1], filter [0:1].
		G_Test[i] =  regVals[i] & 0x1F;							// Simply filter [0:5]
	}
	return MPU6050_OK;											// Return good if reached end.
}

/**
  * @brief	Sets the weirdly ordered X, Y and Z test registers for both the gyroscope and accelerometer self test.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  Config_Reg_Addr	The address of the configuration register to modify (GYRO_CONFIG_ADDR or ACCEL_CONFIG_ADDR).
  * @param  axes		The axes to set the self test on. Use N_CONFIG_NN_ST (N generic). Can bitwise OR multiple axes.
  * @param  state		Enable or Disable the self test (MPU6050_ENABLE or MPU6050_DISABLE).
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef setSelfTest(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t Config_Reg_Addr, uint8_t axes, uint8_t state){
	/* Check to see if the parameter axes is valid */
	if(axes & 0x1F) return MPU6050_FAIL_INVALID_PARAMETER;			// If any bit set in 0-4 return invalid parameter.

	if	   (state == MPU6050_ENABLE)  return setBits  (hi2c, MPU6050_Addr, Config_Reg_Addr, axes);	// If enable set the axes to self test
	else if(state == MPU6050_DISABLE) return clearBits(hi2c, MPU6050_Addr, Config_Reg_Addr, axes);	// If disable set the aces to not self test
	else return MPU6050_FAIL_INVALID_PARAMETER;												// Else invalid parameter on state
}

/**
  * @brief	Soft resets all registers of the MPU6050 to startup defaults.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @retval MPU6050_OK if either of 2 possible register values correctly passed. MPU6050_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef resetMPU6050(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr){
	return setBits(hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_RESET);
}

/**
  * @brief	Resets signal paths (scaling, filters, etc.) for all sensors.
  * @note	This also clears the sensor registers. Use resetMPU6050SignalPaths to avoid this.
  * @note	This may also reset the serial interface. Datasheet is not specific. Beware.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @retval MPU6050_OK if either of 2 possible register values correctly passed. MPU6050_FAIL_X otherwise.
  */
MPU6050_ReturnTypedef resetMPU6050SignalCondition(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr){
	return setBits(hi2c, MPU6050_Addr, USER_CONTROL_ADDR, USER_CONTROL_SIGNAL_COND_RESET);
}

/**
  * @brief	Resets signal paths (scaling, filters, etc.) for all sensors.
  * @note	This does not clear the sensor registers. Use resetMPU6050SignalCondition to reset the registers.
  * @note	This will also initialize the I2C interface. A small delay ensures that the device is given time for this.
  * @note	The bits written by this method will be automatically cleared by the reset.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param *huart:		HAL UART handle for printing errors to the serial connection.
  * @retval MPU6050_OK if either of 2 possible register values correctly passed. MPU6050_FAIL_X otherwise.
  */
MPU6050_ReturnTypedef resetMPU6050SignalPaths(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t paths){
	/* Check to see if the paths parameter is valid */
	if(paths > SIGNAL_PATH_ALL) return MPU6050_FAIL_INVALID_PARAMETER;
	/* Else continue to write bits to reset. */
	return setBits(hi2c, MPU6050_Addr, SIGNAL_PATH_RESET_ADDR, paths);
}

/**
  * @brief	Sets the state of the MPU6050 to sleep or not sleep.
  * @note	The MPU6050 is put to sleep at startup by default.
  * @note	Please use the standard definitions for MPU6050_Addr and state.
  * @note	Enabled = NOT Sleeping & vice versa.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param	state			MOU5060_ENABLE or MOU6050_DISABLE
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t state){
	/* Note: The logic of enabling & disabling is different from other similar methods.
	 * The MPU6050 sleeps (and is disabled) if the sleep bit is set, which differs from the logic of other set methods. */
	if	   (state == MPU6050_ENABLE)  return clearBits(hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_SLEEP);
	else if(state == MPU6050_DISABLE) return setBits  (hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_SLEEP);
	else return MPU6050_FAIL_INVALID_PARAMETER;
}

/**
  * @brief	Sets the register sample rate of the MPU6050.
  * @note	DEPRICATED. Do not use.
  * @note	The accelerometers only output at 1kHz. If a sample rate greater than this is selected, the device will produce duplicate results.
  * @note	The sample rate depends on the DLPF state. If this changes the resulting sample rate will change. See Register Map Sec. 4.2.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @param	sampleRate:		The register sample rate in kHz.
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050SampleRate(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t sampleRate){
	uint8_t DLPF[1];

	int status = getRegister(hi2c, MPU6050_Addr, CONFIG_ADDR, DLPF);			// Get the configuration register (0x1A)
	if(status != MPU6050_OK) return status;										// Return error if error.

	DLPF[0] &= CONFIG_DLPF_CFG;													// Filter to just have the DLPF information

	uint8_t gyroRate = ((DLPF[0] == 0x00) || (DLPF[0] == 0x07)) ? 8 : 1;		// If DLPF = 0 or 7, 8kHz, else 1kHz

	/* If the sample rate requested was greater than the gyroscope rate, set to maximum rate. */
	uint8_t divider = (gyroRate < sampleRate) ? 0 : ( (gyroRate - sampleRate ) / sampleRate );	// Set the divider value
	status = setMPU6050SampleRateDivider(hi2c, MPU6050_Addr, divider);

	return divider == 0 ? MPU6050_WARNING : MPU6050_OK;						// Return success if method completes or warning in case of corrective action.
}

/**
  * @brief	Sets the sample rate divider of the MPU6050.
  * @note	This method should normally be called by the method setSampleRate for predictable results.
  * @note	The sample rate depends on the DLPF state. If this changes the resulting sample rate will change. See Register Map Sec. 4.2.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @param	rate:			The rate at which the cycling should occur. See note.
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef setMPU6050SampleRateDivider(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t divider){
	return setRegister(hi2c, MPU6050_Addr, SMPLRT_DIV_ADDR, &divider);
}

/**
  * @brief	Sets the accelerometer-only low power cycling state.
  * @note	The device will automatically be set to sleep when power cycling is disabled and vice versa.
  * @note	The thermometer and gyroscopes will be automatically disabled when power cycling is enabled but not the other way around.
  * @note 	Automatically run when setting the power cycle rate (setMPU6050PowerCycleRate)
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @PARAM	state		MPU6050_ENABLE or MPU6050_DISABLE
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050PowerCycle(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t state){
	if(state == MPU6050_ENABLE){
		/* Disable the thermometer and gyroscopes */
		setMPU6050TemperatureSensor(hi2c, MPU6050_Addr, MPU6050_DISABLE);
		setMPU6050Gyroscopes(hi2c, MPU6050_Addr, MPU6050_GYROSCOPE_ALL_SET, MPU6050_DISABLE);

		/* Enable the MPU6050 and set the cycling but to begin cycling. */
		int status = setMPU6050(hi2c, MPU6050_Addr, MPU6050_ENABLE);		// Save the exit status disable sleep method.
		/* If the state was an error return the error. Otherwise return the outcome of the enabling of the cycle bit. */
		return status != MPU6050_OK ? status : setBits(hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_CYCLE);
	}
	else if(state == MPU6050_DISABLE){
		/* Sleep the MPU6050 and then clear the cycling bit. */
		uint8_t state = setMPU6050(hi2c, MPU6050_Addr, MPU6050_DISABLE);	// Save the exit status enable sleep method.
		/* If the state was an error return the error. Otherwise return the outcome of the disabling of the cycle bit. */
		return state !=MPU6050_OK ? state : clearBits(hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_CYCLE);
	}
	else return MPU6050_FAIL_INVALID_PARAMETER;
}

/**
  * @brief	Sets the sample rate while low-power cycling.
  * @note	Automatically enables power cycling by running setMPU6050PowerCycle
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @param	rate:			The rate at which the cycling should occur. Please use the PWR_CYCLE_X definitions.
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050PowerCycleRate(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t rate){
	/* Check to see if the value is supported. */
	if(!( (rate == PWR_MGMT_2_LP_WAKE_1_25Hz) ||
		  (rate == PWR_MGMT_2_LP_WAKE_5Hz)    ||
		  (rate == PWR_MGMT_2_LP_WAKE_20Hz)   ||
		  (rate == PWR_MGMT_2_LP_WAKE_40Hz) ) ) return MPU6050_FAIL_INVALID_PARAMETER;			// Return notice of invalid value.

	int status = clearBits(hi2c, MPU6050_Addr, PWR_MGMT_2_ADDR, PWR_MGMT_2_LP_WAKE_40Hz);		// Clear the LP_WAKE_CTRL bits.
	status = status != MPU6050_OK ? status : setBits(hi2c, MPU6050_Addr, PWR_MGMT_2_ADDR, rate);// If clear returned an error return the error, else set the rate and return.
	return status != MPU6050_OK ? status : setMPU6050PowerCycle(hi2c, MPU6050_Addr, MPU6050_ENABLE);	// Enable power cycling
}

/**
  * @brief	Sets the Digital Low Pass Filter setting of the NPU6050.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @param	filterSetting:	The set setting of the filter. Please use MPU6050_DLPF_.
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050DLPF(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t filterSetting){
	/* Check to see if the setting value is supported. */
	if(filterSetting >= 7) return MPU6050_FAIL_INVALID_PARAMETER;								// Return notice of invalid value.

	int status = clearBits(hi2c, MPU6050_Addr, CONFIG_ADDR, CONFIG_DLPF_CFG);					// Clear the bits in the DLPF section
	return status != MPU6050_OK ? status : setBits(hi2c, MPU6050_Addr, CONFIG_ADDR, filterSetting);	// If clear returned an error return the error, else set the setting and return.
}

/*
 * @brief	Sets the FIFO buffer inputs.
 */
MPU6050_ReturnTypedef setMPU6050FIFOEnable(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, MPU6050_FIFOEnableTypedef enableSetting){
	int status = clearBits(hi2c, MPU6050_Addr, FIFO_EN_ADDR, FIFO_EN_ALL);								// Clear the current register.
	return status != MPU6050_OK ? status : setBits(hi2c, MPU6050_Addr, FIFO_EN_ADDR, enableSetting);
  }

/*
 * @brief	Enables / disables the FIFO register.
 * @note	Automatically resets the FIFO upon disable.
 */
MPU6050_ReturnTypedef setMPU6050FIFO(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, MPU050_SetterTypedef state){
	int status;
	if(state == MPU6050_ENABLE) status = setBits(hi2c, MPU6050_Addr, USER_CONTROL_ADDR, USER_CONTROL_FIFO_EN);	// Enable the FIFI bit
	else{
		status = clearBits(hi2c, MPU6050_Addr, USER_CONTROL_ADDR, USER_CONTROL_FIFO_EN);						// Disable the FIFO bit
		if(status == MPU6050_OK) resetMPU6050FIFO(hi2c, MPU6050_Addr);		// Reset the FIFO register
	}
	return status;
}

/*
 * @brief	Resets the FIFO register.
 */
MPU6050_ReturnTypedef resetMPU6050FIFO(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr){
	return setBits(hi2c, MPU6050_Addr, USER_CONTROL_ADDR, USER_CONTROL_FIFO_RESET);
}

/*
 * @brief	Sets the Interrupt (INT) Pin behavior.
 * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
 * @param  MPU6050_Addr:	The I2C address of the MPU6050
 * @param  cfg:				The specific configuration structure for this method.
 */
MPU6050_ReturnTypedef setMPU6050INTPinConfig(I2C_HandleTypeDef *hi2c, MPU6050_AddressTypedef MPU6050_Addr, MPU6050_INTPinConfigTypedef cfg){
	int status = clearBits(hi2c, MPU6050_Addr, INT_PIN_CFG_ADDR, INT_PIN_CFG_ALL_INT);
	if(status == MPU6050_OK) status = setBits(hi2c, MPU6050_Addr, INT_PIN_CFG_ADDR, cfg.LEVEL |
																					cfg.TYPE |
																					cfg.LATCH |
																					cfg.CLEAR);
	return status;
}
/*
 * @brief	Sets interrupt enables.
 */
MPU6050_ReturnTypedef setMPU6050INTEnable(I2C_HandleTypeDef * hi2c, MPU6050_AddressTypedef MPU6050_Addr, MPU6050_INTEnableTypedef en){
	return setBits(hi2c, MPU6050_Addr, INT_ENABLE_ADDR, en);
}

/**
  * @brief	Sets the state of the gyroscopes on a per-axis basis.
  * @note	The gyroscopes are enabled by default.
  * @note	Multiple gyroscopes can be set at once by bitwise ORing several gyroscope definitions in param gyros.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @paramm	gyros:			Which gyroscopes to enable / disable. Use GYROSCOPE_N_SET. See note.
  * @param  state:			MPU6050_ENABLE or MPU6050_DISABLE
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050Gyroscopes(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t gyros, uint8_t state){
	/* Check to see if the gyros parameter is valid */
	if(gyros > 0x07) return MPU6050_FAIL_INVALID_PARAMETER;											// Return notice of invalid value.

		 if(state == MPU6050_ENABLE)  return clearBits(hi2c, MPU6050_Addr, PWR_MGMT_2_ADDR, gyros);	// Clear the bits to bring the gyros out of standby.
	else if(state == MPU6050_DISABLE) return setBits  (hi2c, MPU6050_Addr, PWR_MGMT_2_ADDR, gyros);	// Set the bits to put the gyros into standby.
	else return MPU6050_FAIL_INVALID_PARAMETER;														// Invalid parameter state.
}

MPU6050_ReturnTypedef setMPU6050GyroscopeScale(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t scale){
	return setScaleSel(hi2c, MPU6050_Addr, GYRO_CONFIG_ADDR, scale);
}

/**
  * @brief	Sets the state of the accelerometers on a per-axis basis.
  * @note	The accelerometers are enabled by default.
  * @note	Multiple accelerometers can be set at once by bitwise ORing several acelorometers definitions in param acceleros.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @param  acceleros:		Which accelerometers to enable / disable. Use ACCELEROETER_N_SET. See note.
  * @param  state:			MPU6050_ENABLE or MPU6050_DISABLE
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050Accelerometers(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t acceleros, uint8_t state){
	/* Check to see if the acceleros parameter is valid */
	if( (acceleros < 0x08) || (acceleros > 0x38) ) return MPU6050_FAIL_INVALID_PARAMETER;

		 if(state == MPU6050_ENABLE)  return clearBits(hi2c, MPU6050_Addr, PWR_MGMT_2_ADDR, acceleros);	// Clear the bits to bring the accelerometers out of standby.
	else if(state == MPU6050_DISABLE) return setBits  (hi2c, MPU6050_Addr, PWR_MGMT_2_ADDR, acceleros);	// Set the bits to put the accelerometers into standby.
	else return MPU6050_FAIL_INVALID_PARAMETER;															// Invalid parameter state.
}

/**
  * @brief	Sets the scale of the accelerometers.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @param  scale:			The scale to be used. Please use MPU6050_ACCELLEROMETER_SCALE_X.
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050AccelerometerScale(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t scale){
	return setScaleSel(hi2c, MPU6050_Addr, ACCEL_CONFIG_ADDR, scale);
}

MPU6050_ReturnTypedef setScaleSel(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t Config_Register_Addr, uint8_t scale){
	/* Check to see if the scale parameter is valid */
	if(scale & ~0x18) return MPU6050_FAIL_INVALID_PARAMETER;						// If any bit outside of bits 3-4

	int status = clearBits(hi2c, MPU6050_Addr, Config_Register_Addr, ACCEL_CONFIG_AFS_SEL);				// Clear the (A)FS_SEL bits. Used ACCEL_CONFIG_AFS_SEL but is identical to GYRO_CONFIG_FS_SEL
	return status != MPU6050_OK ? status : setBits(hi2c, MPU6050_Addr, Config_Register_Addr, scale);	// If clear returned an error, return the error or set the new scale
}

/**
  * @brief	Sets the state of the on-bpard temperature sensor.
  * @note	The temperature sensor is enabled by default.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr:	The I2C address of the MPU6050
  * @param  state:			MPU6050_ENABLE or MPU6050_DISABLE
  * @retval MPU6050_OK if success, MPU_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050TemperatureSensor(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t state){
		 if(state == MPU6050_ENABLE)  return clearBits(hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_TEMP_DIS);	// Clear the disabling bit.
	else if(state == MPU6050_DISABLE) return setBits  (hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_TEMP_DIS);	// Set the disabling bit.
	else return MPU6050_FAIL_INVALID_PARAMETER;																		// Invalid parameter state.
}

/**
  * @brief	Sets the clock source of the MPU6050.
  * @note	The 8MHz relaxation oscillator is enabled by default.
  * @note	Only the internal oscillator and gyro oscillators are currently supported. Any other source will be rejected.
  * @param *hi2c:			The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  clock			The clock identifier. Ref. Register Map Sec. 4.28.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef setMPU6050ClockSource(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t clock){
	/* Check if the value is supported. */
	if( (clock > 7) || ((4 <= clock) && (clock <=6)) ) return MPU6050_FAIL_INVALID_PARAMETER;	// Return with notice of invalid clock

	int status = clearBits(hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, PWR_MGMT_1_CLK_STOP);			// Save the result of the clear operation
	return status != MPU6050_OK ? status : setBits(hi2c, MPU6050_Addr, PWR_MGMT_1_ADDR, clock);	// If clear returned an error return the error, else set the clock source and return.
}

/**
  * @brief	Reads the current accelerometer register data for all 3 axes and scales the result to a value in g's.
  * @note	If memory was not sufficiently allocated the method will overwrite unintended memory without warning.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  data[3]		A pointer to a size 3 floating point array to store the result.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef getMPU6050AccelerometerMeasurements(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, float data[3]){
	int16_t rawData[3];

	int status = getMPU6050AccelerometerData(hi2c, MPU6050_Addr, rawData);		// Get the raw values and save the outcome status
	if( status != MPU6050_OK) return status;							// Return the error if an error occurred

	uint16_t scale;
	status = getAccelerometerScale(hi2c, MPU6050_Addr, &scale);			// Get the scale and save the outcome status
	if( status != MPU6050_OK) return status;							// Return the error if an error occurred

	for(int i = 0; i < 3; i++) data[i] = ( (float)rawData[i] ) / scale;	// Calculate the value in terms of g's
	return MPU6050_OK;													// Return OK if method completes;
}

/**
  * @brief	Reads the current accelerometer register data for all 3 axes.
  * @note	This produces the effective 16-bit register values. For actual values please use getMPU6050AccelerometerMeasurements().
  * @note	If memory was not sufficiently allocated the method will overwrite unintended memory without warning.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  data[3]		A pointer to a size 3 array to store the data.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef getMPU6050AccelerometerData(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, int16_t data[3]){
	return get3AxisData(hi2c, MPU6050_Addr, ACCEL_XOUT_ADDR1, data);
}

/**
  * @brief	Reads the Accelerometer Configuration Register (0x1C) and interprets the set scale of the accelerometer.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param *scale		A pointer to save the value of LSB per g.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef getAccelerometerScale(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint16_t *scale){
	uint8_t reg[1];

	int status  = getRegister(hi2c, MPU6050_Addr, ACCEL_CONFIG_ADDR, reg);	// Get the configuration register and save the outcome.
	if(status != MPU6050_OK) return status;									// If an error occurred return the error.

	reg[0] = (reg[0] & ACCEL_CONFIG_AFS_SEL) >> 3;	// Filter the AFFS-SEL bits and divide the value to the actual selection.
	*scale = 0x4000 >> reg[0];						// Formula for determining the LSB per g. Ref. Register Map Sec. 4.17.

	return MPU6050_OK;								// Return OK if method completes.
}

/**
  * @brief	Reads the current gyroscope register data for all 3 axes and scales the result to a value in degrees/s.
  * @note	If memory was not sufficiently allocated the method will overwrite unintended memory without warning.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  data[3]		A pointer to a size 3 floating point array to store the result.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
MPU6050_ReturnTypedef getMPU6050GyroscopeMeasurements(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, float data[3]){
	int16_t rawData[3];

	int status = getGyroscopeData(hi2c, MPU6050_Addr, rawData);			// Get the raw values and save the outcome status
	if( status != MPU6050_OK) return status;							// Return the error if an error occurred

	uint16_t scale;
	status = getGyroscopeScale(hi2c, MPU6050_Addr, &scale);				// Get the scale and save the outcome status
	if( status != MPU6050_OK) return status;							// Return the error if an error occurred

	for(int i = 0; i < 3; i++) data[i] = ( (float)rawData[i] ) / scale;	// Calculate the value in terms of deg/s
	return MPU6050_OK;													// Return OK if method completes;
}

/**
  * @brief	Reads the current gyroscope register data for all 3 axes.
  * @note	This produces the effective 16-bit register values. For actual values please use getMPU6050GyroscopeMeasurements().
  * @note	If memory was not sufficiently allocated the method will overwrite unintended memory without warning.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  data[3]		A pointer to a size 3 array to store the data.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef getGyroscopeData(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, int16_t data[3]){
	return get3AxisData(hi2c, MPU6050_Addr, GYRO_XOUT_ADDR1, data);
}

/**
  * @brief	Reads the Gyroscope Configuration Register (0x1B) and interprets the set scale of the gyroscope.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param *scale		A pointer to save the value of LSB per deg/s.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef getGyroscopeScale(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint16_t *scale){
	uint8_t reg[1];

	int status  = getRegister(hi2c, MPU6050_Addr, GYRO_CONFIG_ADDR, reg);	// Get the configuration register and save the outcome.
	if(status != MPU6050_OK) return status;									// If an error occurred return the error.

	reg[0] = (reg[0] & GYRO_CONFIG_FS_SEL) >> 3;	// Filter the AFFS-SEL bits and divide the value to the actual selection.
	*scale = 250 << reg[0];							// Formula for determining the LSB per deg/s. Ref. Register Map Sec. 4.19.

	return MPU6050_OK;								// Return OK if method completes.
}


static MPU6050_ReturnTypedef get3AxisData(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t Reg_0_Addr, int16_t data[3]){
	uint8_t rawData[6];																// Data holder for all 6 1-byte registers.

	int status = getRegisters(hi2c, MPU6050_Addr, Reg_0_Addr, rawData, 6);			// Fetch 6 registers starting at Reg_0 and save the status.
	if( status != MPU6050_OK) return status;										// If an error occurred return the error

	for(int i = 0; i < 3; i++) data[i] = (rawData[2*i] << 8) | rawData[2*i+1];		// Merge the byte registers into a 16-bit value

	return MPU6050_OK;																// Return success of method reached end
}

MPU6050_ReturnTypedef getMPU6050Temperature(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, float *temp){
	uint8_t rawData[2];															// Copy of MPU6050 registers

	int status = getRegisters(hi2c, MPU6050_Addr, TEMP_OUT_ADDR1, rawData, 2);	// Get the registers and save the outcome status.
	int16_t T = (rawData[0] << 8) | rawData[1];									// Concatenate the registers
	*temp = ((float)T / 340) + 36.53;											// Adjust as per the formula. Reg. Register Map Sec.4.18.

	return status;															// Return OK if method completes.
}

/*
 * @brief 	Gets the next n contents of the FIFO register.
 * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
 * @param  MPU6050_Addr	The I2C address of the MPU6050.
 * @param *cont			Pointer to save the value of FIFO_COUNT
 */
MPU6050_ReturnTypedef getMPU6050FIFOCount(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint16_t *count){
	uint8_t data[2];															// Data allocation
	int status = getRegisters(hi2c, MPU6050_Addr, FIFO_COUNT_H_ADDR, data, 2);	// Get FIFO Count data.
	*count = (data[0] << 8) | data[1];											// Merge registers.
	return status;																// Return result of transaction.
 }

/*
 * @brief 	Gets the next n contents of the FIFO register.
 * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
 * @param  MPU6050_Addr	The I2C address of the MPU6050.
 * @param  n			The number of bytes to read. Allocate enough data!
 * @param *data			Pointer to allocated memory.
 */
MPU6050_ReturnTypedef getMPU6050FIFO(I2C_HandleTypeDef * hi2c, uint8_t MPU6050_Addr, uint16_t n, uint8_t * data){
	int status;
	for(int i = 0; i < n; i++) {
		status = getRegister(hi2c, MPU6050_Addr, FIFO_R_W_ADDR, &data[i]);
		if(status != MPU6050_OK) return status;
	}
	return status;
}

/**
  * @brief	Sets individual bits in a specific register of the MPU6050.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  MPU6050_Register_Addr: The internal register to be modified.
  * @param  bits:		The bit pattern. Any binary 1 will be set to the register.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef setBits(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t MPU6050_Register_Addr, uint8_t bits){
	uint8_t reg[1];																// Copy of current register value

	int status = getRegister(hi2c, MPU6050_Addr, MPU6050_Register_Addr, reg);	// Save the result of the read operation.
	if( status != MPU6050_OK) return status;									// Return with the error if it is an error.
																				// Otherwise method continues.
	reg[0] |= bits;																// Set the bits
	return setRegister(hi2c, MPU6050_Addr, MPU6050_Register_Addr, reg);			// Write modified register back
}

/**
  * @brief	Clears individual bits in a specific register of the MPU6050.
  * @param *hi2c:		The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr	The I2C address of the MPU6050
  * @param  MPU6050_Register_Addr: The internal register to be modified.
  * @param  bits:		The bit pattern. Any binary 1 will be cleared from the register.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef clearBits(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t MPU6050_Register_Addr, uint8_t bits){
	uint8_t reg[1];																// Copy of current register value

	int status = getRegister(hi2c, MPU6050_Addr, MPU6050_Register_Addr, reg);	// Save the result of the read operation.
	if( status != MPU6050_OK) return status;										// Return with the error if it is an error.
																					// Otherwise method continues.
	reg[0] &= ~bits;															// Clear the bits
	return setRegister(hi2c, MPU6050_Addr, MPU6050_Register_Addr, reg);			// Write modified register back
}

/**
  * @brief Reads a single register from the MPU6050.
  * @note  This simply calls the getRegisters() method since the HAL method automatically increments.
  * @param *hi2c:		 The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr: The I2C address of the MPU6050. Use MPU6050_I2C_ADDR_GND_AD0 or MPU6050_I2C_ADDR_VCC_AD0.
  * @param  MPU6050_Register_Addr: The internal register to be written.
  * @param *data:		 Pointer to where the register data is to be stored.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef getRegister(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t MPU6050_Register_Addr, uint8_t* data){
	return getRegisters(hi2c, MPU6050_Addr, MPU6050_Register_Addr, data, 1);
}

/**
  * @brief Write a single register to the MPU6050.
  * @note  This simply calls the setRegisters() method since the HAL method automatically increments.
  * @param *hi2c:		 The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr: The I2C address of the MPU6050. Use MPU6050_I2C_ADDR_GND_AD0 or MPU6050_I2C_ADDR_VCC_AD0.
  * @param  MPU6050_Register_Addr: The internal register to be written.
  * @param *data:		 Pointer to the data to be written.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef setRegister(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t MPU6050_Register_Addr, uint8_t *data){
	return setRegisters(hi2c, MPU6050_Addr, MPU6050_Register_Addr, data, 1);
}

/**
  * @brief Reads a set of sequential registers from the MPU6050.
  * @note  This simply calls the getRegisters() method since the HAL method automatically increments.
  * @param *hi2c:		 The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr: The I2C address of the MPU6050. Use MPU6050_I2C_ADDR_GND_AD0 or MPU6050_I2C_ADDR_VCC_AD0.
  * @param  MPU6050_Register_Addr: The internal register to be written.
  * @param *data:		 Pointer to where the register data is to be stored.
  * @param size:		 the number of registers to be read.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef getRegisters(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t MPU6050_Register_Addr, uint8_t* data, uint8_t size){
	if( (validAddress(MPU6050_Addr) == MPU6050_OK) && (readableRegister(MPU6050_Register_Addr) == MPU6050_OK) ){				// Check for valid parameters
		if(HAL_I2C_Mem_Read(hi2c, MPU6050_Addr << 1, MPU6050_Register_Addr, 1, data, size, HAL_MAX_DELAY) == HAL_OK) return MPU6050_OK;		// If read success
		else return MPU6050_FAIL_HAL;			// HAL Method I2C transfer unsuccessful.
	}
	else return MPU6050_FAIL_INVALID_PARAMETER;
}

/**
  * @brief Write a set of sequential registers to the MPU6050.
  * @note  The lowest address register is stored at the lowest index.
  * @param *hi2c:		 The HAL I2C handle for the I2C line that the MPU6050 is on.
  * @param  MPU6050_Addr: The I2C address of the MPU6050. Use MPU6050_I2C_ADDR_GND_AD0 or MPU6050_I2C_ADDR_VCC_AD0.
  * @param  MPU6050_Register_Addr: The internal register to be written.
  * @param *data:		 Pointer to the data to be written.
  * @param  size:		 the number of registers to be set.
  * @retval MPU6050_OK if success. MPU6050_FAIL_ otherwise.
  */
static MPU6050_ReturnTypedef setRegisters(I2C_HandleTypeDef *hi2c, uint8_t MPU6050_Addr, uint8_t MPU6050_Register_Addr, uint8_t* data, uint8_t size){
	/* Check for valid parameters */
	if( (validAddress(MPU6050_Addr) == MPU6050_OK)){
		for(int i = 0; i < size; i++) if(writableRegister(MPU6050_Register_Addr) != MPU6050_OK) return MPU6050_FAIL_INVALID_PARAMETER;

		if(HAL_I2C_Mem_Write(hi2c, MPU6050_Addr << 1, MPU6050_Register_Addr, 1, data, size, HAL_MAX_DELAY) == HAL_OK) return MPU6050_OK;	// If read success
		else return MPU6050_FAIL_HAL;			// HAL Method I2C transfer unsuccessful.
	}
	else return MPU6050_FAIL_INVALID_PARAMETER;
}

/**
  * @brief Checks if the address used is indeed one of the two possible MPU6050 I2C Addresses
  * @param Address: The intended address of the MPU6050
  * @retval MPU6050_OK if valid, MPU6050_FAIL_INVALID_PARAMETER if invalid
  */
static MPU6050_ReturnTypedef validAddress(uint8_t Address){
	if((Address == MPU6050_I2C_ADDR_GND_AD0) || (Address == MPU6050_I2C_ADDR_VCC_AD0)) return MPU6050_OK;
	else return MPU6050_FAIL_INVALID_PARAMETER;
}

/**
  * @brief Checks if the register address used is indeed a readable MPU6050 register.
  * @param Reg_Address: The intended address of the register.
  * @retval MPU6050_OK if valid, MPU6050_FAIL_INVALID_PARAMETER if invalid
  */
static MPU6050_ReturnTypedef readableRegister(uint8_t Reg_Address){
	/* Check if within one of the following ranges of readable registers as per the memory map. */
	if( ( (0x0D <= Reg_Address) && (Reg_Address <= 0x10) )  ||
		( (0x19 <= Reg_Address) && (Reg_Address <= 0x1C) )  ||
		( (0x23 <= Reg_Address) && (Reg_Address <= 0x38) )  ||
		( (0x3A <= Reg_Address) && (Reg_Address <= 0x60) )  ||
		( (0x63 <= Reg_Address) && (Reg_Address <= 0x68) )  ||
		( (0x6A <= Reg_Address) && (Reg_Address <= 0x6C) )  ||
		( (0x72 <= Reg_Address) && (Reg_Address <= 0x75) )  ) return MPU6050_OK;
	else return MPU6050_FAIL_INVALID_PARAMETER;
}

/**
  * @brief Checks if the register address used is indeed a writable MPU6050 register.
  * @param Reg_Address: The intended address of the register.
  * @retval MPU6050_OK if valid, MPU6050_FAIL_INVALID_PARAMETER if invalid
  */
static MPU6050_ReturnTypedef writableRegister(uint8_t Reg_Address){
	/* Check if within one of the following ranges of readable registers as per the memory map. */
	if( ( (0x0D <= Reg_Address) && (Reg_Address <= 0x10) )  ||
		( (0x19 <= Reg_Address) && (Reg_Address <= 0x1C) )  ||
		( (0x23 <= Reg_Address) && (Reg_Address <= 0x34) )  ||
		( (0x37 <= Reg_Address) && (Reg_Address <= 0x38) )  ||
		( (0x63 <= Reg_Address) && (Reg_Address <= 0x68) )  ||
		( (0x6A <= Reg_Address) && (Reg_Address <= 0x6C) )  ||
		( (0x72 <= Reg_Address) && (Reg_Address <= 0x74) )  ) return MPU6050_OK;
	else return MPU6050_FAIL_INVALID_PARAMETER;
}
