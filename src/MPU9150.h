
#ifndef MPU9150_H_
#define MPU9150_H_
// Includes
#include <cstdint>

// Gyroscope and Accelerometer Registers
#define MPU9150_SELF_TEST_X			0x0D
#define MPU9150_SELF_TEST_Y			0x0E
#define MPU9150_SELF_TEST_Z			0x0F
#define MPU9150_SELF_TEST_A			0x10
#define MPU9150_SMPLRT_DIV			0x19
#define MPU9150_CONFIG				0x1A
#define MPU9150_GYRO_CONFIG			0x1B
#define MPU9150_ACCEL_CONFIG		0x1C
#define MPU9150_FF_THR				0x1D
#define MPU9150_FF_DUR				0x1E
#define MPU9150_MOT_THR				0x1F
#define MPU9150_MOT_DUR				0x20
#define MPU9150_ZRMOT_THR			0x21
#define MPU9150_ZRMOT_DUR			0x22
#define MPU9150_FIFO_EN				0x23
#define MPU9150_I2C_MST_CTRL		0x24
#define MPU9150_I2C_SLV0_ADDR		0x25
#define MPU9150_I2C_SLV0_REG		0x26
#define MPU9150_I2C_SLV0_CTRL		0x27
#define MPU9150_I2C_SLV1_ADDR		0x28
#define MPU9150_I2C_SLV1_REG		0x29
#define MPU9150_I2C_SLV1_CTRL		0x2A
#define MPU9150_I2C_SLV2_ADDR		0x2B
#define MPU9150_I2C_SLV2_REG		0x2C
#define MPU9150_I2C_SLV2_CTRL		0x2D
#define MPU9150_I2C_SLV3_ADDR		0x2E
#define MPU9150_I2C_SLV3_REG		0x2F
#define MPU9150_I2C_SLV3_CTRL		0x30
#define MPU9150_I2C_SLV4_ADDR		0x31
#define MPU9150_I2C_SLV4_REG		0x32
#define MPU9150_I2C_SLV4_DO			0x33
#define MPU9150_I2C_SLV4_CTRL		0x34
#define MPU9150_I2C_SLV4_DI			0x35
#define MPU9150_I2C_MST_STATUS		0x36
#define MPU9150_I2C_INT_PIN_CFG		0x37
#define MPU9150_INT_ENABLE			0x38
#define MPU9150_IN_STATUS			0x3A
#define MPU9150_ACCEL_XOUT_H		0x3B
#define MPU9150_ACCEL_XOUT_L		0x3C
#define MPU9150_ACCEL_YOUT_H		0x3D
#define MPU9150_ACCEL_YOUT_L		0x3E
#define MPU9150_ACCEL_ZOUT_H		0x3F
#define MPU9150_ACCEL_ZOUT_L		0x40
#define MPU9150_TEMP_OUT_H			0x41
#define MPU9150_TEMP_OUT_L			0x42
#define MPU9150_GYRO_XOUT_H			0x43
#define MPU9150_GYRO_XOUT_L			0x44
#define MPU9150_GYRO_YOUT_H			0x45
#define MPU9150_GYRO_YOUT_L			0x46
#define MPU9150_GYRO_ZOUT_H			0x47
#define MPU9150_GYRO_ZOUT_L			0x48
#define MPU9150_EXT_SENS_DATA_00	0x49
#define MPU9150_EXT_SENS_DATA_01	0x4A
#define MPU9150_EXT_SENS_DATA_02	0x4B
#define MPU9150_EXT_SENS_DATA_03	0x4C
#define MPU9150_EXT_SENS_DATA_04	0x4D
#define MPU9150_EXT_SENS_DATA_05	0x4E
#define MPU9150_EXT_SENS_DATA_06	0x4F
#define MPU9150_EXT_SENS_DATA_07	0x50
#define MPU9150_EXT_SENS_DATA_08	0x51
#define MPU9150_EXT_SENS_DATA_09	0x52
#define MPU9150_EXT_SENS_DATA_10	0x53
#define MPU9150_EXT_SENS_DATA_11	0x54
#define MPU9150_EXT_SENS_DATA_12	0x55
#define MPU9150_EXT_SENS_DATA_13	0x56
#define MPU9150_EXT_SENS_DATA_14	0x57
#define MPU9150_EXT_SENS_DATA_15	0x58
#define MPU9150_EXT_SENS_DATA_16	0x59
#define MPU9150_EXT_SENS_DATA_17	0x5A
#define MPU9150_EXT_SENS_DATA_18	0x5B
#define MPU9150_EXT_SENS_DATA_19	0x5C
#define MPU9150_EXT_SENS_DATA_20	0x5D
#define MPU9150_EXT_SENS_DATA_21	0x5E
#define MPU9150_EXT_SENS_DATA_22	0x5F
#define MPU9150_EXT_SENS_DATA_23	0x60
#define MPU9150_MOT_DETECT_STATUS	0x61
#define MPU9150_I2C_SLV0_DO			0x63
#define MPU9150_I2C_SLV1_DO			0x64
#define MPU9150_I2C_SLV2_DO			0x65
#define MPU9150_I2C_SLV3_DO			0x66
#define MPU9150_I2C_MST_DELAY_CTRL	0x67
#define MPU9150_SIGNAL_PATH_RESET	0x68
#define MPU9150_MOT_DETECT_CTRL		0x69
#define MPU9150_USERCTRL			0x6A
#define MPU9150_PWR_MGMT_1			0x6B
#define MPU9150_PWR_MGMT_2			0x6C
#define MPU9150_FIFO_COUNTH			0x72
#define MPU9150_FIFO_COUNTL			0x73
#define MPU9150_FIFO_R_W			0x74
#define MPU9150_WHO_AM_I			0x75
#define MPU9150_MAG_WIA				0x00
#define MPU9150_MAG_INFO			0x01
#define MPU9150_MAG_ST1				0x02
#define MPU9150_MAG_HXL				0x03
#define MPU9150_MAG_HXH				0x04
#define MPU9150_MAG_HYL				0x05
#define MPU9150_MAG_HYH				0x06
#define MPU9150_MAG_HZL				0x07
#define MPU9150_MAG_HZH				0x08
#define MPU9150_MAG_ST2				0x09
#define MPU9150_MAG_CNTL			0x0A
#define MPU9150_MAG_RSV				0x0B
#define MPU9150_MAG_ASTC			0x0C
#define MPU9150_MAG_TS1				0x0D
#define MPU9150_MAG_TS2				0x0E
#define MPU9150_MAG_I2CDIS			0x0F
#define MPU9150_MAG_ASAX			0x10
#define MPU9150_MAG_ASAY			0x11
#define MPU9150_MAG_ASAZ			0x12

#define MPU9150_MAG_ADDR			0x0C

class MPU9150{
	private:
		uint8_t I2CBus, I2CAddress;
		int writeByte(int device, uint8_t regAddr, uint8_t value);
		int writeBits(int device, uint8_t regAddr, uint8_t value, uint8_t bitmask);
		int readByte(int device, uint8_t regAddr, uint8_t* value);
		int mpuFile, magFile;
		int16_t accel_X, accel_Y, accel_Z;
		int16_t temp;
		int16_t gyro_X, gyro_Y, gyro_Z;
		int16_t mag_X, mag_Y, mag_Z;
	public:
		/**
		 * MPU9150 constructor.  Accepts for arguments the I2C bus.  Assumes
		 * default address of 0x68.
		 */
		MPU9150(int bus);
		
		/**
		 * MPU9150 alternate constructor.  Accepts as arguments th2 I2C bus and
		 * bus address.
		 */
		MPU9150(int bus, int address);
		
		/**
		 * Initialization function.  Initializes MPU9150 to a working state.
		 * Sets register at 0x6B to 0x00
		 */
		int initialize();
		
		/**
		 * Update function.  Records full sensor state.
		 */
		int getSensorState();
		
		/**
		 * Returns the acceleration in the X axis in g's.
		 */
		float getAccelX();
		
		/**
		 * Returns the acceleration in the Y axis in g's.
		 */
		float getAccelY();
		
		/**
		 * Returns the acceleration in the Z axis in g's.
		 */
		float getAccelZ();
		
		/**
		 * Returns the rotational velocity in the X axis in degrees per second.
		 */
		float getGyroX();
		
		/**
		 * Returns the rotational velocity in the Y axis in degrees per second.
		 */
		float getGyroY();
		
		/**
		 * Returns the rotational velocity in the Z axis in degrees per second.
		 */
		float getGyroZ();
		
		/**
		 * Returns the magnetic field strength in the X axis in microteslas.
		 */
		float getMagX();
		
		/**
		 * Returns the magnetic field strength in the Y axis in microteslas.
		 */
		float getMagY();
		
		/**
		 * Returns the magnetic field strength in the Z axis in microteslas.
		 */
		float getMagZ();
};



#endif // MPU9150_H_
