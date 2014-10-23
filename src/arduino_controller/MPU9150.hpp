#include <wire.h>
#define ADDR1	0x68
#define ADDR2	0x0C

uint8_t _i2c_addr = 0;

void _i2c_write(uint8_t dest, uint8_t val){
	Wire.beginTransmission(_i2c_addr);	// select I2C device
	Wire.write(dest);	// Select device register
	Wire.write(val);	// Write value
	Wire.endTransmission(true);	// issue stop command
}
void _i2c_write_bits(uint8_t dest, uint8_t 
uint8_t _i2c_read(uint8_t reg){
	Wire.beginTransmission(_i2c_addr);
	Wire.requestFrom(reg, 1, true);
	while(!Wire.available){
		// do nothing
	}
	return Wire.read();
}
void MPU9150_init(){
	_i2c_addr = ADDR1;
	_i2c_write(MPU9150_PWR_MGMT_1, 1 << 0);
	// Configure gyro to +- 250 deg / sec
	uint8_t temp = _i2c_read(MPU9150_GYRO_CONFIG);
	_i2c_write(MPU9150_GYRO_CONFIG, temp & ~(1 << 3 | 1 << 4));
	
	// configure acc to +- 2g
	temp = _i2c_read(MPU9150_ACCEL_CONFIG);
	_i2c_write(MPU9150_ACCEL_CONFIG, temp & ~(1 << 3 | 1 << 4));

	// enable mag passthrough
	_i2c_write(MPU9150_I2C_INT_PIN_CFG, 1 << 1);

	// configure mag
	_i2c_addr = ADDR2;
	_i2c_write(MPU9150_MAG_CNTL, 1 << 0);
	_i2c_addr = ADDR1;

	// configure i2c master
	_i2c_write(MPU9150_I2C_MST_CTRL, 1 << 4 | 8);

	// configure i2c slave 0
	_i2c_write(MPU9150_SLV0_ADDR, ADDR2 | 1 << 7);
	_i2c_write(MPU9150_SLV0_REG, 0x03);
	_i2c_write(MPU9150_SLV0_CTRL, 6 | 1 << 7);

	// configure interrupt enable
	_i2c_write(MPU9150_INT_ENABLE, 1);

	// Configure controls
	_i2c_write(MPU9150_USER_CTRL, 1 << 6 | 1 << 5);
}
