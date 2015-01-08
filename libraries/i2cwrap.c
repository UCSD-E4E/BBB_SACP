#include <stdint.h>
#include <i2cmaster.h>
#include <i2cwrap.h>

void twi_init(void){
	i2c_init();
}

int twi_write(uint8_t addr, uint8_t dest, uint8_t val){
	if(i2c_start(addr << 1)){
		return 1;
	}
	if(i2c_write(dest)){
		return 1;
	}
	if(i2c_write(val)){
		return 1;
	}
	i2c_stop();
	return 0;
}

int twi_read_byte(uint8_t addr, uint8_t reg, uint8_t* target){
	i2c_start_wait(addr << 1);
	if(i2c_write(reg)){
		return 1;
	}
	if(i2c_rep_start((addr << 1) + 1)){
		return 1;
	}
	*target = i2c_readNak();
	i2c_stop();
	return 0;
}
