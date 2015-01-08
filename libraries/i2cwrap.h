/**
 * I2C Wrapper for i2cmaster.h.
 *
 * Author: Nathan Hui
 * Date: 12/28/14
 */

#ifndef _I2CWRAP_H
#define _I2CWRAP_H

#include <stdint.h>

/**
 * Initializes the I2C interface
 */
void twi_init(void);

/**
 * Writes a single byte to the specified destination at the given address
 * @param	uint8_t	addr	I2C bus address of address to write to
 * @param	uint8_t	dest	Device address to write to
 * @param	uint8_t val	Byte value to write to device
 * @return	0 if write successful, 1 if write failed.
 */
int twi_write(uint8_t addr, uint8_t dest, uint8_t val);

/**
 * Reads a single byte from the specified destination on the specified device
 * to the target address
 * @param	uint8_t	addr	I2C bus address of address to write to
 * @param	uint8_t reg	Device register to read from
 * @param	uint8_t*	target	Pointer to a uint8_t variable to store the
 * 								requested data
 * @return	0 if write successful, 1 if write failed.
 */
int twi_read_byte(uint8_t addr, uint8_t reg, uint8_t* target);

#endif	// _I2CWRAP_H