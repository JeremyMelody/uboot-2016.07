#include "myir_header.h"

#include <common.h>
#include <i2c.h>
#include <asm/arch/i2c.h>

#define DEBUG

#ifdef DEBUG
	#define dbg_msg(fmt, arg...)	printf("MYIR_HEADER: "fmt, #arg)
#endif
#define err_msg(fmt, arg...)	printf("MYIR_HEADER: "fmt, #arg)

unsigned char cal_checksum(const char *p, int count)
{
	unsigned char sum = 0;
	
	while (count--) {
		sum += *p++;
	}

	return sum;
}

/*
 * return 0 - successfully, others fail.
 */
int check_header(const myir_header_t *header)
{
	if (header->header != 0xFF55) {
		err_msg("Invalid header: 0x%04X\n", header->header);
		return -1;
	}
	
	if (cal_checksum((char *)header, sizeof(myir_header_t))) {
		printf("checksum failed!\n");
		return -2;
	}
	
	return 0;
}

/*
 * Get subtype
 */
const unsigned char *get_header_subtype(const myir_header_t *header)
{
	if (!header)
		return NULL;
	
	return header->subtype;
}

/*
 * Fill "rchksum" field.
 */
void make_header(myir_header_t *header)
{
	header->rchksum = 0x100 - cal_checksum((char *)header, sizeof(myir_header_t) - sizeof(header->rchksum));
}

/*
 * Get header from eeprom
 */
int get_header(myir_header_t *header)
{
	u32 i2c_base_old;
	
	if (!header)
		return -1;

	/* Set the i2c base to I2C_BASE1. Modified by Conway*/	

	if (i2c_probe(LCD_HEADER_I2C_ADDR)) {
		printf("Not found the LCD header IC\n");
		return -1;
	}

	if (i2c_read(LCD_HEADER_I2C_ADDR,
				 0,
				 2,
				 (unsigned char *)header,
				 sizeof(myir_header_t))) {
		printf("Failed to read lcd header\n");
		return -1;
	}

	/* Restore i2c base. Commented by Conway*/
//	set_i2c_base(i2c_base_old);
	
	if (check_header(header))
		return -1;

	return 0;
}
