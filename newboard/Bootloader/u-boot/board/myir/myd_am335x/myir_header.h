#ifndef MYIR_HEADER_H
#define MYIR_HEADER_H

#define LCD_HEADER_I2C_ADDR 0x52 /* Address of the IC storage lcd header */

/* check all char field with 'strncmp' function */
typedef struct myir_header_s {
	unsigned short header;     /* Always '0xFF55' */
	unsigned char type[8];     /* "CM" - core module, "MB" - mather board, "DM" - display module */
	unsigned char subtype[14]; /* LCD for example: "4.3r", "7.0c", "7.0r" etc. */
	unsigned char revision[7]; /* "1.0", "1.0a", "1.1" .... */
	unsigned char rchksum;     /* Make sure sum of all bytes to be 0x00 */
} myir_header_t;

/*
 * return 0 - successfully, others fail.
 */
int check_header(const myir_header_t *header);

/*
 * Set header
 */
void set_heaher(const char *header);

/*
 * Check the header is valid or not
 */
int check_header(const myir_header_t *header);

/*
 * Get subtype
 */
const unsigned char *get_header_subtype(const myir_header_t *header);

/*
 * Get header from eeprom
 */
int get_header(myir_header_t *header);

#endif
