#ifndef DS18B20_H
#define	DS18B20_H
	#include <stdint.h>
	#include <stdbool.h>


    #define DS18B20_FAMILY_ID			0x28
    #define DS18B20_TEMP_RES_mask		0x60        // Masks bit 5 and 6 to get the temperature resolution
    #define DS18B20_TIMEOUT				100
    #define DS18B20_MAX_RETRIES			5          // Maximum number of retries before an reset or other failsafe must activate
    #define DS18B20_ERROR_TEMP			-256.0
    #define DS18B20_TIMEOUT_MS			1200
	#define DS18B20_ROM_LENGHT			8			// 8 bytes, thus 64 bit lenght
	#define DS18B20_SCRATCHPAD_LENGHT	9

    typedef enum
    {
		DS_SUCCESFULL,          //  0
        DS_INVALID_PARAM,		//  1
        OWI_PULL_UP_LOW_ERROR,  //  2
        OWI_PULL_UP_HIGH_ERROR, //  3
        DS_NODEVICE,            //  4
        DS_ROM_CRC_NOK,         //  5
        DS_CRC_NOK,             //  6
        DS_TIMEOUT,             //  7
		DS_UNKNOWN_ERROR		//  8
    }DS_ERROR_CODES;

    typedef enum
    {
        SOURCE_VDD       = 0,   //  The DS18B20 is powered from the VDD connection. 
        SOURCE_PARASITIC = 1,   //  The DS18B20 is powered by the data line
    }POWER_SUPPLY;


    /* List of all ROM and Function command for the DS18B20 */
    typedef enum 
    {
        /* ROM commands */
        READ_ROM			= 0x33,
        MATCH_ROM			= 0x55,
        SKIP_ROM			= 0xCC,
        SEARCH_ROM			= 0xF0,

        /* Function commands */
        ALARM_SEARCH		= 0xEC,
        WRITE_SCRATCHPAD	= 0x4E,
        READ_SCRATCHPAD		= 0xBE,
        COPY_SCRATCHPAD		= 0x48,
        CONVERT_TEMP		= 0x44,
        RECAL_EE			= 0xB8,
        READ_PWR_SUPPLY		= 0xB4           
    }DS_COMMANDS;


    typedef enum
    {
		TEMP_RESOLUTION_Bitmask	 = 0x60,	// 0110 0000
        TEMP_9BIT_RESOLUTION     = 0x1F,	// 0001 1111
        TEMP_10BIT_RESOLUTION    = 0x3F,	// 0011 1111
        TEMP_11BIT_RESOLUTION    = 0x5F,	// 0101 1111
        TEMP_12BIT_RESOLUTION    = 0x7F,	// 0111 1111
    }DS_RESOLUTION;



	void DS18B20_clearSearch	(void);
	bool DS18B20_Search			(uint8_t * ROM);
    bool DS18B20_ReadROM		(uint8_t * ROM, uint8_t * data);

    float			DS18B20_CalculateTemperature(uint16_t regVal);
    DS_ERROR_CODES	DS18B20_RecalEeprom			(uint8_t * ROM);
    DS_ERROR_CODES	DS18B20_CopyScratchPad		(uint8_t * ROM);
    DS_ERROR_CODES	DS18B20_StartTempConversion	(uint8_t * ROM);
    DS_ERROR_CODES	DS18B20_ReadScratchPad		(uint8_t * ROM, uint8_t * data);
    DS_ERROR_CODES	DS18B20_WriteScratchPad		(uint8_t * ROM, uint8_t Thigh, uint8_t Tlow, uint8_t settings);
	DS_ERROR_CODES	DS18B20_Init				(void);
    void			TestTempCalc				(void);

#endif	/* DS18B20_H */

