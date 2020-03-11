
#ifndef TLV320ADC3100
#define	TLV320ADC3100
	#include <stdint.h>
	#include <stdbool.h>
    #include "beep_types.h"
    #include "beep_protocol.h"

    #define TLV_LOGGING_ENABLED 0

	#define TLV_ADDRESS	0x18 // The TLV supports a general call on slave address 0x00
	// I2C Write = 0, Read = 1

	typedef struct
	{
		uint8_t			currentPage;
		uint32_t		fs;
		uint32_t		mclk_freq;
		uint8_t			NADC;
		uint8_t			MADC;
		uint8_t			AOSR;
		uint8_t			IADC;           // Multiply by two to get the decimal value
		uint8_t			P;              // 1-8
		uint8_t			R;              // 1-16
		uint16_t		J;              // 1-63
		uint16_t		D;              // 0-9999
		uint8_t			decRation;
		uint8_t			BLCK_offset;
		uint8_t			BLCKnDiv;       // 1-128
        AUDIO_INPUTe	rightInput;
        AUDIO_INPUTe	leftInput;
		uint8_t			processingBlock;
	}TLVs;

	bool TLV_init(AUDIO_INPUTe channel, int8_t volumeControl, uint8_t gainCoarse, bool min6dB);
    bool TLV_Appnote_init(AUDIO_INPUTe channel, int8_t volumeControl, uint8_t gainCoarse, bool min6dB);

#endif	/* TLV320ADC3100 */

