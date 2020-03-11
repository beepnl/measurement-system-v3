#define NRF_LOG_MODULE_NAME TLV
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "TLV320ADC3100.h"
#include "I2C.h"
#include "I2S.h"
#include "power_app.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_util.h"
#include "audioFFT.h"
#include "audio_app.h"

/*
		AIN_IN3LM	= 0,
		AIN_IN2LP	= 1,
		AIN_IN2RP	= 2,
		AIN_OFF		= 0xFF,
*/

TLVs tlv = 
{
	.currentPage	= UINT8_MAX,	// default invalid value
	.fs				= 4E3,
	.mclk_freq		= MCLK_FREQ_HZ, /// NRF_I2S_MCK_32MDIV31, 32 MHz / 31 = 1.0322581 MHz.
	.NADC			= 24,
	.MADC			= 4,
	.AOSR			= 128,
	.P				= 1,		// 1-8
	.R				= 3,		// 1-16
	.J				= 32,		// 1-63
	.D				= 0,		// 0 -9999
	.decRation		= 1,		// 1 - 16
	.BLCK_offset	= 0,
	.rightInput		= AIN_IN2RP,
    .leftInput		= AIN_OFF,
	.processingBlock= 1,
};


static void TLV_sw_reset(void)
{
    I2C_write(TLV_ADDRESS, 1, 1);
}










static uint32_t TLV_calc_fs(uint32_t PLL, uint32_t NADC, uint32_t MADC, uint32_t AOSR, uint8_t P, uint8_t R, uint16_t J, uint16_t D)
{
	uint32_t fs;
	uint32_t K;

	// First calculate the K parameter from J and D, then calculate the sample frequency
	K   = J; // decimal point D removed due to uint32_t limits 
	fs  = ((PLL * K * R) / (NADC * MADC * AOSR * P));

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("Fs=%u kHz, PLL=%u Hz, P=%u, R=%u, K:J.D=%u.%04u", fs, PLL, P, R, J, D);
        NRF_LOG_FLUSH();
    #endif
}


static void TLV_calcK(uint32_t fs, uint32_t PLL, uint32_t NADC, uint32_t MADC, uint32_t AOSR, uint8_t * P, uint8_t * R, uint16_t * J, uint16_t * D)
{
	if(P == NULL || R == NULL || J == NULL || D == NULL)
	{
		return;
	}
	uint32_t K;

	K = (fs * NADC * MADC * AOSR * (*P)) / ((PLL / 10000) * (*R));

	*J = K / 10000;
	*D = K % 10000;

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("Fs=%u kHz, PLL=%u Hz, K= %u, J=%u, D=%u", fs, PLL/1E6, K, *J, *D);
        NRF_LOG_FLUSH();
    #endif
}



uint8_t TLV_readPageSet(void)
{
	uint8_t regVal = 0;
	I2C_read(TLV_ADDRESS, 0, &regVal);
	tlv.currentPage = regVal;
	return regVal;
}


void TLV_writePageSet(uint8_t page)
{
	if((page != 0) && (page != 1) && (page != 4) && (page != 5) && !(page >= 32 && page <= 47))
	{
		NRF_LOG_INFO("Incorrect TLV Page number: %u, write aborted", page);
		return;
	}

	// Write the new page number and check whether the value is correctly written
	if(I2C_write(TLV_ADDRESS, 0, page) == NRF_SUCCESS)
	{
		tlv.currentPage = page;
	}
}

static uint32_t TLV_get_fs(uint32_t PLL)
{
	bool pllOn = false;
	bool NADCon = false;
	bool MADCon = false;
	uint32_t fs, NADC, MADC, IADC, AOSR, P, R, J, D, decimation, ADC_CLKIN;
	uint32_t K;
	uint8_t reg, val;

	NRF_LOG_FLUSH();

    if(tlv.currentPage != 0)
	{
		TLV_writePageSet(0);
	}

	// Clock generation
    I2C_read(TLV_ADDRESS, 4, &reg);

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("PLL CLKin=%u, Codec CLKin=%u", (reg & 0x0C) >> 2, (reg & 0x03));
        NRF_LOG_FLUSH();
    #endif

	// Reaad the P and R values
	I2C_read(TLV_ADDRESS, 5, &reg);
    pllOn = (reg & 0x80 ? true : false);
    val = (reg & 0x70) >> 4;
	P = (val == 0 ? 8 : val);
    val = (reg & 0x0F);
	R = (val == 0 ? 16 : val);

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("PLL enable=%u, P=%u, R=%u", pllOn, P, R);
        NRF_LOG_FLUSH();
    #endif

	// Read the J and D values
    I2C_read(TLV_ADDRESS, 6, &reg);
	J =  reg & 0x3F;
    I2C_read(TLV_ADDRESS, 7, &reg);
	D =  (reg & 0x3F) << 8;
    I2C_read(TLV_ADDRESS, 7, &reg);
    D |= reg;

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("J=%u, D=%u, K=%u.%u", J, D, J, D);
        NRF_LOG_FLUSH();
    #endif

	// Read the NADC, MADC and AOSR value.
	I2C_read(TLV_ADDRESS, 18, &reg);
    NADCon = (reg & 0x80 ? true : false);
	val = reg & 0x7F;
	NADC = (val == 0 ? 128 : val);

    I2C_read(TLV_ADDRESS, 19, &reg);
    MADCon = (reg & 0x80 ? true : false);
    val = reg & 0x7F;
    MADC = (val == 0 ? 128 : val);

    I2C_read(TLV_ADDRESS, 20, &reg);
    AOSR = (reg == 0 ? 256 : reg);
    I2C_read(TLV_ADDRESS, 21, &reg);
    IADC = reg * 2;

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("NADC: En=%u = %u, MADC: En=%u = %u, IADC=%u, AOSR=%u", NADCon, NADC, MADCon, MADC, IADC, AOSR);
        NRF_LOG_FLUSH();
    #endif

	// First calculate the K parameter from J and D, then calculate the sample frequency
	K = (J * 10000) + D;
	fs = ((PLL / 10000) * K * R) / (NADC * MADC * AOSR * P);

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("Fs=%u Hz, PLL=%u Hz, P=%u, R=%u, K:J.D=%u.%04u", fs, PLL/1E6, P, R, J, D);
        NRF_LOG_FLUSH();
    #endif

	uint32_t condition1, condition2;
	if(D == 0)
	{
		condition1 = PLL / P;
		condition2 = (PLL / (P * 10000)) * K * R;

        #if TLV_LOGGING_ENABLED
            NRF_LOG_INFO("512kHz <= %uHz <= 20MHz:\t\t%s", condition1, (condition1 >= 512E3 && condition1 <= 20E6) ? "True":"False");
            NRF_LOG_INFO("80MHz <= %uHz <= 110MHz:\t\t%s", condition2, (condition2 >= 80E6 && condition2 <= 110E6) ? "True":"False");
            NRF_LOG_INFO("4 <= J:%u <= 55:\t\t\t\t%s", J, (J>= 4 && J <= 55) ? "True":"False");
            NRF_LOG_FLUSH();
        #endif
	}
	else
	{
        #if TLV_LOGGING_ENABLED
            NRF_LOG_INFO("Invalid configuration, PLL cannot be used");
        #endif
	}

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("MADC(%u) x AOSR(%u) = %u >= IADC(%u): \t%s", MADC, AOSR, (MADC * AOSR), IADC, ((MADC * AOSR)>= IADC) ? "True":"False");
        NRF_LOG_FLUSH();
    #endif

    I2C_read(TLV_ADDRESS, 22, &reg);
    decimation = (reg == 0 ? 16 : reg);

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("Decimation=%u", decimation);
        NRF_LOG_FLUSH();
    #endif

	ADC_CLKIN = NADC * MADC * AOSR * fs;

    #if TLV_LOGGING_ENABLED
        NRF_LOG_INFO("ADC_CLKIN=%u", ADC_CLKIN);
        NRF_LOG_FLUSH();
    #endif
}

static uint8_t TLV_readADCflags(void)
{
	uint8_t retval = 0;
	if(tlv.currentPage != 0)
	{
		TLV_writePageSet(0);
	}

    I2C_read(TLV_ADDRESS, 36, &retval);

    #if 1 //TLV_LOGGING_ENABLED
        NRF_LOG_INFO("Page 0, reg=36, ADC flags= 0x%02X", retval);
        NRF_LOG_FLUSH();
        NRF_LOG_INFO("\tL-ADC gain ok: %s, powered: %s, saturated: %s", 
						bool_to_str(retval & (1<<7)),
						bool_to_str(retval & (1<<6)),
						bool_to_str(retval & (1<<5)));
		NRF_LOG_FLUSH();
		NRF_LOG_INFO("\tR-ADC gain ok: %s, powered: %s, saturated: %s", 
				bool_to_str(retval & (1<<3)),
				bool_to_str(retval & (1<<2)),
				bool_to_str(retval & (1<<1)));
        NRF_LOG_FLUSH();
    #endif
}



static uint8_t TLV_readSettings(void)
{
	uint8_t retval = 0;
	if(tlv.currentPage != 1)
	{
		TLV_writePageSet(1);
	}

    I2C_read(TLV_ADDRESS, 51, &retval);
    NRF_LOG_INFO("p1, reg=51, MIC bias = 0x%02X", retval);
    NRF_LOG_FLUSH();
	nrf_delay_ms(100);

    I2C_read(TLV_ADDRESS, 52, &retval);
    NRF_LOG_INFO("p1, reg=52, Left ADC input = 0x%02X", retval);
    NRF_LOG_FLUSH();
	nrf_delay_ms(100);

    I2C_read(TLV_ADDRESS, 54, &retval);
    NRF_LOG_INFO("p1, reg=54, Left ADC input = 0x%02X", retval);
    NRF_LOG_FLUSH();
	nrf_delay_ms(100);


    I2C_read(TLV_ADDRESS, 55, &retval);
    NRF_LOG_INFO("p1, reg=55, Right ADC input = 0x%02X", retval);
    NRF_LOG_FLUSH();
	nrf_delay_ms(100);

    I2C_read(TLV_ADDRESS, 57, &retval);
    NRF_LOG_INFO("p1, reg=57, Right ADC input = 0x%02X", retval);
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);

    I2C_read(TLV_ADDRESS, 59, &retval);
    NRF_LOG_INFO("p1, reg=59, Left Analog PGA = 0x%02X, L-PGA muted: %u, volume %u.%u", retval, retval & (1<<7) ? 1 : 0, ((retval & 0x7F) / 2),
		(((retval & 0x7F) % 2) == 0 ? 0 : 5));
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);

    I2C_read(TLV_ADDRESS, 60, &retval);
    NRF_LOG_INFO("p1, reg=60, Right Analog PGA = 0x%02X, R-PGA muted: %u, volume %u.%u", retval, retval & (1<<7) ? 1 : 0, ((retval & 0x7F) / 2), (((retval & 0x7F) % 2) == 0 ? 0 : 5));
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);

	TLV_writePageSet(0);	

    I2C_read(TLV_ADDRESS, 81, &retval);
    NRF_LOG_INFO("p0, reg=81, ADC Digital = 0x%02X, L-ADC pwr: %u, R-ADC pwr: %u, soft step: 0x%X", retval, retval & (1<<7) ? 1: 0, retval & (1<<6) ? 1 : 0, retval & 0x03);
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);

    I2C_read(TLV_ADDRESS, 82, &retval);
    NRF_LOG_INFO("p0, reg=82, ADC volume control = 0x%02X, L-ADC muted: %u, R-ADC muted: %u", retval, retval & (1<<7), retval & (1<<3));
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);

    I2C_read(TLV_ADDRESS, 61, &retval);
    NRF_LOG_INFO("p0, reg=61, Processing block = %ud/0x%02X", retval, retval);
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);
}

static uint8_t TLV_readPGAflags(void)
{
	uint8_t retval = 0;
	if(tlv.currentPage != 1)
	{
		TLV_writePageSet(1);
	}

    I2C_read(TLV_ADDRESS, 62, &retval);

    #if 1 //TLV_LOGGING_ENABLED
        NRF_LOG_INFO("Page 1, reg=62, 0x%02X, L_ADC_PGA gain ok: %s, R_ADC_PGA gain ok: %s",
					retval,
                    bool_to_str(retval & (1<<1)),
                    bool_to_str(retval & (1<<0)));
        NRF_LOG_FLUSH();
    #endif
}

bool TLV_Appnote_init(AUDIO_INPUTe channel, int8_t volumeControl, uint8_t gainCoarse, bool min6dB)
{
	uint8_t regval;

		powerApp_Enable(true, PWR_AUDIO);
	nrf_delay_ms(1);

	// Calculate the coefficients for the given sample rate and mclk frequency
	#if 0
		tlv.fs = 4E6 / 96;
		TLV_calcK(tlv.fs, tlv.mclk_freq, tlv.NADC, tlv.MADC, tlv.AOSR, &tlv.P, &tlv.R, &tlv.J, &tlv.D);
	#else
		tlv.fs = TLV_calc_fs(tlv.mclk_freq, tlv.NADC, tlv.MADC, tlv.AOSR, tlv.P, tlv.R, tlv.J, tlv.D);
	#endif

	// Set the device to page 0 for configuration
	TLV_writePageSet(0);

    TLV_sw_reset();

		// PLL_CLKIN = MCLK, CODEC_CLKIN = PPL_CLK
		I2C_write(TLV_ADDRESS, 4, 0x03);

        // Reg 5/0x05: 7=PLL enable, [6:4]=PLL DIV P, [3:0]=PLL MULT R
        regval = 0x80 | ((tlv.P << 4) & 0x70) | (tlv.R & 0x0F);
		I2C_write(TLV_ADDRESS, 5, regval);

		// Reg 6/0x06: PPL J val
        regval = tlv.J & 0x3F;
        I2C_write(TLV_ADDRESS, 6, tlv.J & 0x3F);

		// Reg 7/0x07 & 8/0x08: PPL D-val
        I2C_write(TLV_ADDRESS, 7, (uint8_t)(tlv.D >> 8) & 0x3F);
        I2C_write(TLV_ADDRESS, 8, (uint8_t)(tlv.D >> 0));

		// Reg 18/0x12: ADC NADC Clock divider
        regval = 0x80 | ((tlv.NADC >= 128) ? 0 : tlv.NADC);
        I2C_write(TLV_ADDRESS, 18,  regval);

        // Reg 19/0x13: ADC MADC Clock divider
        regval = 0x80 | ((tlv.MADC >= 128) ? 0 : tlv.MADC);
        I2C_write(TLV_ADDRESS, 19,  regval);

        // Reg 20/0x14: ADC AOSR
        regval = ((tlv.AOSR >= 256) ? 0 : tlv.AOSR);
        I2C_write(TLV_ADDRESS, 20,  regval);
        
        // Reg 22/0x16: decimation ratio
		#if 1
			regval = ((tlv.decRation >= 16) ? 0 : tlv.decRation);
			I2C_write(TLV_ADDRESS, 22,  regval & 0x0F);
		#endif

        // Reg 27/0x1B: ADC Audio Interface Control: I2S, 16 bits, BCLK is input, WCLK is input, Tri-state of DOUT disabled
        I2C_write(TLV_ADDRESS, 27,  0x00);
        
		#if 0
			// Reg 30/0x1E: BLCK offset
			regval = (((tlv.BLCKnDiv == 0) ? 0 : 1) << 7) | ((tlv.BLCKnDiv >= 128) ? 0 : (tlv.BLCKnDiv & 0x7F));
			I2C_write(TLV_ADDRESS, 30,  regval);
		#endif

		// Set processing block
		if(tlv.processingBlock >= 19)
		{
			tlv.processingBlock = 0;
		}
		I2C_write(TLV_ADDRESS, 61,  tlv.processingBlock);

		const uint8_t processingBlockINSTR [19] = { 2, 188, 240, 236, 96, 120, 120, 88, 120, 128, 46, 60, 64, 70, 124, 120, 36, 64, 62};

        tlv.IADC = processingBlockINSTR[tlv.processingBlock];

        // Reg 21/0x15: IADC AOSR. Divide by 2 to get the register value, all values above 384 are 0xFF and have the value of 510
        regval = ((tlv.IADC > 384) ? UINT8_MAX : (tlv.IADC >> 1));
        I2C_write(TLV_ADDRESS, 21,  regval);

		// Read back the set clock configurations
        TLV_get_fs(MCLK_FREQ_HZ);

	TLV_writePageSet(1);

		// MIC bias
		I2C_write(TLV_ADDRESS, 51,  0x20);

		// Left Analog PGA Setting = 0dB
        I2C_write(TLV_ADDRESS, 59,  0x00);

        // Right Analog PGA Setting = 0dB
        I2C_write(TLV_ADDRESS, 60,  0x00);

		// Analog routing Left
		#if 0
			I2C_write(TLV_ADDRESS, 52,  0xF3); // IN2LP 0-dB
		#else
			I2C_write(TLV_ADDRESS, 52,  0xCF); // IN3LM 0-dB
		#endif
        I2C_write(TLV_ADDRESS, 54,  0x3F); // Do not by-pass left PGA, no bias unused channels

		// Right 
        I2C_write(TLV_ADDRESS, 55,  0xF3);
        I2C_write(TLV_ADDRESS, 57,  0x3F);

	TLV_writePageSet(0);	
	  
		// Power up left and right ADC's 81 = 0x51
		I2C_write(TLV_ADDRESS, 0x51,  0xC2);

		// unmute ADC' 82
        I2C_write(TLV_ADDRESS, 0x52,  0x00);

    #if TLV_LOGGING_ENABLED
        nrf_delay_ms(100);
        TLV_readADCflags();
        TLV_readPGAflags();
        TLV_readSettings();
    #endif

	return true;
	// Sample the TLV with the I2S interface
//	I2S_checkRX();
}






bool TLV_init(AUDIO_INPUTe channel, int8_t volumeControl, uint8_t gainCoarse, bool min6dB)
{
	uint8_t regval, readval;
    NRF_LOG_FLUSH();

    switch(channel)
    {
        case AIN_IN3LM:      // 0, Left-PGA
            tlv.leftInput   = AIN_IN3LM;
			tlv.rightInput  = AIN_OFF;
            break;

        case AIN_IN2LP:      // 1, Left-PGA
            tlv.leftInput   = AIN_IN2LP;
			tlv.rightInput  = AIN_OFF;
            break;

        case AIN_IN2RP:      // 2, default  Right-PGA
            tlv.leftInput   = AIN_OFF;
			tlv.rightInput  = AIN_IN2RP;
            break;

        default:
            return false;
            break;
    }

    // Limit the gainCoarse to between 0.0 dB - +40.0 dB
    if(gainCoarse > 80.0)
    {
        gainCoarse = 80.0;
    }

    // Limit the gainCoarse to between -12.0 dB - +20.0 dB in 0.5dB/digit
    if(volumeControl < -24)
    {
        volumeControl = -24;
    }
    else if(volumeControl > 40)
    {
        volumeControl = 40;
    }

	powerApp_Enable(true, PWR_AUDIO);
	nrf_delay_ms(1);

	// Calculate the coefficients for the given sample rate and mclk frequency
	#if 0
		tlv.fs = 4E6 / 96;
		TLV_calcK(tlv.fs, tlv.mclk_freq, tlv.NADC, tlv.MADC, tlv.AOSR, &tlv.P, &tlv.R, &tlv.J, &tlv.D);
	#else
		tlv.fs = TLV_calc_fs(tlv.mclk_freq, tlv.NADC, tlv.MADC, tlv.AOSR, tlv.P, tlv.R, tlv.J, tlv.D);
	#endif

	// Set the device to page 0 for configuration
	TLV_writePageSet(0);

    TLV_sw_reset();

		// PLL_CLKIN = MCLK, CODEC_CLKIN = PPL_CLK
		I2C_write(TLV_ADDRESS, 4, 0x03);

        // Reg 5/0x05: 7=PLL enable, [6:4]=PLL DIV P, [3:0]=PLL MULT R
        regval = 0x80 | ((tlv.P << 4) & 0x70) | (tlv.R & 0x0F);
		I2C_write(TLV_ADDRESS, 5, regval);

		// Reg 6/0x06: PPL J val
        I2C_write(TLV_ADDRESS, 6, tlv.J & 0x3F);

		// Reg 7/0x07 & 8/0x08: PPL D-val
        I2C_write(TLV_ADDRESS, 7, (uint8_t)(tlv.D >> 8) & 0x3F);
        I2C_write(TLV_ADDRESS, 8, (uint8_t)(tlv.D >> 0));

		// Reg 18/0x12: ADC NADC Clock divider
        regval = 0x80 | ((tlv.NADC >= 128) ? 0 : tlv.NADC);
        I2C_write(TLV_ADDRESS, 18,  regval);

        // Reg 19/0x13: ADC MADC Clock divider
        regval = 0x80 | ((tlv.MADC >= 128) ? 0 : tlv.MADC);
        I2C_write(TLV_ADDRESS, 19,  regval);

        // Reg 20/0x14: ADC AOSR
        regval = ((tlv.AOSR >= 256) ? 0 : tlv.AOSR);
        I2C_write(TLV_ADDRESS, 20,  regval);
        
        // Reg 22/0x16: decimation ratio
		#if 1
			regval = ((tlv.decRation >= 16) ? 0 : tlv.decRation);
			I2C_write(TLV_ADDRESS, 22,  regval & 0x0F);
		#endif

        // Reg 27/0x1B: ADC Audio Interface Control: I2S, 16 bits, BCLK is input, WCLK is input, Tri-state of DOUT disabled
        I2C_write(TLV_ADDRESS, 27,  0x00);
        
		#if 0
			// Reg 30/0x1E: BLCK offset
			regval = (((tlv.BLCKnDiv == 0) ? 0 : 1) << 7) | ((tlv.BLCKnDiv >= 128) ? 0 : (tlv.BLCKnDiv & 0x7F));
			I2C_write(TLV_ADDRESS, 30,  regval);
		#endif

		// Set processing block
		if(tlv.processingBlock >= 19)
		{
			tlv.processingBlock = 0;
		}
		I2C_write(TLV_ADDRESS, 61,  tlv.processingBlock);

		const uint8_t processingBlockINSTR [19] = { 2, 188, 240, 236, 96, 120, 120, 88, 120, 128, 46, 60, 64, 70, 124, 120, 36, 64, 62};

        tlv.IADC = processingBlockINSTR[tlv.processingBlock];

        // Reg 21/0x15: IADC AOSR. Divide by 2 to get the register value, all values above 384 are 0xFF and have the value of 510
        regval = ((tlv.IADC > 384) ? UINT8_MAX : (tlv.IADC >> 1));
        I2C_write(TLV_ADDRESS, 21,  regval);

		// Read back the set clock configurations
        TLV_get_fs(MCLK_FREQ_HZ);

	// Set the device to page 1 for configuration
	TLV_writePageSet(1);
        
		// Reg 51/0x33: MICBias [6:5] = 0x20 (0b10) MICBIAS1 is 2.5V / 0x10 (0b01) MICBIAS1 is 2.0V
        I2C_write(TLV_ADDRESS, 51,  0x20);

        /* Reg 52/0x34: Left ADC input selection for left PGA
		 * 1100 1111 = LCH_SEL3 (IN3L(M) single-ended selected with 0-dB setting)
		 */
		if(tlv.leftInput == AIN_IN3LM)
		{
			regval = 0xCF | ((min6dB) ? (1<<4) : (0<<4) ); // 1100 1111
		}
		else if(tlv.leftInput == AIN_IN2LP)
		{
			regval = 0xF3 | ((min6dB) ? (1<<2) : (0<<2) ); // 1111 0011
		}
		else
		{
			regval = 0xFF; // Inputs disconnected from Left-PGA
		}
        I2C_write(TLV_ADDRESS, 52,  regval);

		// Reg 54/0x36: Left ADC input selection for Left ADC default value 0x3F=0b00111111
        I2C_write(TLV_ADDRESS, 54,  0x3F);


        // Reg 55/0x37: Right ADC input selection for Right ADC default value 0x57=0b0101 0111
		if(tlv.rightInput == AIN_IN2RP)
		{
			regval = 0xF3 | ((min6dB) ? (1<<2) : (0<<2) ); // 1111 0011
		}
		else
		{
			regval = 0xFF; // Inputs disconnected from Right-PGA
		}
        I2C_write(TLV_ADDRESS, 55,  regval);

        // Reg 57/0x39: Right ADC input selection for Right ADC default value 0x17=0b0001 0111
        I2C_write(TLV_ADDRESS, 57,  0x3F);

        // Reg 59/0x03B: Left Analog PGA settings
		regval = (gainCoarse & 0x7F);
        I2C_write(TLV_ADDRESS, 59,  regval);

        // Reg 60/0x03C: Right Analog PGA settings
		regval = (gainCoarse & 0x7F);
        I2C_write(TLV_ADDRESS, 60,  regval);

	TLV_writePageSet(0);

		// Power up ADC channel
        regval =	((tlv.leftInput  == AIN_OFF) ? 0 : (1<<7))	|
					((tlv.rightInput == AIN_OFF) ? 0 : (1<<6))	|
					0x02;
		I2C_write(TLV_ADDRESS, 81,  regval);

		// set gain 0dB for both channels. Unmute or mute when the channels are active or not.
        regval =	((tlv.leftInput  == AIN_OFF) ? (1<<7) : 0)	|
					((tlv.rightInput == AIN_OFF) ? (1<<3) : 0);

        I2C_write(TLV_ADDRESS, 82,  regval);

        // Set the volume control
        I2C_write(TLV_ADDRESS, 83,  volumeControl & 0x7F);
        I2C_write(TLV_ADDRESS, 84,  volumeControl & 0x7F);

    #if 0 //TLV_LOGGING_ENABLED
        TLV_readADCflags();
        TLV_readPGAflags();
        TLV_readSettings();
    #else
        nrf_delay_ms(100);
    #endif
        
	
    #if 0 // implementation testing
        // Sample the TLV with the I2S interface
        I2S_checkRX();
    #endif
    return true;
}











