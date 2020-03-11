#include "DS18B20.h"
#include "OWI.h"
#define NRF_LOG_MODULE_NAME DS18B20
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "utilities.h"



/*
*****************************************************************************************
* Description	: This function calculates the CRC on the received data.
*
* Arguments		: *DS_bytes pointer to the arry that holds the DS bytes
*
* Return		: Returns the calculated checksum. Caller must check whether it's correct.
*****************************************************************************************
*/
uint8_t DS18B20_CalcCRC(uint8_t * DS_bytes, uint8_t N)
{
	uint8_t DS_current_byte = 0x00;
	uint8_t DS_crc          = 0x00;
	uint8_t DS_crc_carry    = 0x00;
	uint8_t i               = 0x00;
	uint8_t j               = 0x00;
    
    // Check if the number of bytes is non-zero
    if(N == 0){
        return 0x00;
    }

	/* Loop for all 7 DS bytes */
	for (i = 0 ; i < N ; i++)
	{
		DS_current_byte = DS_bytes[i];    // Get  byte

		/* Calculate CRC for all bits */
		for (j = 0 ; j < 8 ; j++)             
		{
			/* XOR bit 0 with bit 0 of current CRC */
			if ( (DS_crc & 0x01) != (DS_current_byte & 0x01) )
			{
				DS_crc_carry = 0x80;
			}
			else
			{
				DS_crc_carry = 0x00;
			}

			/* Shift CRC */
			DS_crc >>= 1;     

			/* Check for carry */
			if (DS_crc_carry == 0x80)
			{
				DS_crc |= DS_crc_carry;
				DS_crc ^= 0x0C;  
			}

			DS_current_byte >>= 1;
		}
	}
    
	return DS_crc;  // Return the calculated checksum byte.
} /* DS_CalcCRC */


float DS18B20_CalculateTemperature (uint16_t regVal)
{
	return ((float)((int16_t)regVal)) / 16.0;
}


uint8_t ROM_NO[8]				= {0};
uint8_t LastDiscrepancy			= 0;
uint8_t LastFamilyDiscrepancy	= 0;
bool LastDeviceFlag				= false;


void DS18B20_clearSearch(void)
{
	
	LastDiscrepancy			= 0;
	LastFamilyDiscrepancy	= 0;
	LastDeviceFlag			= false;
	memset(ROM_NO, 0, 8);
}

bool DS18B20_Search(uint8_t * ROM)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	bool    search_result;
	uint8_t id_bit, cmp_id_bit;

	uint8_t rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;

	// if the last call was not the last one
	if (!LastDeviceFlag)
	{
		// 1-Wire reset
		if (!OWI_presenceDetect()) 
		{
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0;
			NRF_LOG_INFO("No One-Wire devices presence detected");
			return false;
		}

		// issue the search command
		OWI_write(0xF0);   // NORMAL SEARCH

		// loop to do the search
		do
		{
			// read a bit and its complement
			id_bit		= OWI_readBit();
			cmp_id_bit	= OWI_readBit();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1)) 
			{
				break;
			}
			else
			{
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit)
				{
					search_direction = id_bit;  // bit write value for search
				}
				else
				{
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy)
					{
						search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					}
					else
					{
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);
					}
					// if 0 was picked then record its position in LastZero
					if (search_direction == 0)
					{
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
						{
							LastFamilyDiscrepancy = last_zero;
						}
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
				{
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				}
				else
				{
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;
				}

				// serial number search direction write bit
				if(search_direction)
				{
					OWI_Write1();
				}
				else
				{
					OWI_Write0();
				}

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0)
				{
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		}
		while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65))
		{
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0)
			{
				LastDeviceFlag = true;
			}
			search_result = true;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0])
	{
		DS18B20_clearSearch();
		NRF_LOG_INFO("No more DS18B20 devices found");
	}
	else
	{
		printStringWithHexArray("DS18B20 found: ", 8, ROM_NO);
		if(ROM != NULL)
		{
			memcpy(ROM, ROM_NO, 8);
		}
	}
	return search_result;
}

void DS18B20_sendROM(uint8_t * ROM)
{
	// When no ROM code is given, skip it.
	if(ROM == NULL)
	{
		/* Send the skip ROM command to skip the ROM */
		OWI_write(SKIP_ROM);
	}
	else
	{
		OWI_write(MATCH_ROM);
		uint8_t i;
		for(i=0; i<DS18B20_ROM_LENGHT; i++)
		{
			OWI_write(ROM[i]);
		}
	}
}


/*
* \brief        Function to write data to the scratchpad to address location 2 through 4.
*               All three bytes must be written consecutively.      
* \arguments    *unsigend char pointer to the location of the 3 bytes to be written.
*/
DS_ERROR_CODES DS18B20_WriteScratchPad(uint8_t * ROM, uint8_t Thigh, uint8_t Tlow, uint8_t settings)
{
    uint8_t i;
        
    OWI_reset();
    
    if(OWI_presenceDetect() == false)
    {
        return DS_NODEVICE;
    }
    
    // Write the ROM code
    DS18B20_sendROM(ROM);
        
    /* Send command 0x4E to Write the scratchpad. */
    OWI_write(WRITE_SCRATCHPAD);

    /* Write 3 bytes to the DS18B20's scratch pad. */
	OWI_write(Thigh);
	OWI_write(Tlow);
	OWI_write(settings);

    nrf_delay_us(20);
    
    /* Finish communication with a TX reset */
    OWI_reset();
    return DS_SUCCESFULL;
} // DS18B20_ReadTemp


/*
*****************************************************************************************
* \brief        Function to write data to the scratchpad to address location 2 through 4.
*               All three bytes must be written consecutively.      
* \arguments    *unsigend char pointer to the location of the 3 bytes to be written.
*****************************************************************************************
*/
DS_ERROR_CODES DS18B20_CopyScratchPad(uint8_t * ROM)
{   
    unsigned char ConvertDone = 0;
    
    /* Reset the DS18B20 */
    OWI_reset();
    
    /* Check if a slave device can be detected. */
    if(!OWI_presenceDetect())
    {
        return DS_NODEVICE;
    }
    
    /* Send the ROM */
    DS18B20_sendROM(ROM);
        
    /* Send command 0x48 to Copy the scratchpad to E2 */
    OWI_write(COPY_SCRATCHPAD);
        
    // Wait for the DS18B20 to complete copying the scratchpad to E2.
    OWI_Power();

	while(ConvertDone == 0)
	{
		// Read a bit from the DS18B20
		nrf_delay_us(240); 
		OWI_Write1();
		ConvertDone = OWI_readByte();
	}
    
    /* Finish communication with a TX reset */
    OWI_reset();
    
    return DS_SUCCESFULL;
} // DS18B20_CopyScratchPad



DS_ERROR_CODES DS18B20_RecalEeprom(uint8_t * ROM)
{   
    unsigned char ConvertDone = 0;
    
    /* Reset the DS18B20 */
    OWI_reset();
    
    /* Check if a slave device can be detected. */
    if(!OWI_presenceDetect())
    {
        return DS_NODEVICE;
    }
    
    /* Send the ROM */
    DS18B20_sendROM(ROM);
        
    /* Send command 0x48 to Copy the scratchpad to E2 */
    OWI_write(RECAL_EE);
        
    // Wait for the DS18B20 to complete copying the scratchpad to E2.
    OWI_Power();

	while(ConvertDone == 0)
	{
		// Read a bit from the DS18B20
		nrf_delay_ms(1); 
		OWI_Write1();
		ConvertDone = OWI_readByte();
	}
    
    /* Finish communication with a TX reset */
    OWI_reset();
    
    return DS_SUCCESFULL;
} // DS18B20_RecalEeprom

/*
*****************************************************************************************
* \brief        Function to read the DS18B20
* \arguments    *DS_bytes pointer to the arry that holds the DS bytes
*****************************************************************************************
*/
bool DS18B20_ReadROM(uint8_t * ROM, uint8_t * data)
{
	uint8_t i, crc;

    /* Reset the DS18B20 */
    OWI_reset();
    
    /* Check if a slave device can be detected. */
    if(!OWI_presenceDetect())
    {
        return DS_NODEVICE;
    }

	// Transmit either the SKIP ROM command orthe match ROM command
    DS18B20_sendROM(ROM);
        
    /* Send command 0x33 to Read the ROM */
    OWI_write(READ_ROM);

    /* Read 8 bytes from the DS2401: 1 byte family code, 8 bytes unique ID & 1 byte CRC */
    for(i = 0 ; i < 8 ; i++)
    {
        data[i] = OWI_readByte();
    }
    
    // Calculate the CRC for the received data.
    crc = DS18B20_CalcCRC(data, 7);

	return (crc == data[7]) ? true : false; 
} /* DS18B20_ReadROM */



/*
 *
 */
DS_ERROR_CODES DS18B20_ReadScratchPad(uint8_t * ROM, uint8_t * data)
{
    uint8_t i, crc;
    static uint16_t regVal;
    
    // Send a reset to begin communication
    OWI_reset();
    
    // Try to detect a slave device. 
    if(!OWI_presenceDetect())
    {
        return DS_NODEVICE;
    }
    
    // Transmit either the SKIP ROM command orthe match ROM command
    DS18B20_sendROM(ROM);
       
    /* Send command 0x49 to Read the scratchpad. */
    OWI_write(READ_SCRATCHPAD);

    /* Read 9 bytes from the DS18B20's scratch pad. */
    for (i = 0 ; i < 9 ; i++)
    {
        data[i] = OWI_readByte();
    } 
    
    OWI_reset();    // Finish communication with the DS18B20.
    
    // Calculate the CRC for the received data.
    crc = DS18B20_CalcCRC(data, 8);
    
    //  Check if the CRC is correct
    if(crc != data[8])
    {
        return DS_CRC_NOK;
    }
 
        
    return DS_SUCCESFULL;
} // DS18B20_ReadScratchPad





//
///*
//*****************************************************************************************
//* \brief        Function to write data to the scratchpad to address location 2 through 4.
//*               All three bytes must be written consecutively.      
//* \arguments    *unsigend char pointer to the location of the 3 bytes to be written.
//*****************************************************************************************
//*/
//DS_ERROR_CODES DS18B20_ReadPowerSupply(sDS18B20 * DS)
//{       
//    /* Reset the DS18B20 */
//    OWI_reset();
//    
//    /* Check if a slave device can be detected. */
//    if(OWI_presenceDetect() == false)
//    {
//        return DS_NODEVICE;
//    }
//    
//    /* Send the skip ROM command to skip the ROM */
//    OWI_write(SKIP_ROM);
//        
//    /* Send command 0x48 to determine whether the power is derived from VDD or parasitic. */
//    OWI_write(READ_PWR_SUPPLY);
//    
//    // Wait for the DS18B20 to complete copying the scratchpad to E2.
//    nrf_delay_us(240); 
//    OWI_Write1();
//    
//    // If the bit read is 0 the supply source is parasitic, otherwise the source is VDD. 
//    if(OWI_readBit() == 0)
//    {
//        DS->supply = SOURCE_PARASITIC;  
//    }
//    else
//    {
//        DS->supply = SOURCE_VDD;  
//    }  
//    
//    /* Finish communication with a TX reset */
//    OWI_reset();
//    
//    return DS_SUCCESFULL;
//} // DS18B20_ReadPowerSupply



/*
*****************************************************************************************
* \brief        Function to write data to the scratchpad to address location 2 through 4.
*               All three bytes must be written consecutively.      
* \arguments    *unsigend char pointer to the location of the 3 bytes to be written.
*****************************************************************************************
*/
DS_ERROR_CODES DS18B20_StartTempConversion(uint8_t * ROM)
{      
    /* Reset the DS18B20 */
    OWI_reset();
    
    /* Check if a slave device can be detected. */
    if(OWI_presenceDetect() == false)
    {
        return DS_NODEVICE;
    }
    
    // Transmit either the SKIP ROM command orthe match ROM command
    DS18B20_sendROM(ROM);
           
    /* Send command 0x44 to start a Temperature conversion. */
    OWI_write(CONVERT_TEMP);
	OWI_Power();
            
    return DS_SUCCESFULL;
} // DS18B20_ReadPowerSupply


bool DS18B20_PollConversion (void)
{
    // Read a bit from the DS18B20
    nrf_delay_us(240); 
    OWI_Write1();
    
    if(OWI_readBit() == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}




void TestTempCalc(void)
{
	uint8_t i;
	uint16_t test[10] = 
	{
		0x07D0,    // +125.0
		0x0550,    // +85.0
		0x0191,    // +25.0625
		0x00A2,    // +10.125
		0x0008,    // +0.5
		0x0000,    // 0.0
		0xFFF8,    // -0.5
		0xFF5E,    // -10.125
		0xFE6F,    // -25.0625
		0xFC90,    // -55.0	
	};


	for(i=0; i<(sizeof(test)/sizeof(test[0])) ; i++)
	{
		NRF_LOG_INFO("TempTest[%u]: 0x%04X = "NRF_LOG_FLOAT_MARKER, 
			i,
			test[i],
			NRF_LOG_FLOAT(DS18B20_CalculateTemperature(test[i])));
		NRF_LOG_FLUSH();
	}
}


/*
*****************************************************************************************
* \brief        Function to initialize the DS18B20 to 8 bit resolution.    
* \arguments    sDS18B20 structure for storing data and settings.
*****************************************************************************************
*/
DS_ERROR_CODES DS18B20_Init(void)
{
	/* Set the OWI Pull-up HIGH or the DS2401 won't work. */
    OWI_High();
    
    return DS_SUCCESFULL;
}






