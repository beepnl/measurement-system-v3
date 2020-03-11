
#ifndef OWI_H
#define	OWI_H
	#include <stdint.h>
	#include <stdbool.h>

	#define TIME_PERIOD		100
	#define TIME_A			10
	#define TIME_B			(TIME_PERIOD - TIME_A)
	#define TIME_C			(TIME_PERIOD - 2 * TIME_A)
	#define TIME_RESET		600

    void		OWI_reset				(void);
    bool		OWI_presenceDetect		(void);
	void		OWI_Power				(void);
    void		OWI_High				(void);
	void		OWI_Write0				(void);
	void		OWI_Write1				(void);
    uint32_t	OWI_readBit				(void);
    void		OWI_write				(uint8_t data);
    uint8_t		OWI_readByte			(void);

#endif	/* OWI_H */

