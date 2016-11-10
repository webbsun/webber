/*
    NXP LPC210x CSR1000/CSR1001 host SPI boot definitions file

    NOTE:  This code was "seeded" from WINARM and/or Olimex sources.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

//#ifndef CSR1K_HOST_BOOT_H
#define CSR1K_HOST_BOOT_H
 #include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>

/* Use of these typedefs might help port this to other platforms */
typedef unsigned int                uint32;
typedef int                         int32;
typedef unsigned short              uint16;
typedef short                       int16;
typedef unsigned char               uint8;
typedef char                        int8;

 /*
    LPC's PLL

   - Main clock F_OSC = 14.7456MHz on Olimex LPC-P2106-B board [limits: 10 - 25 MHz]
   - System should run at maximium frequency [limit: max 60 MHz]
   - So choose multiplier M = 4
     so CCLK = M * F_OSC= 4 * 14745600Hz = 58982400 Hz
   - MSEL-Bit in PLLCFG (bits 0-4) MSEL = M - 1
   - F_CCO must be inbetween the limits 156 MHz to 320 MHz
     datasheet: F_CCO = F_OSC * M * 2 * P
   - choose devider P=2 => F_CCO = 14745000Hz * 4 * 2 * 2
     = 235920000 ~=236 MHz
   - PSEL0 (Bit5 in PLLCFG) = 1, PSEL1 (Bit6) = 0 (0b01)
*/

#define F_OSC                       14745600
#define PLL_M                       4
#define CCLK                        (F_OSC * PLL_M)  /* CPU clock speed */

/*  VPB (V... Peripheral Bus)
 *  - choosen: VPB should run at full speed -> divider VPBDIV=1
 * => pclk = cclk = 59MHz
 */
#define PCLK                        CCLK

extern void timeDelayInUS(uint32 us_ticks);
#define timeDelayInMS(_d)       timeDelayInUS((_d) * 1000)

extern void CSR1Kboot(void);

//#endif  /* CSR1K_HOST_BOOT_H */



