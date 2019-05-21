#ifndef __flashRead_H
#define __flashRead_H

#ifdef __cplusplus
  extern "C" {
#endif
	 
#include "stm32f2xx_hal.h"
		
uint32_t flash_read(uint32_t address);	 
	 
	 
#ifdef __cplusplus
}
#endif
#endif /*__flashRead_H */
