#include "flashRead.h"

//функции для чтения данных из флеш
uint32_t flash_read(uint32_t address) {
	return (*(__IO uint32_t*) address);
}
