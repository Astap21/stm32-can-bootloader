#include "flashRead.h"

//������� ��� ������ ������ �� ����
uint32_t flash_read(uint32_t address) {
	return (*(__IO uint32_t*) address);
}
