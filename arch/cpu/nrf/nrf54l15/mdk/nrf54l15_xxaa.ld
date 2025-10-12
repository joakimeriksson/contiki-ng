/* Linker script for minimal nRF54L15 application core bring-up */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x0015D000
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x0002F000
}

INCLUDE "nrf_common.ld"
