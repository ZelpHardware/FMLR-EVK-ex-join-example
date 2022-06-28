// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
// Copyright (C) 2014-2016 IBM Corporation. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

#include "peripherals.h"
#include "hw.h"
#include "debug.h"
#include "flc.h"

uint32_t eeprom_buffer[EEPROM_SZ/4]; // used during flash page erase
// write 32-bit word to EEPROM memory
void eeprom_write (void* dest, unsigned int val) {

    debug_printf("eeprom_write val: %x to addr: %x\r\n", val, dest);
	uint32_t err;

	uint32_t *eeprom_pointer = (uint32_t*)(DATA_EEPROM_BASE);
	uint32_t *ram_pointer = eeprom_buffer;

	for (int i=0; i< (EEPROM_SZ/4); i++){
		*ram_pointer = *eeprom_pointer;
		ram_pointer++;
		eeprom_pointer++;
	}

	*((uint32_t*)((uint32_t*)(dest) - (uint32_t*)(DATA_EEPROM_BASE) + eeprom_buffer)) = (uint32_t)(val);

	err = FLC_PageErase(DATA_EEPROM_BASE, 0x55, 0x02); //Todo: error handling
	if (err!=0){
		debug_printf("err:%x\r\n", err);
	}
    flash_write((uint32_t*)(DATA_EEPROM_BASE), eeprom_buffer, EEPROM_SZ/4, false);


#if 0
    u4_t* addr = dest;
    // check previous value
    if( *addr != val ) {
        // unlock data eeprom memory and registers
        FLASH->PEKEYR = 0x89ABCDEF; // FLASH_PEKEY1
        FLASH->PEKEYR = 0x02030405; // FLASH_PEKEY2

        // only auto-erase if neccessary (when content is non-zero)
#if defined(STM32L0)
        FLASH->PECR &= ~FLASH_PECR_FIX; // clear FIX
#elif defined(STM32L1)
        FLASH->PECR &= ~FLASH_PECR_FTDW; // clear FTDW
#endif

        // write value
        *addr = val;

        // check for end of programming
        while( FLASH->SR & FLASH_SR_BSY ); // loop while busy

        // lock data eeprom memory and registers
        FLASH->PECR |= FLASH_PECR_PELOCK;

        // verify value
	ASSERT( *((volatile u4_t*) addr) == val );
    }
#endif
}

void eeprom_copy (void* dest, const void* src, int len) {
	ASSERT( (((u4_t) dest | (u4_t) src | len) & 3) == 0 );

  debug_printf("eeprom_copy src addr: 0x%x to dest addr: 0x%x, len:%d\r\n", src, dest, len);
  uint32_t err;

  //copy flash page to RAM
  uint32_t *eeprom_pointer = (uint32_t*)(DATA_EEPROM_BASE);
  uint32_t *ram_pointer = eeprom_buffer;
  //debug_printf("eeprom_copy flash addr: 0x%x to buffer addr: 0x%x, len:%d\r\n", eeprom_pointer, ram_pointer, len);
  for (int i=0; i< (EEPROM_SZ/4); i++){
    *ram_pointer = *eeprom_pointer;
    ram_pointer++;
    eeprom_pointer++;
  }

  //modify RAM
  ram_pointer = (uint32_t*)((uint32_t)(eeprom_buffer) - DATA_EEPROM_BASE + (uint32_t)(dest));
  uint32_t *src_pointer = (uint32_t*)(src);
  //debug_printf("eeprom_copy modify buffer addr: 0x%x from src addr: 0x%x, len:%d\r\n", ram_pointer, src_pointer, len);
  for (int i=0; i< len/4; i++){
    *ram_pointer = *src_pointer;
    ram_pointer++;
    src_pointer++;
  }

  //write EEPROM
  //debug_printf("eeprom_copy erase\r\n");
  err = FLC_PageErase(DATA_EEPROM_BASE, 0x55, 0x02); //Todo: error handling
  if (err!=0){
    debug_printf("err:%x\r\n", err);
  }

  eeprom_pointer = (uint32_t*)(DATA_EEPROM_BASE);
  ram_pointer = eeprom_buffer;
  //debug_printf("eeprom_copy write flash addr: 0x%x from eeprom buffer addr: 0x%x, len:%d\r\n", ram_pointer, eeprom_pointer, len);
  flash_write((uint32_t*)(DATA_EEPROM_BASE), eeprom_buffer, EEPROM_SZ/4, false);


#if 0 //original implementation
    u4_t* d = (u4_t*) dest;
    u4_t* s = (u4_t*) src;
    len >>= 2;
    while( len-- ) {
        eeprom_write(d++, *s++);
    }
#endif
}
