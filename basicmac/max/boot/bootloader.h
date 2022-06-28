/*     _____                _        __     _
 *    /__   \_ __ __ _  ___| | __ /\ \ \___| |_
 *      / /\/ '__/ _` |/ __| |/ //  \/ / _ \ __|
 *     / /  | | | (_| | (__|   '/ /\  /  __/ |_
 *     \_\  |_|  \__,_|\___|_|\_\_\ \_\\___|\__|
 *
 * Copyright (c) 2016-2018 Trackio International AG
 * All rights reserved.
 *
 */

//
//   EEPROM
//   +-----------------------+
//   |                       |
//   |                       |
//   +-----------------------+  <-- 08080000 BOOT_DEVINFO (fwupdate, target, serial, deveui, joineui...)
//
//
//   FLASH
//   +-----------------------+  <-- 08020000 (128K)
//   |                       |  ^
//   |                       |  |
//   | firmware update       |  | upsize (incl header)
//   |                       |  |                                          crc32(uphdr+12, upsize-12)
//   |                       |  v                                  <----------------------------------------->
//   +-----------------------+  <-- uphdr (upmagic, upcrc, upsize, uptype, upinfo, target, fwsize), payload...
//   |                       |
//   |                       |
//   |                       |
//   | (temporary update)    |
//   |                       |
//   |                       |
//   |                       |
//   +-----------------------+
//   |                       |
//   |                       |
//   |                       |
//   +-----------------------+
//   |                       |  ^
//   |                       |  |
//   |                       |  |
//   |                       |  |
//   | firmware              |  | size (incl header)
//   |                       |  |
//   |                       |  |
//   |                       |  |                                                   crc32(inittab+12, size-12)
//   |                       |  v                                                  <-------------------------->
//   +-----------------------+  <-- 08003000 (12K) BOOT_INITTAB (magic, crc, size, version, target...), code...
//   |                       |
//   | bootloader            |  <-- HAL_boottab (variable address, table of pointers to exported functions)
//   |                       |
//   +-----------------------+  <-- 08000000
//

#ifndef _bootloader_h_
#define _bootloader_h_

#include <stdint.h>

#define BOOT_INITTAB_MAGIC	0x57464e54	// "TNFW"
#define BOOT_UPHDR_MAGIC	0x50554e54	// "TNUP"

// Firmware initialization table
typedef struct {
    uint32_t	magic;		// magic marker for inittab
    uint32_t	crc;		// firmware CRC
    uint32_t	size;		// firmware size (in bytes, including this header)

    /* -- everything below until end (size-12) is included in CRC -- */

    uint32_t	version;	// firmware version
    uint32_t	target;		// firmware target

    uint32_t	sidata; 	// load address of data segment
    uint32_t	sdata;		// start address of data segment
    uint32_t	edata;	 	// end address of data segmnet
    uint32_t	sbss;		// start address of BSS segment
    uint32_t	ebss;		// end address of BSS segment

    uint32_t	irqinit;	// NVIC interrupt vector initializer

    uint32_t	entrypoint;	// address of firmware entrypoint
} boot_inittab;

// NVIC interrupt definition
typedef struct {
    uint32_t	num;		// NVIC interrupt number
    void*	handler;	// Pointer to handler function
} boot_irqdef;

// Firmware update types -- DO NOT ALTER VALUES!
enum {
    BOOT_UPTYPE_PLAIN       = 0,
    BOOT_UPTYPE_LZ4         = 1,
    BOOT_UPTYPE_LZ4DICT     = 2,
    BOOT_UPTYPE_SELFEXTRACT = 3,
    BOOT_UPTYPE_LZ4PATCH    = 4,
};

// Firmware update header
typedef struct {
    uint32_t	upmagic;	// magic marker for update header
    uint32_t	upcrc;		// update CRC
    uint32_t	upsize;		// update size (in bytes, including this header, without trailer)

    /* -- everything below until end (upsize-12) is included in CRC -- */

    uint32_t	uptype;		// update type
    uint32_t	upinfo;		// additional type-specific update info (e.g. dictionary crc)
    uint32_t	target;         // firmware target
    uint32_t	fwversion;	// firmware version
    uint32_t	fwcrc;		// firmware CRC
    uint32_t	fwsize;		// firmware size (in bytes, including inittab)
} boot_uphdr;

// update patch block
typedef struct {
    uint8_t hash[8];  // block hash (sha256[0-7], or crc32 || aes128-cmac[0-3])
    uint8_t size;     // block size (log2, multiple of flash page size, e.g. 12 for 4096)
    uint8_t off;      // block offset (in multiples of block size)
    uint16_t dictoff; // dictionary offset (in bytes)
    uint16_t dictlen; // dictionary size (in bytes)
    uint16_t len;     // length of lz4-compressed block data (in bytes, up to block size)
    uint8_t data[0];  // lz4-compressed block data (plus 0-3 bytes word padding)
} boot_uppatchblock;

// Bootloader information table
typedef struct {
    uint32_t version;				// version of boot loader
    int (*update) (void* ptr);			// function to set firmware update pointer (return value only since v7!)
    __attribute__((noreturn))
    void (*panic) (uint32_t no, uint32_t addr);	// firmware panic function
    uint32_t (*crc32) (void* buf, uint32_t nwords);	// CRC32 function

    // new in version 3
    void (*erase_flash) (uint32_t* dst, uint32_t* end);
    void (*write_flash) (uint32_t* dst, uint32_t* src, uint32_t nwords, uint32_t flags);
    uint32_t (*aes) (uint8_t mode, uint8_t* buf, uint16_t len, uint32_t* key, uint32_t* aux);
} boot_boottab;

// public functions
int lz4compress(unsigned char* src, int srclen, unsigned char* dict, int dictlen, unsigned char* dst, int dstlen);
int lz4decompress(unsigned char* src, int srclen, unsigned char* dst, unsigned char* dict, int dictlen);
void write_flash (uint32_t* dst, uint32_t* src, uint32_t nwords, uint32_t flags);

enum {
    BOOT_WF_PAGEERASE = (1 << 0),
};

// Device information table (in EEPROM)
// all fields MUST be 32-bit aligned
// *** DO NOT MODIFY; APPEND ONLY; 1k MAX! ***
typedef struct {
    uint32_t	fwupdate;	// 0x00 pointer to valid update
    uint32_t	fwupdate2;	// 0x04 pointer to valid update
    uint32_t	hwid;		// 0x08 device target id (hardware identifier)
    uint8_t	serial[16];	// 0x0c Production serial number
    uint32_t	regdomain;      // 0x1c Regulatory domain (os_getRegDomain)
    uint8_t	deveui[8];	// 0x20 LoRaWAN Device EIU
    uint8_t	joineui[8];	// 0x28 LoRaWAN Join EUI
    uint8_t	nwkkey[16];	// 0x30 LoRaWAN network key
    uint8_t	appkey[16];	// 0x40 LoRaWAN application key
    uint8_t	nwkskey[16];	// 0x50 LoRaWAN network session key
    uint8_t	appskey[16];	// 0x60 LoRaWAN application session key
    uint32_t	netid;          // 0x70 network id
    uint32_t	devaddr;	// 0x74 device address
    uint32_t	seqnoUp;	// 0x78 up frame sequence counter
    uint32_t	seqnoDn;	// 0x7c down frame sequence counter (network)
    uint32_t	seqnoADn;	// 0x80 down frame sequence counter (application)
    uint32_t	datarate;	// 0x84 current data rate
    int32_t	rssical;	// 0x88 RSSI calibration value (Gemtek)
    int32_t	powoffset;	// 0x8c RF output power correction offset
    uint32_t	bootmode;	// 0x90 Boot mode
} boot_devinfo;


// Include some convenience macros if building for devices with bootloader support
#ifdef CFG_bootloader
#include "hw.h"
extern uint32_t _sinittab;	// defined in linker script
#define BOOT_INITTAB		((boot_inittab*) &_sinittab)
#define BOOT_DEVINFO		((boot_devinfo*) EEPROM_BASE)
#define BOOT_EECONFIG		(EEPROM_BASE + 1024)
extern boot_boottab* HAL_boottab;	// provided by HAL
#endif

#endif
