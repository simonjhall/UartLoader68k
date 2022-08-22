/*
 * main.cpp
 *
 *  Created on: 29 Aug 2018
 *      Author: simon
 */

#include "uart.h"
#include "common.h"
#include "crc32.h"
#include "misc_asm.h"
#include "inter_process.h"

#include "sdcard_spi.h"

#include <stdlib.h>

extern "C" void skip_strip(void);

// #define PRINT_ENABLE
// #define SDCARD_ENABLE
#define MEM_ZERO_ENABLE

/* memory layout example:
- riscv 32-bit
- 16 MB ebay board

- rom: uart
- dram:
0x10000000
hooks
0x10000050
nothing
0x10000100
"LOAD_POINT", executable base address after loading
0x1000b744
current end of debugger binary (32KB of user mode stack is included in this)
0x100ffffc
initial machine mode stack
*/


#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static unsigned int swap32(unsigned int num)
{
	return ((num>>24)&0xff) | // move byte 3 to byte 0
                    ((num<<8)&0xff0000) | // move byte 1 to byte 2
                    ((num>>8)&0xff00) | // move byte 2 to byte 1
                    ((num<<24)&0xff000000); // byte 0 to byte 3
}
#endif

static void ClearHooks(void)
{
	int *p = (int *)GetHooks();

	for (unsigned int count = 0; count < (s_hooksSize / 4); count++)
		p[count] = 0;
}

#ifdef __m68k__
extern "C" void bus_error(ExceptionState *pState)
{
	if (GetHooks()->BusError)
		GetHooks()->BusError(pState);
	else
		ASSERT(0);
}

extern "C" void addr_error(ExceptionState *pState)
{
	if (GetHooks()->AddrError)
		GetHooks()->AddrError(pState);
	else
		ASSERT(0);
}

extern "C" void illegal_instruction(ExceptionState *pState)
{
	if (GetHooks()->IllegalInst)
		GetHooks()->IllegalInst(pState);
	else
		ASSERT(0);
}

extern "C" void div_zero(ExceptionState *pState)
{
	if (GetHooks()->DivZero)
		GetHooks()->DivZero(pState);
	else
		ASSERT(0);
}

extern "C" void misc_trap(ExceptionState *pState)
{
	if (GetHooks()->MiscTrap)
		GetHooks()->MiscTrap(pState);
	else
		ASSERT(0);
}

extern "C" void trace(ExceptionState *pState)
{
	if (GetHooks()->Trace)
		GetHooks()->Trace(pState);
	else
		ASSERT(0);
}

extern "C" void spurious_int(ExceptionState *pState)
{
	if (GetHooks()->SpuriousInt)
		GetHooks()->SpuriousInt(pState);
	else
		ASSERT(0);
}

extern "C" void auto_1(ExceptionState *pState)
{
	if (GetHooks()->Auto1)
		GetHooks()->Auto1(pState);
	else
		ASSERT(0);
}

extern "C" void auto_2(ExceptionState *pState)
{
	if (GetHooks()->Auto2)
		GetHooks()->Auto2(pState);
	else
		ASSERT(0);
}

extern "C" void auto_3(ExceptionState *pState)
{
	if (GetHooks()->Auto3)
		GetHooks()->Auto3(pState);
	else
		ASSERT(0);
}

extern "C" void auto_4(ExceptionState *pState)
{
	if (GetHooks()->Auto4)
		GetHooks()->Auto4(pState);
	else
		ASSERT(0);
}

extern "C" void auto_5(ExceptionState *pState)
{
	if (GetHooks()->Auto5)
		GetHooks()->Auto5(pState);
	else
		ASSERT(0);
}

extern "C" void auto_6(ExceptionState *pState)
{
	if (GetHooks()->Auto6)
		GetHooks()->Auto6(pState);
	else
		ASSERT(0);
}

extern "C" void auto_7(ExceptionState *pState)
{
	if (GetHooks()->Auto7)
		GetHooks()->Auto7(pState);
	else
		ASSERT(0);
}

extern "C" void trap(ExceptionState *pState)
{
	if (GetHooks()->Trap)
		if (GetHooks()->Trap(pState))
			return;

	switch (pState->d[0])
	{
	case TRAP_PRINT_CHAR:
		put_char(pState->d[1]);
		break;
	case TRAP_PRINT_HEX_NUM:
		put_hex_num(pState->d[1]);
		break;
	case TRAP_PRINT_HEX_BYTE:
		put_hex_byte(pState->d[1]);
		break;
	case TRAP_PRINT_DEC_SHORT_NUM:
		put_dec_short_num(pState->d[1], pState->d[2]);
		break;
	case TRAP_PRINT_STRING:
		put_string((char *)pState->d[1]);
		break;
	default:
		ASSERT(0);
	}
}

#elif __riscv

extern "C" void interrupt_exception(ExceptionState *pState, unsigned long mcause)
{
	bool failure = false;
	
	if ((long)mcause < 0)
	{
		//interrupt

#define CALL(x, y)	case x:	\
						if (GetHooks()->y) \
							GetHooks()->y(pState); \
						else failure = true;	\
						break;

		switch (mcause & 63)
		{
		CALL(1, SuperSoftInt)
		CALL(3, MachSoftInt)
		CALL(5, SuperTimerInt)
		CALL(7, MachTimerInt)
		CALL(9, SuperExtInt)
		CALL(11, MachExtInt)

		default:
			failure = true;
		}
	}
	else
	{
		//exception
		switch (mcause & 63)
		{
		CALL(0, InstAddrMisaligned)
		CALL(1, InstAddrFault)
		CALL(2, IllegalInst)
		CALL(3, Breakpoint)
		CALL(4, LoadAddrMisaligned)
		CALL(5, LoadAddrFault)
		CALL(6, StoreAddrMisaligned)
		CALL(7, StoreAddrFault)
		CALL(8, EcallFromU)
		CALL(9, EcallFromS)
		CALL(11, EcallFromM)
		CALL(12, InstPageFault)
		CALL(13, LoadPageFault)
		CALL(15, StorePageFault)
		default:
			failure = true;
		}
	}
	
	ASSERT(!failure);
}

#endif

extern "C" __attribute__ ((noreturn)) void _start(void)
{
	skip_strip();

	//wait for pwren to go low
	while (!is_usb_connected())
		;


#ifdef PRINT_ENABLE
	put_char('\n');
	put_string(__DATE__);
	put_char(' ');
	put_string(__TIME__);
	put_char('\n');
	put_string("rom _start begins\nwaiting for data\n");
#endif

	ClearHooks();
	void *pLoadPoint = LOAD_POINT;

#ifdef MEM_ZERO_ENABLE
	{
#ifdef PRINT_ENABLE
		put_string("clearing memory...");
#endif
		unsigned int *pClear = (unsigned int *)LOAD_POINT;
		unsigned int *pClearEnd = (unsigned int *)((char *)RAM_BASE + RAM_SIZE);

		while (pClear != pClearEnd)
		{
			pClear[0] = 0;
			pClear[1] = 0;
			pClear[2] = 0;
			pClear[3] = 0;

			pClear[4] = 0;
			pClear[5] = 0;
			pClear[6] = 0;
			pClear[7] = 0;

			pClear += 8;
		}
#ifdef PRINT_ENABLE
		put_string("done\n");
#endif
	}
#endif

#ifdef SDCARD_ENABLE
#ifdef HAS_SPI
	SdCard sd;
	switch (sd.Init())
	{
		case SdCard::kErrorNoError:
		{
			unsigned char block[512];
			put_string("SD card initialised ok\n");

			sd.ReadBlock(block, 0);

			for (int y = 0; y < 32; y++)
			{
				for (int x = 0; x < 16; x++)
				{
					put_hex_byte(block[x + y * 16]);
					put_char(' ');
				}
				put_char('\n');
			}

			put_char('\n');

			sd.ReadBlock(block, 1);

			for (int y = 0; y < 32; y++)
			{
				for (int x = 0; x < 16; x++)
				{
					put_hex_byte(block[x + y * 16]);
					put_char(' ');
				}
				put_char('\n');
			}
			break;
		}

		case SdCard::kErrorInitTimeout:
			put_string("SD card init timeout\n");
			break;

		case SdCard::kErrorCmd8Error:
			put_string("SD card cmd8 error\n");
			break;
		
		case SdCard::kErrorNotSdCard:
			put_string("not SD card\n");
			break;

		default:
			put_string("unknown error\n");
	}
#endif
#endif

	//wait for the magic number
	unsigned int last = 0;

	while (1)
	{
		unsigned char c = get_char();

		last = (last << 8) | c;

		if (last == 0xbeefcafe)
			break;
	}

#ifdef PRINT_ENABLE
	put_string("magic number received\n");
#endif

	//load the header
	struct TransferHeader
	{
		unsigned int m_size;
		unsigned int m_entryPoint;
		unsigned int m_crc;
	} header;

	static_assert(sizeof(header) == 12, "size change");
	volatile unsigned char *pDest = (unsigned char *)&header;

	while (pDest < (unsigned char *)(&header + 1))
	{
		unsigned char c = get_char();
		*pDest++ = c;
	}

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	header.m_size = swap32(header.m_size);
	header.m_entryPoint = swap32(header.m_entryPoint);
	header.m_crc = swap32(header.m_crc);
#endif

#ifdef PRINT_ENABLE
	put_string("size "); put_hex_num(header.m_size);
	put_string("\nentry point "); put_hex_num(header.m_entryPoint);
	put_string("\ncrc "); put_hex_num(header.m_crc);
#endif

	//now load the data
	pDest = (unsigned char *)pLoadPoint;

	while (pDest < ((unsigned char *)pLoadPoint + header.m_size))
	{
		unsigned char c = get_char();
		*pDest++ = c;
	}

	unsigned int crc = crc32b((unsigned char *)pLoadPoint, header.m_size);
#ifdef PRINT_ENABLE
	put_string("\ndone transfer\ncrc is ");
	put_hex_num(crc);
#endif

	if (crc == header.m_crc)
	{
#ifdef PRINT_ENABLE
		put_string("\nstarting from ram\n");
#endif
		void (*pEntry)(void *) = (void (*)(void *))((unsigned int)header.m_entryPoint);
		pEntry(pLoadPoint);

#ifdef PRINT_ENABLE
		put_string("\nrom _start returning\n");
#endif
	}
	else
	{
#ifdef PRINT_ENABLE
		put_string("\ncrc does not match, ");
		put_hex_num(header.m_crc);
		put_char('\n');
#endif
	}
	while (1);
}
