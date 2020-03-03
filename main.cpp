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
#include <stdlib.h>

extern "C" void skip_strip(void);

#define PRINT_ENABLE

static void ClearHooks(void)
{
	int *p = (int *)GetHooks();

	for (unsigned int count = 0; count < (s_hooksSize / 4); count++)
		p[count] = 0;
}

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
#ifdef __m68k__
	static_assert(((unsigned int)LOAD_POINT) >= ((unsigned int)RAM_BASE + s_hooksSize), "structure change");
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

#ifdef PRINT_ENABLE
	put_string("size "); put_dec_short_num(header.m_size, false);
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

