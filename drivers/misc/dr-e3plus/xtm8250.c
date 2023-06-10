/*
 *  linux/drivers/char/8250.c
 *
 *  Driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *  A note about isabase / iobase / mapbase / membase :
 *  ===================================================
 *  isabase is the ISA bus base address.
 *  iobase is the ISA bus port offset address.
 *  mapbase is the physical address of the IO port.
 *  membase is an 'ioremapped' cookie.
 *
 *  Therefore mapbase = isabase + iobase,
 *  and membase is the ioremap() cookie of ioremap(mapbase, ...).
 *
 *  ===================================================
 *  Fixed rtu framing support for serial modbus - Nov 2009
 *  David Nguyen (david.nguyen@dynamicratings.com)
 *
 */

static char const xtm8250_c_rcsid[]__attribute__ ((used)) = "$Id$";

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/interrupt.h> /* tasklets */

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>		/* for userspace access. DN	*/
#include "xtm8250.h"

/* RTU SUPPORTS - User specifics
 * Note: MODBUS_RTU_FLAG must not same as any
 * 		 UIF_* uart_info flags defined in "serial_core.h"
 * ******************************************************************** */
#define MODBUS_RTU_FLAG	(1<<15)	/* 1. Indicate uart does support rtu	*/
								/*  	stored in uart_port.state->flags	*/
#define MODBUS_RTU_GET	0x4D47	/* 2. Check rtu support state of uart	*/
#define MODBUS_RTU_SET	0x4D52	/* 3. Enable/Disable rtu support of Uart*/
#define MODBUS_TXENB	0x4D53	/* 4. Cmd 'modbus set'. Stored value in	*/
								/*		UART_SCR bit 0					*/
/* -------------------------------------------------------------------- */
/* ********** IIR flags in addition to "serial_reg.h" ***************** */
#define UIIR_ENBFIFO_MSK	0xF0/* "FIFO Use" Mask - IIR[4:7]			*/
#define UIIR_ENBFIFO_YES	0xC0/* "FIFO Enabled"  - IIR[6:7]			*/
#define UIIR_ENBFIFO_NO		0x00/* "FIFO Disabled" - IIR[6:7]			*/
/* -------------------------------------------------------------------- */
#define UIIR_ID4BIT_MSK		0x0F/* Mask for the IID - IIR[0-3]			*/
#define UIIR_ID4BIT_MSI		0x00/* Modem status interrupt 				*/
#define UIIR_ID4BIT_THRI	0x02/* Transmitter holding register empty	*/
#define UIIR_ID4BIT_RDA		0x04/* Received Data Available				*/
#define UIIR_ID4BIT_CTI		0x0C/* Character Timeout Indication			*/
#define UIIR_ID4BIT_RLSI	0x06/* Receiver line status interrupt		*/
/* ******************************************************************** */
/* This array defines a list of ISA bus interrupt numbers
 * with corresponding PXA GPIO pins as implemented on the
 * D030.  Last elements are zero'd to indicate the end of list.
 * ************************************************************ */
unsigned char isa_gpio[][2] = {
	{3,27},
	{4,31},
	{5,46},
	{6,47},
	{7,45},
	{9,11},
	{10,12},
	{11,13},
	{0,0}};

/*
 * Configuration:
 *   share_irqs - whether we pass SA_SHIRQ to request_irq().  This option
 *                is unsafe when used on edge-triggered interrupts.
 */
static unsigned int share_irqs = SERIAL8250_SHARE_IRQS;
static unsigned int isabase = 0x30000000;
static unsigned int skip_txen_test; /* force skip of txen test at init time */

#define serial_inp(up, ofs)			serial_in(up, ofs)
#define serial_outp(up, ofs, val)	serial_out(up, ofs, val)

#if defined(DEBUG)
#define DEBUG_AUTOCONF(fmt...)	printk(KERN_INFO fmt)
#else
#define DEBUG_AUTOCONF(fmt...)	do { } while (0)
#endif

#if defined(DEBUG)
#define DEBUG_INTR(fmt...)	printk(KERN_INFO fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#if defined(DEBUG)
#define DEBUG_GENERAL(fmt...)	printk(KERN_INFO fmt)
#else
#define DEBUG_GENERAL(fmt...)	do { } while (0)
#endif

#if defined(DEBUG)
#define DEBUG_MBRTU(fmt...)	printk(KERN_INFO fmt)
#else
#define DEBUG_MBRTU(fmt...)	do { } while (0)
#endif

#define PASS_LIMIT	256

/*
 * We default to IRQ0 for the "no irq" hack.   Some
 * machine types want others as well - they're free
 * to redefine this in their header file.
 */
#define is_real_interrupt(irq)	((irq) != 0)

#define CONFIG_HUB6 1

#include <asm/serial.h>

/*
 * SERIAL_PORT_DFNS tells us about built-in ports that have no
 * standard enumeration mechanism.   Platforms that can find all
 * serial ports via mechanisms like ACPI or PCI need not supply it.
 */
#ifndef SERIAL_PORT_DFNS
#define SERIAL_PORT_DFNS
#endif

#define UART_NR	CONFIG_SERIAL_8250_NR_UARTS


/* Here we define the default xmit fifo size used
 * for each type of UART.
 * ===============================================
 * FIFO trigger levels:
 *			RX	00  01  10  11	TX	00  01  10  11
 * [16550A]	 -	 1   4   8  14	 -	xx  xx  xx  xx
 * ===============================================
 */

static const struct xtm8250_config uart_config[] = {
	[PORT_UNKNOWN] = {
		.name		= "unknown",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_8250] = {
		.name		= "8250",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16450] = {
		.name		= "16450",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550] = {
		.name		= "16550",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_16550A] = {
		.name		= "16550A",
		.fifo_size	= 16,
		.tx_loadsz	= 16,
		.fcr        = UART_FCR_ENABLE_FIFO |    /* Enable FIFOs */
		              UART_FCR_CLEAR_RCVR  |    /* Clear receive fifo */
		              UART_FCR_CLEAR_XMIT  |    /* Clear transmit fifo */
		              UART_FCR_TRIGGER_8,       /* Trigger level = 8 */
		.flags		= UART_CAP_FIFO,
	},
};


void xtm8250_unregister_port(int line);
int xtm8250_register_port(struct uart_port *port);

#define map_8250_in_reg(up, offset) (offset)
#define map_8250_out_reg(up, offset) (offset)

static _INLINE_ unsigned int serial_in(struct uart_8250_port *up, int offset)
{
	offset = map_8250_in_reg(up, offset) << up->port.regshift;

	switch (up->port.iotype) {
	case UPIO_HUB6:
		outb(up->port.hub6 - 1 + offset, up->port.iobase);
		return inb(up->port.iobase + 1);

	case UPIO_MEM:
		return readb(up->port.membase + offset);

	case UPIO_MEM32:
		return readl(up->port.membase + offset);

	default:
		return inb(up->port.iobase + offset);
	}
}

static _INLINE_ void serial_out(struct uart_8250_port *up, int offset, int value)
{
	offset = map_8250_out_reg(up, offset) << up->port.regshift;
	DEBUG_GENERAL("OUT: O %d V %d M %p\n", offset, value, up->port.membase);

	switch (up->port.iotype) {
	case UPIO_HUB6:
		outb(up->port.hub6 - 1 + offset, up->port.iobase);
		outb(value, up->port.iobase + 1);
		break;

	case UPIO_MEM:
		writeb(value, up->port.membase + offset);
		break;

	case UPIO_MEM32:
		writel(value, up->port.membase + offset);
		break;

	default:
		outb(value, up->port.iobase + offset);
	}
}

/*
 * For the 16C950
 */
static void serial_icr_write(struct uart_8250_port *up, int offset, int value)
{
	serial_outp(up, UART_SCR, offset);
	serial_outp(up, UART_ICR, value);
}

static unsigned int serial_icr_read(struct uart_8250_port *up, int offset)
{
	unsigned int value;

	serial_icr_write(up, UART_ACR, up->acr | UART_ACR_ICRRD);
	serial_outp(up, UART_SCR, offset);
	value = serial_inp(up, UART_ICR);
	serial_icr_write(up, UART_ACR, up->acr);

	return value;
}

/*
 * FIFO support.
 */
static inline void xtm8250_clear_fifos(struct uart_8250_port *p)
{
	if (p->capabilities & UART_CAP_FIFO) {
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_outp(p, UART_FCR, 0);
	}
}

/*
 * IER sleep support.  UARTs which have EFRs need the "extended
 * capability" bit enabled.  Note that on XR16C850s, we need to
 * reset LCR to write to IER.
 */
static inline void xtm8250_set_sleep(struct uart_8250_port *p, int sleep)
{
	if (p->capabilities & UART_CAP_SLEEP) {
		if (p->capabilities & UART_CAP_EFR) {
			serial_outp(p, UART_LCR, 0xBF);
			serial_outp(p, UART_EFR, UART_EFR_ECB);
			serial_outp(p, UART_LCR, 0);
		}
		serial_outp(p, UART_IER, sleep ? UART_IERX_SLEEP : 0);
		if (p->capabilities & UART_CAP_EFR) {
			serial_outp(p, UART_LCR, 0xBF);
			serial_outp(p, UART_EFR, 0);
			serial_outp(p, UART_LCR, 0);
		}
	}
}

/*
 * This is a quickie test to see how big the FIFO is.
 * It doesn't work at all the time, more's the pity.
 */
static int size_fifo(struct uart_8250_port *up)
{
	unsigned char old_fcr, old_mcr, old_dll, old_dlm, old_lcr;
	int count;

	old_lcr = serial_inp(up, UART_LCR);
	serial_outp(up, UART_LCR, 0);
	old_fcr = serial_inp(up, UART_FCR);
	old_mcr = serial_inp(up, UART_MCR);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO |
		    UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_outp(up, UART_MCR, UART_MCR_LOOP);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	old_dll = serial_inp(up, UART_DLL);
	old_dlm = serial_inp(up, UART_DLM);
	serial_outp(up, UART_DLL, 0x01);
	serial_outp(up, UART_DLM, 0x00);
	serial_outp(up, UART_LCR, 0x03);
	for (count = 0; count < 256; count++)
		serial_outp(up, UART_TX, count);
	mdelay(20);/* FIXME - schedule_timeout */
	for (count = 0; (serial_inp(up, UART_LSR) & UART_LSR_DR) &&
	     (count < 256); count++)
		serial_inp(up, UART_RX);
	serial_outp(up, UART_FCR, old_fcr);
	serial_outp(up, UART_MCR, old_mcr);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	serial_outp(up, UART_DLL, old_dll);
	serial_outp(up, UART_DLM, old_dlm);
	serial_outp(up, UART_LCR, old_lcr);

	return count;
}

/*
 * Read UART ID using the divisor method - set DLL and DLM to zero
 * and the revision will be in DLL and device type in DLM.  We
 * preserve the device state across this.
 */
static unsigned int autoconfig_read_divisor_id(struct uart_8250_port *p)
{
	unsigned char old_dll, old_dlm, old_lcr;
	unsigned int id;

	old_lcr = serial_inp(p, UART_LCR);
	serial_outp(p, UART_LCR, UART_LCR_DLAB);

	old_dll = serial_inp(p, UART_DLL);
	old_dlm = serial_inp(p, UART_DLM);

	serial_outp(p, UART_DLL, 0);
	serial_outp(p, UART_DLM, 0);

	id = serial_inp(p, UART_DLL) | serial_inp(p, UART_DLM) << 8;

	serial_outp(p, UART_DLL, old_dll);
	serial_outp(p, UART_DLM, old_dlm);
	serial_outp(p, UART_LCR, old_lcr);

	return id;
}

/*
 * This is a helper routine to autodetect StarTech/Exar/Oxsemi UART's.
 * When this function is called we know it is at least a StarTech
 * 16650 V2, but it might be one of several StarTech UARTs, or one of
 * its clones.  (We treat the broken original StarTech 16650 V1 as a
 * 16550, and why not?  Startech doesn't seem to even acknowledge its
 * existence.)
 *
 * What evil have men's minds wrought...
 */
static void autoconfig_has_efr(struct uart_8250_port *up)
{
	unsigned int id1, id2, id3, rev;

	/*
	 * Everything with an EFR has SLEEP
	 */
	up->capabilities |= UART_CAP_EFR | UART_CAP_SLEEP;

	/*
	 * First we check to see if it's an Oxford Semiconductor UART.
	 *
	 * If we have to do this here because some non-National
	 * Semiconductor clone chips lock up if you try writing to the
	 * LSR register (which serial_icr_read does)
	 */

	up->acr = 0;
	serial_outp(up, UART_LCR, 0xBF);
	serial_outp(up, UART_EFR, UART_EFR_ECB);
	serial_outp(up, UART_LCR, 0x00);
	id1 = serial_icr_read(up, UART_ID1);
	id2 = serial_icr_read(up, UART_ID2);
	id3 = serial_icr_read(up, UART_ID3);
	rev = serial_icr_read(up, UART_REV);

	DEBUG_AUTOCONF("950id=%02x:%02x:%02x:%02x ", id1, id2, id3, rev);

	if (id1 == 0x16 && id2 == 0xC9 &&
	    (id3 == 0x50 || id3 == 0x52 || id3 == 0x54)) {
		up->port.type = PORT_16C950;

		/*
		 * Enable work around for the Oxford Semiconductor 952 rev B
		 * chip which causes it to seriously miscalculate baud rates
		 * when DLL is 0.
		 */
		if (id3 == 0x52 && rev == 0x01)
			up->bugs |= UART_BUG_QUOT;
		return;
	}

	/*
	 * We check for a XR16C850 by setting DLL and DLM to 0, and then
	 * reading back DLL and DLM.  The chip type depends on the DLM
	 * value read back:
	 *  0x10 - XR16C850 and the DLL contains the chip revision.
	 *  0x12 - XR16C2850.
	 *  0x14 - XR16C854.
	 */
	id1 = autoconfig_read_divisor_id(up);
	DEBUG_AUTOCONF("850id=%04x ", id1);

	id2 = id1 >> 8;
	if (id2 == 0x10 || id2 == 0x12 || id2 == 0x14) {
		up->port.type = PORT_16850;
		return;
	}

	/*
	 * It wasn't an XR16C850.
	 *
	 * We distinguish between the '654 and the '650 by counting
	 * how many bytes are in the FIFO.  I'm using this for now,
	 * since that's the technique that was sent to me in the
	 * serial driver update, but I'm not convinced this works.
	 * I've had problems doing this in the past.  -TYT
	 */
	if (size_fifo(up) == 64)
		up->port.type = PORT_16654;
	else
		up->port.type = PORT_16650V2;
}

/*
 * We detected a chip without a FIFO.  Only two fall into
 * this category - the original 8250 and the 16450.  The
 * 16450 has a scratch register (accessible with LCR=0)
 */
static void autoconfig_8250(struct uart_8250_port *up)
{
	unsigned char scratch, status1, status2;

	up->port.type = PORT_8250;

	scratch = serial_inp(up, UART_SCR);
	serial_outp(up, UART_SCR, 0xa5);
	status1 = serial_inp(up, UART_SCR);
	serial_outp(up, UART_SCR, 0x5a);
	status2 = serial_inp(up, UART_SCR);
	serial_outp(up, UART_SCR, scratch);

	if (status1 == 0xa5 && status2 == 0x5a)
		up->port.type = PORT_16450;
}

static int broken_efr(struct uart_8250_port *up)
{
	/*
	 * Exar ST16C2550 "A2" devices incorrectly detect as
	 * having an EFR, and report an ID of 0x0201.  See
	 * http://www.exar.com/info.php?pdf=dan180_oct2004.pdf
	 */
	if (autoconfig_read_divisor_id(up) == 0x0201 && size_fifo(up) == 16)
		return 1;

	return 0;
}

/*
 * We know that the chip has FIFOs.  Does it have an EFR?  The
 * EFR is located in the same register position as the IIR and
 * we know the top two bits of the IIR are currently set.  The
 * EFR should contain zero.  Try to read the EFR.
 */
static void autoconfig_16550a(struct uart_8250_port *up)
{
	unsigned char status1, status2;
	unsigned int iersave;

	up->port.type = PORT_16550A;
	up->capabilities |= UART_CAP_FIFO;

	/*
	 * Check for presence of the EFR when DLAB is set.
	 * Only ST16C650V1 UARTs pass this test.
	 */
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	if (serial_inp(up, UART_EFR) == 0) {
		serial_outp(up, UART_EFR, 0xA8);
		if (serial_inp(up, UART_EFR) != 0) {
			DEBUG_AUTOCONF("EFRv1 ");
			up->port.type = PORT_16650;
			up->capabilities |= UART_CAP_EFR | UART_CAP_SLEEP;
		} else {
			DEBUG_AUTOCONF("Motorola 8xxx DUART ");
		}
		serial_outp(up, UART_EFR, 0);
		return;
	}

	/*
	 * Maybe it requires 0xbf to be written to the LCR.
	 * (other ST16C650V2 UARTs, TI16C752A, etc)
	 */
	serial_outp(up, UART_LCR, 0xBF);
	if (serial_inp(up, UART_EFR) == 0 && !broken_efr(up)) {
		DEBUG_AUTOCONF("EFRv2 ");
		autoconfig_has_efr(up);
		return;
	}

	/*
	 * Check for a National Semiconductor SuperIO chip.
	 * Attempt to switch to bank 2, read the value of the LOOP bit
	 * from EXCR1. Switch back to bank 0, change it in MCR. Then
	 * switch back to bank 2, read it from EXCR1 again and check
	 * it's changed. If so, set baud_base in EXCR2 to 921600. -- dwmw2
	 */
	serial_outp(up, UART_LCR, 0);
	status1 = serial_inp(up, UART_MCR);
	serial_outp(up, UART_LCR, 0xE0);
	status2 = serial_inp(up, 0x02); /* EXCR1 */

	if (!((status2 ^ status1) & UART_MCR_LOOP)) {
		serial_outp(up, UART_LCR, 0);
		serial_outp(up, UART_MCR, status1 ^ UART_MCR_LOOP);
		serial_outp(up, UART_LCR, 0xE0);
		status2 = serial_inp(up, 0x02); /* EXCR1 */
		serial_outp(up, UART_LCR, 0);
		serial_outp(up, UART_MCR, status1);

		if ((status2 ^ status1) & UART_MCR_LOOP) {
			unsigned short quot;

			serial_outp(up, UART_LCR, 0xE0);

			quot = serial_inp(up, UART_DLM) << 8;
			quot += serial_inp(up, UART_DLL);
			quot <<= 3;

			status1 = serial_inp(up, 0x04); /* EXCR1 */
			status1 &= ~0xB0; /* Disable LOCK, mask out PRESL[01] */
			status1 |= 0x10;  /* 1.625 divisor for baud_base --> 921600 */
			serial_outp(up, 0x04, status1);

			serial_outp(up, UART_DLL, quot & 0xff);
			serial_outp(up, UART_DLM, quot >> 8);

			serial_outp(up, UART_LCR, 0);

			up->port.uartclk = 921600*16;
			up->port.type = PORT_NS16550A;
			up->capabilities |= UART_NATSEMI;
			return;
		}
	}

	/*
	 * No EFR.  Try to detect a TI16750, which only sets bit 5 of
	 * the IIR when 64 byte FIFO mode is enabled when DLAB is set.
	 * Try setting it with and without DLAB set.  Cheap clones
	 * set bit 5 without DLAB set.
	 */
	serial_outp(up, UART_LCR, 0);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status1 = serial_inp(up, UART_IIR) >> 5;
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(up, UART_LCR, UART_LCR_DLAB);
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO | UART_FCR7_64BYTE);
	status2 = serial_inp(up, UART_IIR) >> 5;
	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_outp(up, UART_LCR, 0);

	DEBUG_AUTOCONF("iir1=%d iir2=%d ", status1, status2);

	if (status1 == 6 && status2 == 7) {
		up->port.type = PORT_16750;
		up->capabilities |= UART_CAP_AFE | UART_CAP_SLEEP;
		return;
	}

	/*
	 * Try writing and reading the UART_IER_UUE bit (b6).
	 * If it works, this is probably one of the Xscale platform's
	 * internal UARTs.
	 * We're going to explicitly set the UUE bit to 0 before
	 * trying to write and read a 1 just to make sure it's not
	 * already a 1 and maybe locked there before we even start start.
	 */
	iersave = serial_inp(up, UART_IER);
	serial_outp(up, UART_IER, iersave & ~UART_IER_UUE);
	if (!(serial_inp(up, UART_IER) & UART_IER_UUE)) {
		/*
		 * OK it's in a known zero state, try writing and reading
		 * without disturbing the current state of the other bits.
		 */
		serial_outp(up, UART_IER, iersave | UART_IER_UUE);
		if (serial_inp(up, UART_IER) & UART_IER_UUE) {
			/*
			 * It's an Xscale.
			 * We'll leave the UART_IER_UUE bit set to 1 (enabled).
			 */
			DEBUG_AUTOCONF("Xscale ");
			up->port.type = PORT_XSCALE;
			up->capabilities |= UART_CAP_UUE;
			return;
		}
	} else {
		/*
		 * If we got here we couldn't force the IER_UUE bit to 0.
		 * Log it and continue.
		 */
		DEBUG_AUTOCONF("Couldn't force IER_UUE to 0 ");
	}
	serial_outp(up, UART_IER, iersave);
}

/*
 * This routine is called by rs_init() to initialize a specific serial
 * port.  It determines what type of UART chip this serial port is
 * using: 8250, 16450, 16550, 16550A.  The important question is
 * whether or not this UART is a 16550A or not, since this will
 * determine whether or not we can use its FIFO features or not.
 */
static void autoconfig(struct uart_8250_port *up, unsigned int probeflags)
{
	unsigned char status1, scratch, scratch2, scratch3;
	unsigned char save_lcr, save_mcr;
	unsigned long flags;

	if (!up->port.iobase && !up->port.mapbase && !up->port.membase)
		return;

	DEBUG_AUTOCONF("ttyS%d: autoconf (0x%04x, 0x%p): ",
		up->port.line, up->port.iobase, up->port.membase);

	/*
	 * We really do need global IRQs disabled here - we're going to
	 * be frobbing the chips IRQ enable register to see if it exists.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->capabilities = 0;
	up->bugs = 0;

	if (!(up->port.flags & UPF_BUGGY_UART)) {
		/*
		 * Do a simple existence test first; if we fail this,
		 * there's no point trying anything else.
		 *
		 * 0x80 is used as a nonsense port to prevent against
		 * false positives due to ISA bus float.  The
		 * assumption is that 0x80 is a non-existent port;
		 * which should be safe since include/asm/io.h also
		 * makes this assumption.
		 *
		 * Note: this is safe as long as MCR bit 4 is clear
		 * and the device is in "PC" mode.
		 */
		scratch = serial_inp(up, UART_IER);
		serial_outp(up, UART_IER, 0);
		/*
		 * Mask out IER[7:4] bits for test as some UARTs (e.g. TL
		 * 16C754B) allow only to modify them if an EFR bit is set.
		 */
		scratch2 = serial_inp(up, UART_IER) & 0x0F;
		serial_outp(up, UART_IER, 0x0F);

		scratch3 = serial_inp(up, UART_IER) & 0x0F;
		serial_outp(up, UART_IER, scratch);

		if (scratch2 != 0 || scratch3 != 0x0F) {
			/*
			 * We failed; there's nothing here
			 */
			DEBUG_AUTOCONF("IER test failed (%02x, %02x) ", scratch2, scratch3);
			goto out;
		}
	}

	save_mcr = serial_inp(up, UART_MCR);
	save_lcr = serial_inp(up, UART_LCR);

	/*
	 * Check to see if a UART is really there.  Certain broken
	 * internal modems based on the Rockwell chipset fail this
	 * test, because they apparently don't implement the loopback
	 * test mode.  So this test is skipped on the COM 1 through
	 * COM 4 ports.  This *should* be safe, since no board
	 * manufacturer would be stupid enough to design a board
	 * that conflicts with COM 1-4 --- we hope!
	 */
	if (unlikely(!(up->port.flags & UPF_SKIP_TEST))) {
		serial_outp(up, UART_MCR, UART_MCR_LOOP | 0x0A);
		status1 = serial_inp(up, UART_MSR) & 0xF0;
		serial_outp(up, UART_MCR, save_mcr);
		if (status1 != 0x90) {
			DEBUG_AUTOCONF("LOOP test failed (%02x) ", status1);
			goto out;
		}
	}

	/*
	 * We're pretty sure there's a port here.  Lets find out what
	 * type of port it is.  The IIR top two bits allows us to find
	 * out if it's 8250 or 16450, 16550, 16550A or later.  This
	 * determines what we test for next.
	 *
	 * We also initialise the EFR (if any) to zero for later.  The
	 * EFR occupies the same register location as the FCR and IIR.
	 */
	serial_outp(up, UART_LCR, 0xBF);
	serial_outp(up, UART_EFR, 0);
	serial_outp(up, UART_LCR, 0);

	serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	scratch = serial_inp(up, UART_IIR) >> 6;

	DEBUG_AUTOCONF("iinfo=%d ", scratch);

	switch (scratch) {
	case 0:
		DEBUG_AUTOCONF("%s: run autoconfig_8250\n", __FUNCTION__);
		autoconfig_8250(up);
		break;
	case 1:
		DEBUG_AUTOCONF("%s: assign port as unknown\n", __FUNCTION__);
		up->port.type = PORT_UNKNOWN;
		break;
	case 2:
		DEBUG_AUTOCONF("%s: assign port as 16550\n", __FUNCTION__);
		up->port.type = PORT_16550;
		break;
	case 3:
		DEBUG_AUTOCONF("%s: run autoconfig_16550a\n", __FUNCTION__);
		autoconfig_16550a(up);
		break;
	}

	serial_outp(up, UART_LCR, save_lcr);

	if (up->capabilities != uart_config[up->port.type].flags) {
		printk(KERN_WARNING
		       "ttyS%d: detected caps %08x should be %08x\n",
			up->port.line, up->capabilities,
			uart_config[up->port.type].flags);
	}

	up->port.fifosize = uart_config[up->port.type].fifo_size;
	up->capabilities = uart_config[up->port.type].flags;
	up->tx_loadsz = uart_config[up->port.type].tx_loadsz;

	if (up->port.type == PORT_UNKNOWN)
		goto out;

	/*
	 * Reset the UART.
	 */
	serial_outp(up, UART_MCR, save_mcr);
	xtm8250_clear_fifos(up);
	serial_inp(up, UART_RX);	// void()
	if (unlikely(up->capabilities & UART_CAP_UUE))
		serial_outp(up, UART_IER, UART_IER_UUE);
	else
		serial_outp(up, UART_IER, 0);
 out:
	spin_unlock_irqrestore(&up->port.lock, flags);
	DEBUG_AUTOCONF("type=%s\n", uart_config[up->port.type].name);
}

static void autoconfig_irq(struct uart_8250_port *up)
{
	unsigned char save_mcr, save_ier;
	unsigned char save_ICP = 0;
	unsigned int ICP = 0;
	unsigned long irqs;
	int irq;

	if (unlikely(up->port.flags & UPF_FOURPORT)) {
		ICP = (up->port.iobase & 0xfe0) | 0x1f;
		save_ICP = inb_p(ICP);
		outb_p(0x80, ICP);
		(void) inb_p(ICP);
	}

	/* forget possible initially masked and pending IRQ */
	probe_irq_off(probe_irq_on());
	save_mcr = serial_inp(up, UART_MCR);
	save_ier = serial_inp(up, UART_IER);
	serial_outp(up, UART_MCR, UART_MCR_OUT1 | UART_MCR_OUT2);

	irqs = probe_irq_on();
	serial_outp(up, UART_MCR, 0);
	udelay(10);
	if (unlikely(up->port.flags & UPF_FOURPORT))  {
		serial_outp(up, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS);
	} else {
		serial_outp(up, UART_MCR,
			    UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2);
	}
	serial_outp(up, UART_IER, 0x0f);	/* enable all intrs */
	(void)serial_inp(up, UART_LSR);
	(void)serial_inp(up, UART_RX);
	(void)serial_inp(up, UART_IIR);
	(void)serial_inp(up, UART_MSR);
	serial_outp(up, UART_TX, 0xFF);
	udelay (20);
	irq = probe_irq_off(irqs);

	serial_outp(up, UART_MCR, save_mcr);
	serial_outp(up, UART_IER, save_ier);

	if (up->port.flags & UPF_FOURPORT)
		outb_p(save_ICP, ICP);

	up->port.irq = (irq > 0) ? irq : 0;
}

static inline void __stop_tx(struct uart_8250_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_outp(p, UART_IER, p->ier);
	}
}

static void xtm8250_stop_tx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	__stop_tx(up);

	/*
	 * We really want to stop the transmitter from sending.
	 */
	if (up->port.type == PORT_16C950) {
		up->acr |= UART_ACR_TXDIS;
		serial_icr_write(up, UART_ACR, up->acr);
	}
}

static void transmit_chars(struct uart_8250_port *up);

/* NB: Remember to vase lsr flags */
static void xtm8250_start_tx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_outp(up, UART_IER, up->ier);

		if (up->bugs & UART_BUG_TXEN) {
			unsigned char lsr;
			lsr = serial_inp(up, UART_LSR);
			up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS; /*Added*/
			if (lsr & UART_LSR_TEMT)
				transmit_chars(up);
		}
	}
	/* NB: Retransmit not supported */
}

static void xtm8250_stop_rx(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_outp(up, UART_IER, up->ier);
}

static void xtm8250_enable_ms(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;

	/* no MSR capabilities */
	if (up->bugs & UART_BUG_NOMSR)
		return;

	up->ier |= UART_IER_MSI;
	serial_outp(up, UART_IER, up->ier);
}

static _INLINE_ void receive_chars(struct uart_8250_port *up, int *status, struct pt_regs *regs)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned char ch, lsr = *status;
	int cnt = 256;
	char tty_flag;
	int templatency = tty->port->low_latency;

	DEBUG_INTR("...%s...", __FUNCTION__);

	do {
		/* max size = 512 bytes */
		if (unlikely(tty->flip.count >= TTY_FLIPBUF_SIZE)) {
	                tty->port->low_latency=1;
			break;
		}

		if (likely(lsr & UART_LSR_DR))
			ch = serial_inp(up, UART_RX);
		else
			/*
			 * Intel 82571 has a Serial Over Lan device that will
			 * set UART_LSR_BI without setting UART_LSR_DR when
			 * it receives a break. To avoid reading from the
			 * receive buffer without UART_LSR_DR bit set, we
			 * just force the read character to be 0
			 */
			ch = 0;

		tty_flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
                            UART_LSR_FE | UART_LSR_OE))) {
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;

				if (unlikely(uart_handle_break(&up->port))) {
					goto ignore_char;
				}

			} else if (lsr & UART_LSR_PE) {
				up->port.icount.parity++;
			} else if (lsr & UART_LSR_FE) {
				up->port.icount.frame++;
			}
			if (lsr & UART_LSR_OE) {
				up->port.icount.overrun++;
			}
			/* Mask off conditions which should be ignored.
			 * ********************************************/
			lsr &= up->port.read_status_mask;
			if (lsr & UART_LSR_BI) {
				DEBUG_INTR("handling break....");
				tty_flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE) {
				tty_flag = TTY_PARITY;
			} else if (lsr & UART_LSR_FE) {
				tty_flag = TTY_FRAME;
			}
		}

		if (unlikely(uart_handle_sysrq_char(&up->port, ch, regs))) {
			goto ignore_char;
		}
		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, tty_flag);

ignore_char:
		lsr = serial_inp(up, UART_LSR);

	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (cnt-- > 0));

  spin_unlock(&up->port.lock);
  tty_flip_buffer_push(tty);
  tty->port->low_latency = templatency;
  spin_lock(&up->port.lock);

	*status = lsr;
	return;
}

static _INLINE_ void transmit_chars(struct uart_8250_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;

	if (up->port.x_char) {
		serial_outp(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		xtm8250_stop_tx(&up->port);
		return;
	}
	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

	count = up->tx_loadsz;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	DEBUG_INTR("THRE...");

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

/* Func: check_modem_status()
 * Desc: now performs more port check & return status
 * Prev: As	static _INLINE_ void
 * 		(!)		check_modem_status(struct uart_8250_port *up)
 * **************************************************************/
static unsigned int check_modem_status(struct uart_8250_port *up)
{
	unsigned int status = serial_inp(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;

	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

		wake_up_interruptible(&up->port.state->delta_msr_wait);
	}
	return status;
}

/* Base timer interval for polling */
static inline int poll_timeout(int timeout)
{
	return timeout > 6 ? (timeout / 2 - 2) : 1;
}

/*
 * This handles the interrupt from one port.
 * Note: Now handles port locking by itself.
 * ********************************************/
static inline void xtm8250_handle_port(struct uart_8250_port *up, struct pt_regs *regs)
{
	unsigned int lsr;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);	/*Added*/

	lsr = serial_inp(up, UART_LSR);	/* Also clear RLSI */

	DEBUG_INTR("\nlsr = 0x%x...", lsr);

	if (lsr & (UART_LSR_DR | UART_LSR_BI)) {
		receive_chars(up, &lsr, regs);

	}
	check_modem_status(up);
	if (lsr & UART_LSR_THRE)
		transmit_chars(up);


	spin_unlock_irqrestore(&up->port.lock, flags);	/*Added*/
	return;
}

static void restart_comms_tasklet(unsigned long data);
DECLARE_TASKLET(restart_comms_69, restart_comms_tasklet, 69);
DECLARE_TASKLET(restart_comms_70, restart_comms_tasklet, 70);

static int s_xtm8250_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
  struct irq_info *i = dev_id;
  struct list_head *l, *end = NULL;
  int gpio = irq - IRQ_GPIO(2) + 2;
  int irq_low = 0;
  int no_data_on_port = 1;
  int handled = 0;
  int ii;

  struct uart_8250_port *up;
  for (ii = 0; ii < PASS_LIMIT; ii++) {
    no_data_on_port = 1;


    l = i->head;
    end = l;
    do {
      unsigned int iinfo;

      up = list_entry(l, struct uart_8250_port, list);
      iinfo = serial_inp(up, UART_IIR);
      if (!(iinfo & UART_IIR_NO_INT)) {
        xtm8250_handle_port(up, regs);

        handled = 1;
        no_data_on_port = 0;
      }
      l = l->next;

    } while (l != end);


    // Check if IRQ/GPIO pin is still high
    if ((GPLR(gpio) & GPIO_bit(gpio)) &&(GEDR(gpio) & GPIO_bit(gpio))) {
      GEDR(gpio) = GPIO_bit(gpio);
    }
    else {
      irq_low = 1;
    }


    if (irq_low && no_data_on_port) {
      break;
    }
  }

  if (ii >= PASS_LIMIT){
    printk(KERN_ERR "xtm8250: too much work for irq%d\n", irq);
  }

	/* Schedule tasklet to keep the port running */
	if ( handled && is_real_interrupt(irq) ) {
    if (irq == 69)
      tasklet_schedule(&restart_comms_69);
    if (irq == 70)
      tasklet_schedule(&restart_comms_70);
	}

  return irq_low && no_data_on_port;
}


irqreturn_t xtm8250_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
  struct irq_info *i = dev_id;
  spin_lock(&i->lock);
  s_xtm8250_interrupt(irq, dev_id, regs);
  spin_unlock(&i->lock);
  return IRQ_RETVAL(1);
}

/*
 * To support ISA shared interrupts, we need to have one interrupt
 * handler that ensures that the IRQ line has been deasserted
 * before returning.  Failing to do this will result in the IRQ
 * line being stuck active, and, since ISA irqs are edge triggered,
 * no more IRQs will be seen.
 */
static void serial_do_unlink(struct irq_info *i, struct uart_8250_port *up)
{
	spin_lock_irq(&i->lock);

	if (!list_empty(i->head)) {
		if (i->head == &up->list)
			i->head = i->head->next;
		list_del(&up->list);
	} else {
		BUG_ON(i->head != &up->list);
    if (up->port.irq == 69)
      tasklet_kill(&restart_comms_69);
    if (up->port.irq == 70) tasklet_kill(&restart_comms_70);
		i->head = NULL;
	}

	spin_unlock_irq(&i->lock);
}

static int serial_link_irq_chain(struct uart_8250_port *up)
{
	struct irq_info *i = irq_lists + up->port.irq;
	int ret, irq_flags = up->port.flags & UPF_SHARE_IRQ ? SA_SHIRQ : 0;

	spin_lock_irq(&i->lock);

	if (i->head) {
		DEBUG_GENERAL("add IRQ to list\n");
		list_add(&up->list, i->head);
		spin_unlock_irq(&i->lock);

		ret = 0;
	} else {
		DEBUG_GENERAL("add IRQ %d to system\n", up->port.irq);

		INIT_LIST_HEAD(&up->list);
		i->head = &up->list;
		spin_unlock_irq(&i->lock);

		set_irq_type(up->port.irq, IRQT_RISING);

		irq_flags |= SA_INTERRUPT;

		ret = request_irq(up->port.irq, xtm8250_interrupt,
				  irq_flags, "xtmserial", i);
		if (ret < 0)
			serial_do_unlink(i, up);
	}

	return ret;
}

static void serial_unlink_irq_chain(struct uart_8250_port *up)
{
	struct irq_info *i = irq_lists + up->port.irq;

	BUG_ON(i->head == NULL);

	if (list_empty(i->head)) {
		DEBUG_GENERAL("Freeing IRQ %d\n",up->port.irq);
		free_irq(up->port.irq, i);
	}
	serial_do_unlink(i, up);
}

/*
 * This function is used to handle ports that do not have an
 * interrupt.  This doesn't work very well for 16450's, but gives
 * barely passable results for a 16550A.  (Although at the expense
 * of much CPU overhead).
 */
static void xtm8250_timeout(unsigned long data)
{
  struct uart_8250_port *up = (struct uart_8250_port *)data;
  unsigned int iinfo;

  iinfo = serial_inp(up, UART_IIR);
  if (!(iinfo & UART_IIR_NO_INT)) {
    xtm8250_handle_port(up, NULL);
  }
  mod_timer(&up->timer, jiffies + poll_timeout(up->port.timeout));
}

static void xtm8250_backup_timeout(unsigned long data)
{
  struct uart_8250_port *up = (struct uart_8250_port *)data;
  struct irq_info *i = irq_lists + up->port.irq;
  void* dev_id = (void*) i;
  int irq = up->port.irq;
  unsigned long flags;

  spin_lock_irqsave(&i->lock, flags);
  s_xtm8250_interrupt(irq, dev_id, NULL);
  spin_unlock_irqrestore(&i->lock, flags);

}


static void restart_comms_tasklet(unsigned long data){
  struct irq_info *i = irq_lists + data;
  void* dev_id = (void*) i;

  s_xtm8250_interrupt(data, dev_id, NULL);
}

/* nb: Remember to save lsr flag */
static unsigned int xtm8250_tx_empty(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;
	unsigned int lsr;

	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&up->port.lock, flags);

	return ((lsr & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);
}

/* Func: xtm8250_get_mctrl
 * NB. To call check_modem_status() here
 * ***************************************/
static unsigned int xtm8250_get_mctrl(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned int status;
	unsigned int ret = 0;

	status = check_modem_status(up);

	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void xtm8250_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_outp(up, UART_MCR, mcr);
}

static void xtm8250_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;

	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_outp(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

/*************************************************************************/

/* Remember to save lsr flag */
static int xtm8250_get_lsr_info(struct uart_8250_port *up, unsigned int *pval)
{
	unsigned int	lsr;
	unsigned int	result;
	unsigned long	flags;

	spin_lock_irqsave(&up->port.lock, flags);
	lsr = serial_inp(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;	/*Added*/
	spin_unlock_irqrestore(&up->port.lock, flags);

	result = ((lsr & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);
	return put_user(result,pval);
}

static int xtm8250_get_modem_info(struct uart_8250_port *up, unsigned int *pval)
{
	unsigned char	mcr = up->mcr;
	unsigned char	msr = 0;
	unsigned int	result;
	unsigned long	flags;

	spin_lock_irqsave(&up->port.lock, flags);
	msr = check_modem_status(up);
	spin_unlock_irqrestore(&up->port.lock, flags);

	result = \
		  ((mcr & UART_MCR_RTS)	 ? TIOCM_RTS	: 0)
		| ((mcr & UART_MCR_DTR)	 ? TIOCM_DTR	: 0)

#ifdef TIOCM_OUT1
		| ((mcr & UART_MCR_OUT1) ? TIOCM_OUT1	: 0)
		| ((mcr & UART_MCR_OUT2) ? TIOCM_OUT2	: 0)
#endif

		| ((msr & UART_MSR_DCD)	 ? TIOCM_CAR	: 0)
		| ((msr & UART_MSR_RI)	 ? TIOCM_RNG	: 0)
		| ((msr & UART_MSR_DSR)	 ? TIOCM_DSR	: 0)
		| ((msr & UART_MSR_CTS)	 ? TIOCM_CTS	: 0);

	return put_user(result,pval);
}

static int xtm8250_set_modem_info(struct uart_8250_port *up, unsigned int cmd, unsigned int *pval)
{
	unsigned int	arg;
	unsigned long	flags;
	unsigned int	old_mcr;

	int error = get_user(arg, pval);
	if (error)
		return error;

	/* Get current modem control settings */
	up->mcr = up->port.ops->get_mctrl(&up->port);
	old_mcr = up->mcr;

	switch (cmd) {
	/* Case 1
	 * ******************************************************/
#ifdef MODBUS_TXENB
	case MODBUS_TXENB:
		DEBUG_MBRTU("%s: Enable RTU tx reg %d from %d\n", __FUNCTION__, UART_SCR, arg);
		spin_lock_irq(&up->port.lock);
		serial_outp(up, UART_SCR, ( arg ? 1:0 ));
		spin_unlock_irq(&up->port.lock);
		return 0;
#endif
	/* Case 2
	 * ******************************************************/
	case TIOCMBIS:
		if (arg & TIOCM_RTS)
			up->mcr |= UART_MCR_RTS;
		if (arg & TIOCM_DTR)
			up->mcr |= UART_MCR_DTR;
#ifdef TIOCM_OUT1
		if (arg & TIOCM_OUT1)
			up->mcr |= UART_MCR_OUT1;
		if (arg & TIOCM_OUT2)
			up->mcr |= UART_MCR_OUT2;
#endif
		break;

	/* Case 3
	 * ******************************************************/
	case TIOCMBIC:
		if (arg & TIOCM_RTS)
			up->mcr &= ~UART_MCR_RTS;
		if (arg & TIOCM_DTR)
			up->mcr &= ~UART_MCR_DTR;
#ifdef TIOCM_OUT1
		if (arg & TIOCM_OUT1)
			up->mcr &= ~UART_MCR_OUT1;
		if (arg & TIOCM_OUT2)
			up->mcr &= ~UART_MCR_OUT2;
#endif
		break;

	/* Case 4
	 * ******************************************************/
	case TIOCMSET:
		up->mcr = ((up->mcr & ~(UART_MCR_RTS  |
#ifdef TIOCM_OUT1
					    		UART_MCR_OUT1 |
					    		UART_MCR_OUT2 |
#endif
					    		UART_MCR_DTR))
			     | ((arg & TIOCM_RTS)  ? UART_MCR_RTS	: 0)
#ifdef TIOCM_OUT1
			     | ((arg & TIOCM_OUT1) ? UART_MCR_OUT1	: 0)
			     | ((arg & TIOCM_OUT2) ? UART_MCR_OUT2	: 0)
#endif
			     | ((arg & TIOCM_DTR)  ? UART_MCR_DTR	: 0));
		break;

	/* ******************************************************/
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&up->port.lock, flags);
	if (old_mcr != up->mcr) {
		up->port.ops->set_mctrl(&(up->port), up->mcr);
	}
	spin_unlock_irqrestore(&up->port.lock, flags);

	return 0;
}

static int xtm8250_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	int error;
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	struct uart_icount cprev, cnow;				/* icounts - KS	*/
	struct serial_icounter_struct *p_cuser;		/* icounts - US	*/
	unsigned long flags;
#ifdef MODBUS_RTU_GET
	unsigned long value=0;
#endif

#ifdef MODBUS_RTU_GET
	if ((cmd != TIOCGSERIAL)	&& (cmd != TIOCSSERIAL)		&&
	    (cmd != MODBUS_RTU_GET) && (cmd != MODBUS_RTU_SET)	&&
	    (cmd != TIOCSERCONFIG)	&& (cmd != TIOCSERGSTRUCT)	&&
	    (cmd != TIOCMIWAIT)		&& (cmd != TIOCGICOUNT))
#else
	if ((cmd != TIOCGSERIAL)	&& (cmd != TIOCSSERIAL)		&&
	    (cmd != TIOCSERCONFIG)	&& (cmd != TIOCSERGSTRUCT)	&&
	    (cmd != TIOCMIWAIT)		&& (cmd != TIOCGICOUNT))
#endif
	{	if (up->port.state->tty->flags & (1 << TTY_IO_ERROR))
		    return -EIO;
	}

	switch (cmd) {		/* "asm/ioctls.h" */
#ifdef MODBUS_RTU_GET
	/* NB: uart flags only valid when port is open.
	 * ****************************************************** */
	case MODBUS_RTU_GET:
		error = put_user((up->port.state->flags & MODBUS_RTU_FLAG)? 1:0,\
						(unsigned long*)arg );
		if (error)
			return error;
		else
			return 0;

	case MODBUS_RTU_SET:
		error = get_user(value, (unsigned long*)arg);
		if (error)
			return error;

		/* Only 16550A to be used*/
		if (up->port.type != PORT_16550A) {
			return -EPERM;
		}

		/* Note: Not to confuse
		 * - We touch "uart_port info flag" but
		 * - We DON'T touch "uart_port flag"
		 * ***************************************/
		up->port.state->flags=((up->port.state->flags & ~MODBUS_RTU_FLAG) | \
										(value ? MODBUS_RTU_FLAG : 0)	);
		return 0;
#endif

	case TIOCMGET:
		return xtm8250_get_modem_info(up, (unsigned int*)arg);

	case TIOCMBIS		:
	case TIOCMBIC		:
	case TIOCMSET		:
#ifdef MODBUS_TXENB
	case MODBUS_TXENB	:
#endif
		return xtm8250_set_modem_info(up, cmd, (unsigned int*)arg);

	case TIOCSERGETLSR: /* Get line status register */
		return xtm8250_get_lsr_info(up, (unsigned int*)arg);

	/* NB
	 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
	 * - mask passed in arg for lines of interest
	 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
	 * Caller should use TIOCGICOUNT to see which one it was
	 * ****************************************************** */
	case TIOCMIWAIT:
		spin_lock_irqsave(&up->port.lock, flags);
		/* note the counters on entry */
		cprev = up->port.icount;
		spin_unlock_irqrestore(&up->port.lock, flags);
		while (1) {
			interruptible_sleep_on(&(up->port.state->delta_msr_wait));
			/* see if a signal did it */
			if (signal_pending(current))
				return -ERESTARTSYS;
			spin_lock_irqsave(&up->port.lock, flags);
			cnow = up->port.icount; /* atomic copy */
			spin_unlock_irqrestore(&up->port.lock, flags);

			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
			    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO; /* no change => error */

			if ( ((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
			     ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
			     ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
			     ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
				return 0;
			}
			cprev = cnow;

		} /* while */

	/* Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
	 * Return: write counters to the user passed counter struct
	 * NB: both 1->0 and 0->1 transitions are counted except for
	 *     RI where only 0->1 is counted.
	 * ****************************************************** */
	case TIOCGICOUNT:
		spin_lock_irqsave(&up->port.lock, flags);
		cnow = up->port.icount;
		spin_unlock_irqrestore(&up->port.lock, flags);
		p_cuser = (struct serial_icounter_struct *) arg;

		error = put_user(cnow.cts, &p_cuser->cts);
		if (error)
			return error;

		error = put_user(cnow.dsr, &p_cuser->dsr);
		if (error)
			return error;

		error = put_user(cnow.rng, &p_cuser->rng);
		if (error)
			return error;

		error = put_user(cnow.dcd, &p_cuser->dcd);
		if (error)
			return error;

		error = put_user(cnow.rx, &p_cuser->rx);
		if (error)
			return error;

		error = put_user(cnow.tx, &p_cuser->tx);
		if (error)
			return error;

		error = put_user(cnow.frame, &p_cuser->frame);
		if (error)
			return error;

		error = put_user(cnow.overrun, &p_cuser->overrun);
		if (error)
			return error;

		error = put_user(cnow.parity, &p_cuser->parity);
		if (error)
			return error;

		error = put_user(cnow.brk, &p_cuser->brk);
		if (error)
			return error;

		error = put_user(cnow.buf_overrun, &p_cuser->buf_overrun);
		if (error)
			return error;

		return 0;

	default:
		DEBUG_GENERAL("%s: Sir, I don't know cmd: 0x%x\n", __FUNCTION__, cmd);
		return -ENOIOCTLCMD;

	}	/* switch */

	return 0;
}

/*************************************************************************/

/* Func: xtm8250_startup()
 * Note: Bring forward session: "Clear the interrupt registers again"
 * ************************************************************/
static int xtm8250_startup(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;
	unsigned char lsr, iinfo;
	int retval;

	up->capabilities = uart_config[up->port.type].flags;
	up->mcr = 0;

	/* DN. 11/2009
	 * - Remember to set fifosize & xmit size properly
	 * **********************************************/
	up->port.fifosize = (unsigned char)(uart_config[up->port.type].fifo_size);
	up->tx_loadsz	  = (unsigned int) (uart_config[up->port.type].tx_loadsz);

	xtm8250_clear_fifos(up);

	serial_inp(up, UART_LSR);
	serial_inp(up, UART_RX);
	serial_inp(up, UART_IIR);
	serial_inp(up, UART_MSR);

	/*
	 * At this point, there's no way the LSR could still be 0xff;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (unlikely(!(up->port.flags & UPF_BUGGY_UART) && \
				(serial_inp(up, UART_LSR) == 0xff))) {
		printk(KERN_WARNING "ttyS%d: LSR safety check engaged!\n", up->port.line);
		return -ENODEV;
	}

	if (is_real_interrupt(up->port.irq)) {
		up->timer.function = xtm8250_backup_timeout;
		up->timer.data = (unsigned long)up;
	}

	/*
	 * If the "interrupt" for this port doesn't correspond with any
	 * hardware interrupt, we use a timer-based system.  The original
	 * driver used to do this with IRQ0.
	 */
	if (unlikely(!is_real_interrupt(up->port.irq))) {
		up->timer.data = (unsigned long)up;
	} else {
		retval = serial_link_irq_chain(up);
		if (retval)
			return retval;
	}

	/* Now, initialize the UART */
	serial_outp(up, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&up->port.lock, flags);
	if (up->port.flags & UPF_FOURPORT) {
		if (!is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT1;
	} else
		/*
		 * Most PC uarts need OUT2 raised to enable interrupts.
		 */
		if (is_real_interrupt(up->port.irq))
			up->port.mctrl |= TIOCM_OUT2;

	xtm8250_set_mctrl(&up->port, up->port.mctrl);

	/* Serial over Lan (SoL) hack:
	   Intel 8257x Gigabit ethernet chips have a
	   16550 emulation, to be used for Serial Over Lan.
	   Those chips take a longer time than a normal
	   serial device to signalize that a transmission
	   data was queued. Due to that, the above test generally
	   fails. One solution would be to delay the reading of
	   iir. However, this is not reliable, since the timeout
	   is variable. So, let's just don't test if we receive
	   TX irq. This way, we'll never enable UART_BUG_TXEN.
	 */
	if (skip_txen_test)
		goto dont_test_tx_en;

	/*
	 * Do a quick test to see if we receive an
	 * interrupt when we enable the TX irq.
	 */
	serial_outp(up, UART_IER, UART_IER_THRI);	/* Enable Transmitter holding register int. */
	lsr = serial_inp(up, UART_LSR);
	iinfo = serial_inp(up, UART_IIR);
	serial_outp(up, UART_IER, 0);				/* Disable all interrupt	*/

	if ( (lsr & UART_LSR_TEMT) && (iinfo & UART_IIR_NO_INT) ) {
		if (!(up->bugs & UART_BUG_TXEN)) {
			up->bugs |= UART_BUG_TXEN;
			printk (KERN_WARNING "ttyS%d - enabling bad tx status workarounds\n", port->line);
		}
	} else {
		up->bugs &= ~UART_BUG_TXEN;
	}

dont_test_tx_en:
	spin_unlock_irqrestore(&up->port.lock, flags);

	/* Bring forward this session
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_inp(up, UART_LSR);
	serial_inp(up, UART_RX);
	serial_inp(up, UART_IIR);
	serial_inp(up, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_outp(up, UART_IER, up->ier);

	if (unlikely(up->port.flags & UPF_FOURPORT)) {
		unsigned int icp;
		/*
		 * Enable interrupts on the AST Fourport board
		 */
		icp = (up->port.iobase & 0xfe0) | 0x01f;
		outb_p(0x80, icp);
		(void) inb_p(icp);
	}
	/*****************************************/
  mod_timer(&up->timer, jiffies + poll_timeout(up->port.timeout));
	return 0;
}

static void xtm8250_shutdown(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned long flags;


	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	if (unlikely(up->port.flags & UPF_FOURPORT)) {
		/* reset interrupts on the AST Fourport board */
		inb((up->port.iobase & 0xfe0) | 0x1f);
		up->port.mctrl |= TIOCM_OUT1;
	} else
		up->port.mctrl &= ~TIOCM_OUT2;

	xtm8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_outp(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	xtm8250_clear_fifos(up);

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_inp(up, UART_RX);

	del_timer_sync(&up->timer);
	up->timer.function = xtm8250_timeout;
	if (unlikely(is_real_interrupt(up->port.irq)))
		serial_unlink_irq_chain(up);
}

static unsigned int xtm8250_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	/*
	 * Handle magic divisors for baud rates above baud_base on
	 * SMSC SuperIO chips.
	 */
	if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
	    baud == (port->uartclk/4))
		quot = 0x8001;
	else if ((port->flags & UPF_MAGIC_MULTIPLIER) &&
		baud == (port->uartclk/8))
		quot = 0x8002;
	else
		quot = uart_get_divisor(port, baud);

	return quot;
}

static void
xtm8250_set_termios(struct uart_port *port,	  \
					  struct termios *termios,\
					  struct termios *old)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char cval, fcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;

	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;

	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/* Div Calc */
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	quot = xtm8250_get_divisor(port, baud);

	DEBUG_GENERAL("%s: baud:%d, quot:%d\n", __FUNCTION__, baud, quot);

	/*
	 * Oxford Semi 952 rev B workaround
	 */
	if (up->bugs & UART_BUG_QUOT && (quot & 0xff) == 0)
		quot ++;

	if ( (up->capabilities & UART_CAP_FIFO) && (up->port.fifosize > 1) ) {
		if (baud < 2400)
			fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
		else
			fcr = uart_config[up->port.type].fcr;
	}
	/*
	 * MCR-based auto flow control.  When AFE is enabled, RTS will be
	 * deasserted when the receive FIFO contains more characters than
	 * the trigger, or the MCR RTS bit is cleared.  In the case where
	 * the remote UART is not using CTS auto flow control, we must
	 * have sufficient FIFO entries for the latency of the remote
	 * UART to respond.  IOW, at least 32 bytes of FIFO.
	 */
	if (up->capabilities & UART_CAP_AFE && up->port.fifosize >= 32) {
		up->mcr &= ~UART_MCR_AFE;
		if (termios->c_cflag & CRTSCTS)
			up->mcr |= UART_MCR_AFE;
	}

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;

	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/* NB
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 * *********************************************/
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	up->ier &= ~UART_IER_MSI;
	if (!(up->bugs & UART_BUG_NOMSR) &&
	        UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	if (up->capabilities & UART_CAP_UUE)
		up->ier |= UART_IER_UUE | UART_IER_RTOIE;

	serial_outp(up, UART_IER, up->ier);

	if (up->capabilities & UART_CAP_EFR) {
		unsigned char efr = 0;
		/*
		 * TI16C752/Startech hardware flow control.  FIXME:
		 * - TI16C752 requires control thresholds to be set.
		 * - UART_MCR_RTS is ineffective if auto-RTS mode is enabled.
		 */
		if (termios->c_cflag & CRTSCTS)
			efr |= UART_EFR_CTS;

		serial_outp(up, UART_LCR, 0xBF);
		serial_outp(up, UART_EFR, efr);
	}

	if (up->capabilities & UART_NATSEMI) {
		/* Switch to bank 2 not bank 1, to avoid resetting EXCR2 */
		serial_outp(up, UART_LCR, 0xe0);
	} else {
		serial_outp(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */
	}

	serial_outp(up, UART_DLL, quot & 0xff);		/* LS of divisor */
	serial_outp(up, UART_DLM, quot >> 8);		/* MS of divisor */

	serial_outp(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;							/* Save LCR */
	if (up->port.type != PORT_16750) {
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		}
		serial_outp(up, UART_FCR, fcr);		/* set fcr */
	}

	xtm8250_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static void
xtm8250_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_8250_port *p = (struct uart_8250_port *)port;

	xtm8250_set_sleep(p, state != 0);

	if (p->pm)
		p->pm(port, state, oldstate);
}

/*
 * Resource handling.
 */
static int xtm8250_request_std_resource(struct uart_8250_port *up)
{
	unsigned int size = 8 << up->port.regshift;
	int ret = 0;

	switch (up->port.iotype) {
	case UPIO_MEM:
		if (!up->port.mapbase) {
			up->port.mapbase = up->port.iobase;
		}
		if (!up->port.mapbase) {
		/*
			DEBUG_GENERAL("%s:empty mapbase %lx\n",\
					__FUNCTION__, up->port.mapbase);
		*/
			break;
		}

		up->port.mapbase |= isabase;
	/*
		DEBUG_GENERAL("%s: mapbase %lx\n", \
					__FUNCTION__, up->port.mapbase);
	*/
		if (!request_mem_region(up->port.mapbase, size, "xtmserial")) {
			ret = -EBUSY;
			break;
		}

		if (up->port.flags & UPF_IOREMAP) {
			up->port.membase = ioremap(up->port.mapbase, size);
			if (!up->port.membase) {
				release_mem_region(up->port.mapbase, size);
				ret = -ENOMEM;
			}
		}
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void xtm8250_release_std_resource(struct uart_8250_port *up)
{
	unsigned int size = 8 << up->port.regshift;

	switch (up->port.iotype) {
	case UPIO_MEM:
		if (!up->port.mapbase)
			break;

		if (up->port.flags & UPF_IOREMAP) {
			iounmap(up->port.membase);
			up->port.membase = NULL;
		}

		release_mem_region(up->port.mapbase, size);
		break;

	case UPIO_HUB6:
	case UPIO_PORT:
		break;
	}
}

static void xtm8250_release_port(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	xtm8250_release_std_resource(up);
}

static int xtm8250_request_port(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	return xtm8250_request_std_resource(up);
}

static void xtm8250_config_port(struct uart_port *port, int flags)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	int probeflags = PROBE_ANY;
	int ret;

	DEBUG_GENERAL("%s: port %p flags %d\n", __FUNCTION__, port, flags);

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = xtm8250_request_std_resource(up);
	DEBUG_GENERAL("%s: resource ret %d\n", __FUNCTION__, ret);
	if (ret < 0)
		return;

	if (flags & UART_CONFIG_TYPE)
		autoconfig(up, probeflags);

	if (up->port.type != PORT_UNKNOWN && flags & UART_CONFIG_IRQ)
		autoconfig_irq(up);

	if (up->port.type == PORT_UNKNOWN)
		xtm8250_release_std_resource(up);
}

static int
xtm8250_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int j, gpio_pin=0;

	/* get gpio_pin corresponding to ISA IRQ */
	for ( j=0 ; isa_gpio[j][0] ; j++ ) {
		if (isa_gpio[j][0] == ser->irq) {
			gpio_pin = isa_gpio[j][1];
			break;
		}
	}

	DEBUG_GENERAL("ISA irq = %d, GPIO pin = %d, GPIO IRQ %d\n",
			ser->irq, gpio_pin, IRQ_GPIO(gpio_pin));

	if (gpio_pin == 0) {
		/* Can not find gpio pin? Resolve it now... D.N 11/2009
		 * Need to check if ser->irq was previously twiddled.
		 * 	a.If yes, skip irq twiddling & continues with verifying
		 * 	b.If not, just return error. DN. 11/2009
		 * *************************************************/
		for ( j=0 ; isa_gpio[j][1] ; j++ ) {
			if ( IRQ_GPIO(isa_gpio[j][1]) == ser->irq) {
				DEBUG_GENERAL("Skip twiddling irq...\n");
				goto VERIFY;
			}
		}

		printk(KERN_WARNING "Verify failed - GPIO pin not found\n");
		return(-1);
	}

	DEBUG_GENERAL("Twiddling irq\n");
	pxa_gpio_mode(gpio_pin | GPIO_IN);
	/* IRQ_GPIO(pin) yields the IRQ number seen in
	 * cat /proc/interrupts
	 */
	ser->irq = IRQ_GPIO(gpio_pin);

VERIFY:
	/* Real stuff */
	if (ser->irq >= NR_IRQS || ser->irq < 0 ||
	    ser->baud_base < 9600 || ser->type < PORT_UNKNOWN ||
	    ser->type >= ARRAY_SIZE(uart_config) || ser->type == PORT_CIRRUS ||
	    ser->type == PORT_STARTECH) {
		printk (KERN_WARNING "verify failed\n");
		return -EINVAL;
	}
	DEBUG_GENERAL("verify OK\n");
	return 0;
}

static const char *
xtm8250_type(struct uart_port *port)
{
	int type = port->type;

	if (type >= ARRAY_SIZE(uart_config))
		type = 0;
	return uart_config[type].name;
}

static struct uart_ops xtm8250_ops = {
	.tx_empty		= xtm8250_tx_empty,
	.set_mctrl		= xtm8250_set_mctrl,
	.get_mctrl		= xtm8250_get_mctrl,
	.stop_tx		= xtm8250_stop_tx,
	.start_tx		= xtm8250_start_tx,
	.stop_rx		= xtm8250_stop_rx,
	.enable_ms		= xtm8250_enable_ms,
	.break_ctl		= xtm8250_break_ctl,
	.startup		= xtm8250_startup,
	.shutdown		= xtm8250_shutdown,
	.set_termios	= xtm8250_set_termios,
	.pm				= xtm8250_pm,
	.type			= xtm8250_type,
	.release_port	= xtm8250_release_port,
	.request_port	= xtm8250_request_port,
	.config_port	= xtm8250_config_port,
	.verify_port	= xtm8250_verify_port,
	.ioctl			= xtm8250_ioctl,
};

static struct uart_8250_port xtm8250_ports[UART_NR];

static void __init xtm8250_isa_init_ports(void)
{
	struct uart_8250_port *up;
	static int first = 1;
	int i;

	if (!first)
		return;
	first = 0;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &xtm8250_ports[i];

		up->port.line = i;
		spin_lock_init(&up->port.lock);

		init_timer(&up->timer);
		up->timer.function = xtm8250_timeout;

		/*
		 * ALPHA_KLUDGE_MCR needs to be killed.
		 */
		up->mcr_mask = ~ALPHA_KLUDGE_MCR;
		up->mcr_force = ALPHA_KLUDGE_MCR;

		up->port.ops = &xtm8250_ops;
	}

	for (i = 0, up = xtm8250_ports; i < UART_NR; i++, up++) {
		up->port.iobase   = 0;
		up->port.irq      = 0;
		up->port.uartclk  = 921600 * 16;
		up->port.flags    = UPF_IOREMAP;
		up->port.hub6     = 0;
		up->port.membase  = 0;
		up->port.iotype   = UPIO_MEM;
		up->port.regshift = 0;
	}
}

static void __init
xtm8250_register_ports(struct uart_driver *drv, struct device *dev)
{
	int i;

	xtm8250_isa_init_ports();

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &xtm8250_ports[i];

		up->port.dev = dev;
		uart_add_one_port(drv, &up->port);
	}
}

#define SERIAL8250_CONSOLE	NULL

static struct uart_driver xtm8250_reg = {
	.owner			= THIS_MODULE,
	.driver_name	= "xtm serial",
	.devfs_name		= "tts/",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 100,
	.nr				= UART_NR,
	.cons			= SERIAL8250_CONSOLE,
};

/*
 * Func: early_serial_setup() - early registration for 8250 ports
 * NB -
 * Setup an 8250 port structure prior to console initialisation.  Use
 * after console initialisation will cause undefined behaviour.
 */
int __init early_serial_setup(struct uart_port *port)
{
	if (port->line >= ARRAY_SIZE(xtm8250_ports))
		return -ENODEV;

	xtm8250_isa_init_ports();
	xtm8250_ports[port->line].port		= *port;
	xtm8250_ports[port->line].port.ops	= &xtm8250_ops;
	return 0;
}

void xtm8250_suspend_port(int line)
{
	uart_suspend_port(&xtm8250_reg, &xtm8250_ports[line].port);
}

/* NB
 * Added NATSEMI support stuff
 * ******************************/
void xtm8250_resume_port(int line)
{
	struct uart_8250_port *up = &xtm8250_ports[line];

	if (up->capabilities & UART_NATSEMI) {
		unsigned char tmp;

		/* Ensure it's still in high speed mode */
		serial_outp(up, UART_LCR, 0xE0);

		tmp = serial_inp(up, UART_MCR); /* EXCR2 */
		tmp &= ~0xB0; /* Disable LOCK, mask out PRESL[01] */
		tmp |= 0x10;  /* 1.625 divisor for baud_base --> 921600 */
		serial_outp(up, UART_MCR, tmp);

		serial_outp(up, UART_LCR, 0);
	}
	uart_resume_port(&xtm8250_reg, &up->port);
}

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int __devinit xtm8250_probe(struct platform_device *dev)
{
	struct plat_serial8250_port *p = dev->dev.platform_data;
	struct uart_port port;
	int ret, i;

/*
	DEBUG_GENERAL("%s: %p -> %x -> %x\n", __FUNCTION__,\
			p, p?(unsigned int)(p->membase):-1, p?p->flags:-1);
*/
	memset(&port, 0, sizeof(struct uart_port));

	for (i = 0; p && p->flags != 0; p++, i++) {
		port.iobase		= p->iobase;
		port.membase	= p->membase;
		port.irq		= p->irq;
		port.uartclk	= p->uartclk;
		port.regshift	= p->regshift;
		port.iotype		= p->iotype;
		port.flags		= p->flags;
		port.mapbase	= p->mapbase;
		port.hub6		= p->hub6;
		port.dev		= &dev->dev;
		if (share_irqs)
			port.flags |= UPF_SHARE_IRQ;
		ret = xtm8250_register_port(&port);
		if (ret < 0) {
			dev_err(&dev->dev, "unable to register port at index %d "
				"(IO%lx MEM%lx IRQ%d): %d\n", i,
				p->iobase, p->mapbase, p->irq, ret);
		}

		DEBUG_GENERAL("RegPort=%d, IRQ=%d\n",i, port.irq);
	}

	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int __devexit xtm8250_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &xtm8250_ports[i];

		if (up->port.dev == &dev->dev)
			xtm8250_unregister_port(i);
	}
	return 0;
}

static int xtm8250_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &xtm8250_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			uart_suspend_port(&xtm8250_reg, &up->port);
	}

	return 0;
}

/* Func: xtm8250_resume()
 * NB:
 * 		Call xtm8250_resume_port() instead
 * 		of uart_resume_port() for NATSEMI support
 * ******************************************************/
static int xtm8250_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_8250_port *up = &xtm8250_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			xtm8250_resume_port(i);
	}

	return 0;
}

static struct platform_driver xtm8250_isa_driver = {
	.probe		= xtm8250_probe,
	.remove		= __devexit_p(xtm8250_remove),
	.suspend	= xtm8250_suspend,
	.resume		= xtm8250_resume,
	.driver		= {
	.name		= "xtm8250",
	},
};

/*
 * This "device" covers _all_ ISA 8250-compatible serial devices listed
 * in the table in include/asm/serial.h
 */
static struct platform_device *xtm8250_isa_devs;

/*
 * xtm8250_register_port and xtm8250_unregister_port allows for
 * 16x50 serial ports to be configured at run-time, to support PCMCIA
 * modems and PCI multiport cards.
 */
static DECLARE_MUTEX(serial_sem);

static struct uart_8250_port *xtm8250_find_match_or_unused(struct uart_port *port)
{
	int i;

	/*
	 * First, find a port entry which matches.
	 */
	for (i = 0; i < UART_NR; i++)
		if (uart_match_port(&xtm8250_ports[i].port, port))
			return &xtm8250_ports[i];

	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < UART_NR; i++)
		if (xtm8250_ports[i].port.type == PORT_UNKNOWN &&
		    xtm8250_ports[i].port.iobase == 0)
			return &xtm8250_ports[i];

	/*
	 * That also failed.  Last resort is to find any entry which
	 * doesn't have a real port associated with it.
	 */
	for (i = 0; i < UART_NR; i++)
		if (xtm8250_ports[i].port.type == PORT_UNKNOWN)
			return &xtm8250_ports[i];

	return NULL;
}

/**
 *	xtm8250_register_port - register a serial port
 *	@port: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int xtm8250_register_port(struct uart_port *port)
{
	struct uart_8250_port *uart;
	int ret = -ENOSPC;

	DEBUG_GENERAL("%s: %p\n",__FUNCTION__, port->membase);

	if (port->uartclk == 0) {
		printk(KERN_WARNING "%s: no clock\n", __FUNCTION__);
		return -EINVAL;
	}

	down(&serial_sem);

	uart = xtm8250_find_match_or_unused(port);
	if (uart) {
		DEBUG_GENERAL("%s: uart %p\n", __FUNCTION__, uart);
		uart_remove_one_port(&xtm8250_reg, &uart->port);

		uart->port.iobase   = port->iobase;
		uart->port.membase  = port->membase;
		uart->port.irq      = port->irq;
		uart->port.uartclk  = port->uartclk;
		uart->port.fifosize = port->fifosize;
		uart->port.regshift = port->regshift;
		uart->port.iotype   = port->iotype;
		uart->port.flags    = port->flags | UPF_BOOT_AUTOCONF;
		uart->port.mapbase  = port->mapbase;
		if (port->dev)
			uart->port.dev = port->dev;

		ret = uart_add_one_port(&xtm8250_reg, &uart->port);
		if (ret == 0)
			ret = uart->port.line;
	} else {
		printk(KERN_WARNING "%s: null uart\n",__FUNCTION__);
	}
	up(&serial_sem);



	return ret;
}
EXPORT_SYMBOL(xtm8250_register_port);

/**
 *	xtm8250_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void xtm8250_unregister_port(int line)
{
	struct uart_8250_port *uart = &xtm8250_ports[line];

	down(&serial_sem);
	uart_remove_one_port(&xtm8250_reg, &uart->port);
	if (xtm8250_isa_devs) {
		uart->port.flags   &= ~UPF_BOOT_AUTOCONF;
		uart->port.type		= PORT_UNKNOWN;
		uart->port.dev		= &xtm8250_isa_devs->dev;
		uart_add_one_port(&xtm8250_reg, &uart->port);
	} else {
		uart->port.dev		= NULL;
	}
	up(&serial_sem);
}
EXPORT_SYMBOL(xtm8250_unregister_port);

static int __init xtm8250_init(void)
{
	int ret, i;


	printk(KERN_INFO "RTU-enabled xtm8250/16x50 serial driver. [Jun 2010], "
		"%d ports, IRQ sharing %sabled, IRQNR=%d\n", (int) UART_NR,
		share_irqs ? "en" : "dis", NR_IRQS);

	for (i = 0; i < NR_IRQS; i++)
		spin_lock_init(&irq_lists[i].lock);

	ret = uart_register_driver(&xtm8250_reg);
	if (ret) {
		printk(KERN_ERR "uart_register_driver: failed %d\n", ret);
		goto out;
	}

	xtm8250_isa_devs = platform_device_register_simple("xtm8250",
					 PLAT8250_DEV_LEGACY, NULL, 0);
	if (IS_ERR(xtm8250_isa_devs)) {
		ret = PTR_ERR(xtm8250_isa_devs);
		printk(KERN_ERR "platform_device_register_simple: failed %x\n", ret);
		goto unreg;
	}

	xtm8250_register_ports(&xtm8250_reg, &xtm8250_isa_devs->dev);

	ret = platform_driver_register(&xtm8250_isa_driver);
	if (ret == 0)
		goto out;

	printk(KERN_ERR "platform_driver_register: failed %x\n", ret);

	platform_device_unregister(xtm8250_isa_devs);
 unreg:
	uart_unregister_driver(&xtm8250_reg);
 out:

	return ret;
}

static void __exit xtm8250_exit(void)
{
	struct platform_device *isa_dev = xtm8250_isa_devs;

	/*
	 * This tells xtm8250_unregister_port() not to re-register
	 * the ports (thereby making xtm8250_isa_driver permanently
	 * in use.)
	 */
	xtm8250_isa_devs = NULL;

	platform_driver_unregister(&xtm8250_isa_driver);
	platform_device_unregister(isa_dev);

	uart_unregister_driver(&xtm8250_reg);
}

module_init(xtm8250_init);
module_exit(xtm8250_exit);

EXPORT_SYMBOL(xtm8250_suspend_port);
EXPORT_SYMBOL(xtm8250_resume_port);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Rtu-enabled xtm8250/16x50 serial driver. [Jun 2010]");

module_param(share_irqs, uint, 0644);
MODULE_PARM_DESC(share_irqs, "Share IRQs with other non-8250/16550 devices" " (unsafe)");

module_param(isabase, uint, 0644);
MODULE_PARM_DESC(isabase, "Base addr for ISA stuff");

module_param(skip_txen_test, uint, 0644);
MODULE_PARM_DESC(skip_txen_test, "Skip checking for the TXEN bug at init time");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
