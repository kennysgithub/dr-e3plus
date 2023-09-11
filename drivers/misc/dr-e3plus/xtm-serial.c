// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Dynamic Ratings
 *
 */

#define DEBUG
#include <linux/console.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <uapi/linux/serial_reg.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/delay.h>
#include <linux/tty_flip.h>
#include <linux/sched/signal.h>

#include "../../tty/serial/8250/8250.h"

#include "ftdi_isa.h"
#include "xtm-serial-modbus.h"

/* MX25_PAD_CSI_D5__GPIO_1_30              0x80000000 */     /* ISA IRQ 6 */
/* MX25_PAD_CSI_D4__GPIO_1_29              0x80000000 */     /* ISA IR5 5 */
#define BANK_0_IRQ_GPIO		(29)
#define BANK_1_IRQ_GPIO		(30)
#define NUM_SERIAL_PORTS	(8)

#define XTM_UART_CLOCK             (1843200)

struct xtm_serial_port_mappings {
	u16 port_base;
	int irqno;
};

static const struct xtm_serial_port_mappings port_maps[NUM_SERIAL_PORTS] = {
	{ 0x2000, BANK_0_IRQ_GPIO },
	{ 0x2008, BANK_0_IRQ_GPIO },
	{ 0x2010, BANK_0_IRQ_GPIO },
	{ 0x2018, BANK_0_IRQ_GPIO },
	{ 0x3000, BANK_1_IRQ_GPIO },
	{ 0x3008, BANK_1_IRQ_GPIO },
	{ 0x3010, BANK_1_IRQ_GPIO },
	{ 0x3018, BANK_1_IRQ_GPIO },
};

#define PORT_TO_ADDR(port) (unsigned char *)((port_maps[port].port_base) | 0L)

static const struct tty_port_operations xtm_serial_port_ops;

static struct tty_driver *xtm_serial_driver;

struct xtm_serial_port {
	struct tty_port		tty_port;
	unsigned		custom_divisor;
	unsigned		baud;
	/* fake; only needed for quirks, "io" and ident */
	struct uart_8250_port	up;
	struct mutex		port_mutex;
};

struct xtm_serial_device {
	struct xtm_serial_port	xtm_ports[NUM_SERIAL_PORTS];
	/* FIXME? Ever more than these two? */
	unsigned irq0;
	unsigned irq1;
	unsigned cur_irq;

	struct workqueue_struct	*irq_wq;
	struct work_struct	irq_work;
	struct mutex		io_mutex;
	unsigned		opened_ports;
};

// FIXME: What *are* these E3 ports considered to be?
static const struct serial8250_config uart_config[] = {
	[PORT_16550A] = {
	    .name           = "16550A",
	    .fifo_size      = 16,
	    .tx_loadsz      = 16,
	    .fcr            = UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_10,
	    .rxtrig_bytes   = {1, 4, 8, 14},
	    .flags          = UART_CAP_FIFO,
	},
};

static void serial_outp(struct uart_8250_port *up, unsigned reg, unsigned value)
{
	ftdi_isa_write(value, up->port.membase + reg, 0);
}

static unsigned int serial_inp(struct uart_8250_port *up, unsigned reg)
{
	return ftdi_isa_read(up->port.membase + reg, 0);
}

/*
 * FIFO support.
 */
static inline void xtm_clear_fifos(struct uart_8250_port *p)
{
	if (p->capabilities & UART_CAP_FIFO) {
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_outp(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_outp(p, UART_FCR, 0);
	}
}

static unsigned int xtm_get_divisor(struct uart_port *port, unsigned int baud)

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

static unsigned int check_modem_status(struct uart_8250_port *up)
{
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);
	struct tty_port *ttyp = &xsp->tty_port;
	struct device *dev = ttyp->tty->dev;
	unsigned int status = serial_inp(up, UART_MSR);

	dev_dbg(dev, "%s():\n", __func__);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;

	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD) {
			dev_info(dev, "%s(): UART_MSR_DDCD\n", __func__);
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		}
		if (status & UART_MSR_DCTS) {
			dev_info(dev, "%s(): UART_MSR_DCTS\n", __func__);
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);
		}
	}
	return status;
}

static unsigned int xtm_get_mctrl(struct uart_port *port)
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
	pr_debug("%s(): mcr %0x02X\n", __func__, ret);
	return ret;
}

static void xtm_set_mctrl(struct uart_port *port, unsigned int mctrl)
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

	pr_debug("%s(): mcr 0x%02X\n", __func__, mcr);
	serial_outp(up, UART_MCR, mcr);
}

static int xtm_serial_startup(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	unsigned char lsr, iinfo;
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);

	up->port.fifosize = uart_config[up->port.type].fifo_size;
	up->tx_loadsz = uart_config[up->port.type].tx_loadsz;

	xtm_clear_fifos(up);

	serial_inp(up, UART_LSR);
	serial_inp(up, UART_RX);
	serial_inp(up, UART_IIR);
	serial_inp(up, UART_MSR);

	if (unlikely(serial_inp(up, UART_LSR) == 0xFF))
		return -ENODEV;

	serial_outp(up, UART_LCR, UART_LCR_WLEN8);

	mutex_lock(&xsp->port_mutex);

	up->port.mctrl |= TIOCM_OUT2;
	xtm_set_mctrl(&up->port, up->port.mctrl);

	serial_outp(up, UART_IER, UART_IER_THRI);
	lsr = serial_inp(up, UART_LSR);
	iinfo = serial_inp(up, UART_IIR);
	serial_outp(up, UART_IER, 0);

	if ((lsr & UART_LSR_TEMT) && (iinfo & UART_IIR_NO_INT)) {
		pr_err("%s(): no IRQ on port %d, disabling\n",
			__func__, xsp->tty_port.tty->index);
		return -ENODEV;
	}
	mutex_unlock(&xsp->port_mutex);

	serial_inp(up, UART_LSR);
	serial_inp(up, UART_RX);
	serial_inp(up, UART_IIR);
	serial_inp(up, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_outp(up, UART_IER, up->ier);

	return 0;
}

static void xtm_serial_shutdown(struct uart_port *port)
{
	struct uart_8250_port *up = (struct uart_8250_port *)port;
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);

	up->ier = 0;
	serial_outp(up, UART_IER, 0);

	mutex_lock(&xsp->port_mutex);
	up->port.mctrl &= ~TIOCM_OUT2;
	xtm_set_mctrl(&up->port, up->port.mctrl);
	mutex_unlock(&xsp->port_mutex);

	serial_outp(up, UART_LCR, serial_inp(up, UART_LCR) & ~UART_LCR_SBC);
	xtm_clear_fifos(up);

	serial_inp(up, UART_RX);
}

static int xtm_serial_open(struct tty_struct *tty, struct file *filp)
{
	unsigned int line = tty->index;
	struct xtm_serial_device *xsd = tty->driver->driver_state;
	struct xtm_serial_port *xsp = &xsd->xtm_ports[line];
	int ret;

	xsp->up.port.membase = PORT_TO_ADDR(line);
	xsp->up.port.uartclk = XTM_UART_CLOCK;
	xsp->up.port.type    = PORT_16550A;
	xsp->up.capabilities = UART_CAP_FIFO;

	xsp->up.port.get_mctrl = xtm_get_mctrl;
	xsp->up.port.set_mctrl = xtm_set_mctrl;
	xsp->up.port.startup = xtm_serial_startup;
	xsp->up.port.shutdown = xtm_serial_shutdown;

	pr_debug("%s(): line %d\n", __func__, line);

	ret = tty_port_open(&xsp->tty_port, tty, filp);
	if (ret)
		return ret;

	mutex_init(&xsp->port_mutex);

	xsd->opened_ports |= (1 << line);
	ret = xtm_serial_startup((struct uart_port *) &xsp->up);
	if (ret) {
		xsd->opened_ports &= ~(1 << line);
		return ret;
	}

	return 0;
}

static void xtm_serial_close(struct tty_struct *tty, struct file *filp)
{
	unsigned int line = tty->index;
	struct xtm_serial_device *xsd = tty->driver->driver_state;
	struct xtm_serial_port *xsp = &xsd->xtm_ports[line];

	dev_dbg(tty->dev, "%s(): %d\n", __func__, __LINE__);

	xsd->opened_ports &= ~(1 << line);
	xtm_serial_shutdown((struct uart_port *) &xsp->up);

	tty_port_close(tty->port, tty, filp);
	mutex_destroy(&xsp->port_mutex);
}

static void xtm_serial_hangup(struct tty_struct *tty)
{
	dev_dbg(tty->dev, "%s(): %d\n", __func__, __LINE__);
	tty_port_hangup(tty->port);
}

static int xtm_serial_write(struct tty_struct *tty, const unsigned char *buf,
			 int count)
{
	unsigned int line = tty->index;
	struct xtm_serial_device *xsd = tty->driver->driver_state;
	struct xtm_serial_port *xsp = &xsd->xtm_ports[line];
	struct uart_8250_port *up = (struct uart_8250_port *) &xsp->up;
	int ret;
	unsigned const char *p = buf;

	dev_dbg(tty->dev, "%s(): %d line %d count %d\n", __func__, __LINE__,
		line, count);

	/*
	ret = ftdi_isa_write_multiple((u8 *)buf, up->port.membase + UART_TX, count);
	*/
	ret = count;
	while(count--)
		serial_outp(up, UART_TX, *p++);
	return ret;
}

static int xtm_serial_write_room(struct tty_struct *tty)
{
	dev_dbg(tty->dev, "%s(): %d\n", __func__, __LINE__);
	// FIXME: the problem is there's no way to determine the FIFO depth :(
	return 4;
}

static void xtm_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{

	unsigned int line = tty->index;
	struct xtm_serial_device *xsd = tty->driver->driver_state;
	struct xtm_serial_port *xsp = &xsd->xtm_ports[line];
	struct uart_8250_port *up = (struct uart_8250_port *) &xsp->up;

	unsigned char cval, fcr = 0;
	unsigned int baud, quot;
	struct ktermios *termios = &tty->termios;

	dev_dbg(tty->dev, "%s(): %d\n", __func__, __LINE__);

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
	baud = uart_get_baud_rate(&up->port, termios, old_termios, 0, up->port.uartclk/16);
	quot = xtm_get_divisor(&up->port, baud);

	dev_dbg(tty->dev, "%s: baud:%d, quot:%d\n", __func__, baud, quot);

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
	mutex_lock(&xsd->io_mutex);
	uart_update_timeout(&up->port, termios->c_cflag, baud);

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
	up->lcr = cval;					/* Save LCR */
	if (up->port.type != PORT_16750) {
		if (fcr & UART_FCR_ENABLE_FIFO) {
			/* emulated UARTs (Lucent Venus 167x) need two steps */
			serial_outp(up, UART_FCR, UART_FCR_ENABLE_FIFO);
		}
		serial_outp(up, UART_FCR, fcr);		/* set fcr */
	}

	xtm_set_mctrl(&up->port, up->port.mctrl);
	mutex_unlock(&xsd->io_mutex);
}

static int xtm_serial_get_lsr_info(struct uart_8250_port *up, unsigned int *pval)
{
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);
	unsigned int	lsr;
	unsigned int	result;

	mutex_lock(&xsp->port_mutex);
	lsr = serial_inp(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	mutex_unlock(&xsp->port_mutex);

	result = ((lsr & UART_LSR_TEMT) ? TIOCSER_TEMT : 0);
	return put_user(result,pval);
}

static int xtm_serial_get_modem_info(struct uart_8250_port *up, unsigned int *pval)
{
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);
	unsigned char	mcr = up->mcr;
	unsigned char	msr = 0;
	unsigned int	result;

	mutex_lock(&xsp->port_mutex);
	msr = check_modem_status(up);
	mutex_unlock(&xsp->port_mutex);

	result = \
		  ((mcr & UART_MCR_RTS)	 ? TIOCM_RTS	: 0)
		| ((mcr & UART_MCR_DTR)	 ? TIOCM_DTR	: 0)
		| ((mcr & UART_MCR_OUT1) ? TIOCM_OUT1	: 0)
		| ((mcr & UART_MCR_OUT2) ? TIOCM_OUT2	: 0)
		| ((msr & UART_MSR_DCD)	 ? TIOCM_CAR	: 0)
		| ((msr & UART_MSR_RI)	 ? TIOCM_RNG	: 0)
		| ((msr & UART_MSR_DSR)	 ? TIOCM_DSR	: 0)
		| ((msr & UART_MSR_CTS)	 ? TIOCM_CTS	: 0);

	return put_user(result, pval);
}

static int xtm_serial_set_modem_info(struct uart_8250_port *up, unsigned cmd, unsigned *pval)
{
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);
	unsigned int	arg;
	unsigned int	old_mcr;

	int error = get_user(arg, pval);
	if (error)
		return error;

	/* Get current modem control settings */
	up->mcr = up->port.ops->get_mctrl(&up->port);
	old_mcr = up->mcr;

	switch (cmd) {
	case MODBUS_TXENB:
		pr_debug("%s: Enable RTU tx reg %d from %d\n", __func__, UART_SCR, arg);
		mutex_lock(&xsp->port_mutex);
		serial_outp(up, UART_SCR, ( arg ? 1:0 ));
		mutex_unlock(&xsp->port_mutex);
		return 0;

	case TIOCMBIS:
		if (arg & TIOCM_RTS)
			up->mcr |= UART_MCR_RTS;
		if (arg & TIOCM_DTR)
			up->mcr |= UART_MCR_DTR;
		if (arg & TIOCM_OUT1)
			up->mcr |= UART_MCR_OUT1;
		if (arg & TIOCM_OUT2)
			up->mcr |= UART_MCR_OUT2;
		break;

	case TIOCMBIC:
		if (arg & TIOCM_RTS)
			up->mcr &= ~UART_MCR_RTS;
		if (arg & TIOCM_DTR)
			up->mcr &= ~UART_MCR_DTR;
		if (arg & TIOCM_OUT1)
			up->mcr &= ~UART_MCR_OUT1;
		if (arg & TIOCM_OUT2)
			up->mcr &= ~UART_MCR_OUT2;
		break;

	case TIOCMSET:
		up->mcr = ((up->mcr & ~(UART_MCR_RTS  |
					    		UART_MCR_OUT1 |
					    		UART_MCR_OUT2 |
					    		UART_MCR_DTR))
			     | ((arg & TIOCM_RTS)  ? UART_MCR_RTS	: 0)
			     | ((arg & TIOCM_OUT1) ? UART_MCR_OUT1	: 0)
			     | ((arg & TIOCM_OUT2) ? UART_MCR_OUT2	: 0)
			     | ((arg & TIOCM_DTR)  ? UART_MCR_DTR	: 0));
		break;

	default:
		return -EINVAL;
	}

	mutex_lock(&xsp->port_mutex);
	if (old_mcr != up->mcr) {
		up->port.ops->set_mctrl(&(up->port), up->mcr);
	}
	mutex_unlock(&xsp->port_mutex);

	return 0;
}

static int check_port_status_change(unsigned long arg,
	struct xtm_serial_port *xsp, struct uart_icount *cprev, struct uart_icount *cnow)
{
			mutex_lock(&xsp->port_mutex);
			*cnow = xsp->up.port.icount; /* atomic copy */
			mutex_unlock(&xsp->port_mutex);

			if (cnow->rng == cprev->rng && cnow->dsr == cprev->dsr &&
			    cnow->dcd == cprev->dcd && cnow->cts == cprev->cts)
				return -EIO;

			if ( ((arg & TIOCM_RNG) && (cnow->rng != cprev->rng)) ||
			     ((arg & TIOCM_DSR) && (cnow->dsr != cprev->dsr)) ||
			     ((arg & TIOCM_CD)  && (cnow->dcd != cprev->dcd)) ||
			     ((arg & TIOCM_CTS) && (cnow->cts != cprev->cts))) {
				return 0;
			}

			return -EINVAL;	// FIXME
}

static int xtm_serial_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg)
{
	int error;
	unsigned int line = tty->index;
	struct xtm_serial_device *xsd = tty->driver->driver_state;
	struct xtm_serial_port *xsp = &xsd->xtm_ports[line];
	struct uart_8250_port *up = (struct uart_8250_port *) &xsp->up;
	struct tty_port *ttyp = &xsp->tty_port;
	struct device *dev = ttyp->tty->dev;
	struct uart_icount cprev, cnow;			/* icounts - KS	*/
	struct serial_icounter_struct *p_cuser;		/* icounts - US	*/
	unsigned long value = 0;

	dev_dbg(dev, "%s(): cmd 0x%08X, arg 0x%08lX\n", __func__, cmd, arg);

	if ((cmd != TIOCGSERIAL)	&& (cmd != TIOCSSERIAL)		&&
	    (cmd != MODBUS_RTU_GET) 	&& (cmd != MODBUS_RTU_SET)	&&
	    (cmd != TIOCSERCONFIG)	&& (cmd != TIOCSERGSTRUCT)	&&
	    (cmd != TIOCMIWAIT)		&& (cmd != TIOCGICOUNT))
	{	if (tty_io_error(tty))
		    return -EIO;
	}

	switch (cmd) {		/* "asm/ioctls.h" */
	case MODBUS_RTU_GET:
		dev_dbg(dev, "%s(): MODBUS_RTU_GET\n", __func__);
		return put_user((tty->flags & MODBUS_RTU_FLAG) ? 1:0,
						(unsigned long*) arg);

	case MODBUS_RTU_SET:
		dev_dbg(dev, "%s(): MODBUS_RTU_SET\n", __func__);
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
		tty->flags = ((tty->flags & ~MODBUS_RTU_FLAG) |
			(value ? MODBUS_RTU_FLAG : 0));
		return 0;

	case TIOCMGET:
		dev_dbg(dev, "%s(): TIOCMGET\n", __func__);
		return xtm_serial_get_modem_info(up, (unsigned int*) arg);

	case TIOCMBIS		:
	case TIOCMBIC		:
	case TIOCMSET		:
	case MODBUS_TXENB	:
		dev_dbg(dev, "%s(): TIOCMBIS/TIOCMBIC/TIOCMSET/MODBUS_TXENB\n", __func__);
		return xtm_serial_set_modem_info(up, cmd, (unsigned int*) arg);

	case TIOCSERGETLSR: /* Get line status register */
		dev_dbg(dev, "%s(): TIOCSERGETLSR\n", __func__);
		return xtm_serial_get_lsr_info(up, (unsigned int*) arg);

	/* NB
	 * Wait for any of the 4 modem inputs (DCD,RI,DSR,CTS) to change
	 * - mask passed in arg for lines of interest
	 *   (use |'ed TIOCM_RNG/DSR/CD/CTS for masking)
	 * Caller should use TIOCGICOUNT to see which one it was
	 * ****************************************************** */
	case TIOCMIWAIT:
		dev_dbg(dev, "%s(): TIOCMIWAIT\n", __func__);
		mutex_lock(&xsp->port_mutex);
		/* note the counters on entry */
		cprev = up->port.icount;
		mutex_unlock(&xsp->port_mutex);
		while (1) {
			/* interruptible_sleep_on(&(tty->port->delta_msr_wait)); */
			wait_event_interruptible((tty->port->delta_msr_wait),
				check_port_status_change(arg, xsp, &cprev, &cnow));
			/* see if a signal did it */
			if (signal_pending(current))
				return -ERESTARTSYS;
			cprev = cnow;
		}

	/* Get counter of input serial line interrupts (DCD,RI,DSR,CTS)
	 * Return: write counters to the user passed counter struct
	 * NB: both 1->0 and 0->1 transitions are counted except for
	 *     RI where only 0->1 is counted.
	 * ****************************************************** */
	case TIOCGICOUNT:
		dev_dbg(dev, "%s(): TIOCGICOUNT\n", __func__);
		mutex_lock(&xsp->port_mutex);
		cnow = up->port.icount;
		mutex_unlock(&xsp->port_mutex);
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
		pr_debug("%s: unknown cmd 0x%08X\n", __func__, cmd);
		return -ENOIOCTLCMD;

	}

	return 0;
}

static const struct tty_operations xtm_serial_ops = {
	.open = xtm_serial_open,
	.close = xtm_serial_close,
	.hangup = xtm_serial_hangup,
	.write = xtm_serial_write,
	.write_room = xtm_serial_write_room,
	.set_termios = xtm_set_termios,
	.ioctl = xtm_serial_ioctl,
};

static void receive_chars(struct uart_8250_port *up, int *status)
{
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);
	struct tty_port *ttyp = &xsp->tty_port;
	struct device *dev = ttyp->tty->dev;
	unsigned char ch, lsr = *status;
	int cnt = 256;
	char tty_flag;
	int ret;

	do {
		if (likely(lsr & UART_LSR_DR))
			ch = serial_inp(up, UART_RX);

		tty_flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (unlikely(lsr & (UART_LSR_BI | UART_LSR_PE |
                            UART_LSR_FE | UART_LSR_OE))) {
			if (lsr & UART_LSR_BI) {
				lsr &= ~(UART_LSR_FE | UART_LSR_PE);
				up->port.icount.brk++;
				dev_info(dev, "%s(): BREAK recv\n", __func__);
				goto ignore_char;
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
				tty_flag = TTY_BREAK;
			} else if (lsr & UART_LSR_PE) {
				tty_flag = TTY_PARITY;
			} else if (lsr & UART_LSR_FE) {
				tty_flag = TTY_FRAME;
			}
		}

		if (!tty_buffer_request_room(ttyp, 1))
			tty_flip_buffer_push(ttyp);
		dev_dbg(dev, "%s(): char %c tty_flag 0x%02X\n", __func__, ch, tty_flag);
		ret = tty_insert_flip_char(ttyp, ch, tty_flag);
		if (unlikely(!ret)) {
			up->port.icount.buf_overrun++;
			dev_warn(dev, "%s(): couldn't insert char\n", __func__);
		}

ignore_char:
		lsr = serial_inp(up, UART_LSR);

	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (cnt-- > 0));

	tty_flip_buffer_push(ttyp);

	*status = lsr;
	return;
}

static void transmit_chars(struct uart_8250_port *up)
{
	struct xtm_serial_port *xsp = container_of(up, struct xtm_serial_port, up);
	struct tty_port *ttyp = &xsp->tty_port;
	struct device *dev = ttyp->tty->dev;

	dev_info(dev, "%s():\n", __func__);
}


static void xtm_irq_port(struct xtm_serial_device *xtd, int idx)
{
	unsigned int lsr;
	struct uart_8250_port *up = &xtd->xtm_ports[idx].up;

	lsr = serial_inp(up, UART_LSR);
	pr_debug("%s(): port %d LSR 0x%02X\n", __func__, idx, lsr);

	if (lsr & (UART_LSR_DR | UART_LSR_BI))
		receive_chars(up, &lsr);

	check_modem_status(up);

	if (lsr & UART_LSR_THRE)
		transmit_chars(up);

}

static void xtm_irq_worker(struct work_struct *work)
{
	struct xtm_serial_device *xtd = container_of(work, struct xtm_serial_device, irq_work);
	int idx;

	pr_debug("%s(): %d\n", __func__, __LINE__);
	mutex_lock(&xtd->io_mutex);
	for (idx = 0; idx < NUM_SERIAL_PORTS; idx++)
		if (xtd->opened_ports & (1 << idx))
			xtm_irq_port(xtd, idx);
	mutex_unlock(&xtd->io_mutex);

}

static irqreturn_t xtm_serial_irqh(int irq, void *data)
{
	struct xtm_serial_device *xtd = data;

	pr_debug("%s(): irq %d\n", __func__, irq);

	xtd->cur_irq = irq;
	queue_work(xtd->irq_wq, &xtd->irq_work);
	return IRQ_HANDLED;
}

static int __init xtm_serial_init(void)
{
	struct tty_driver *driver;
	int ret;
	unsigned idx;
	struct xtm_serial_device *priv;
	struct tty_port *ttyp;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	driver = tty_alloc_driver(NUM_SERIAL_PORTS,
		TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW);
	if (IS_ERR(driver))
		return PTR_ERR(driver);

	mutex_init(&priv->io_mutex);
	priv->irq_wq = create_singlethread_workqueue("xtm-irq-wq");
	if (!priv->irq_wq) {
		pr_err("%s(): can't create workqueue\n", __func__);
		return -EBUSY;
	};
	INIT_WORK(&priv->irq_work, xtm_irq_worker);

	priv->irq0 = gpio_to_irq(BANK_0_IRQ_GPIO);
	priv->irq1 = gpio_to_irq(BANK_1_IRQ_GPIO);

	driver->driver_name = "xtm_serial";
	driver->name = "serial";
	driver->type = TTY_DRIVER_TYPE_SERIAL;
	driver->subtype = SYSTEM_TYPE_TTY;
	driver->major = 0;
	driver->init_termios = tty_std_termios;
	driver->init_termios.c_oflag = OPOST | OCRNL | ONOCR | ONLRET;
	driver->driver_state = priv;

	tty_set_operations(driver, &xtm_serial_ops);

	for (idx = 0; idx < NUM_SERIAL_PORTS; idx++) {
		ttyp = &priv->xtm_ports[idx].tty_port;
		tty_port_init(ttyp);
		ttyp->ops = &xtm_serial_port_ops;
		tty_port_link_device(ttyp, driver, idx);
	};

	ret = request_irq(priv->irq0, xtm_serial_irqh, IRQF_TRIGGER_RISING,  "xtm_serial_0", priv);
	if (ret) {
		pr_err("%s(): can't get bank 0 IRQ: %d\n", __func__, ret);
		put_tty_driver(driver);
		goto dreg_ports;
	}

	ret = request_irq(priv->irq1, xtm_serial_irqh, IRQF_TRIGGER_RISING,  "xtm_serial_1", priv);
	if (ret) {
		pr_err("%s(): can't get bank 1 IRQ: %d\n", __func__, ret);
		put_tty_driver(driver);
		goto dreg_ports;
	}

	ret = tty_register_driver(driver);
	if (ret < 0) {
dreg_ports:
		put_tty_driver(driver);
		for (idx = 0; idx < NUM_SERIAL_PORTS; idx++)
			tty_port_destroy(&priv->xtm_ports[idx].tty_port);
		goto out_irqs;
	}

	xtm_serial_driver = driver;
	pr_info("E3Plus Serial Ports Driver registered\n");

	return 0;

out_irqs:
	free_irq(priv->irq1, priv);
	free_irq(priv->irq0, priv);
	kfree(priv);
	return ret;

}

static void __exit xtm_serial_exit(void)
{
	int idx;
	struct xtm_serial_device *priv = xtm_serial_driver->driver_state;
	destroy_workqueue(priv->irq_wq);
	put_tty_driver(xtm_serial_driver);
	for (idx = 0; idx < NUM_SERIAL_PORTS; idx++)
		tty_port_destroy(&priv->xtm_ports[idx].tty_port);
	free_irq(priv->irq0, priv);
	free_irq(priv->irq1, priv);
	tty_unregister_driver(xtm_serial_driver);
	kfree(priv);
}

module_init(xtm_serial_init);
module_exit(xtm_serial_exit);

MODULE_AUTHOR("Dynamic Ratings, INC.");
MODULE_LICENSE("GPL v2");
