/*
 * multi-card driver module for CU devices
 * David Keeffe
 * Systemsolve P/L
 *
 * History:
 *	$Log: not supported by cvs2svn $
 *	Revision 1.4  2006-02-23 20:32:19  cvs
 *	Added comments (isabase, iobase, mapbase and membase..)
 *	
 *	Revision 1.3  2006/02/21 23:08:24  cvs
 *	General changes, added files
 *	
 *	Revision 1.2  2006/02/13 06:50:08  cvs
 *	Changes after actual trial with D030 system.
 *	
 *	Revision 1.1.1.1  2006/02/07 04:43:21  cvs
 *	Initial import from T3 sources
 *	
 *	Revision 1.11  2004/06/06 23:14:56  cvs
 *	fixed a few minor whoopses in LOAD_FPGA ioctl.
 *	Added debugging info.
 *	
 *	Revision 1.10  2004/05/27 23:20:56  cvs
 *	Bug fix not detecting a module at 0x304 when there's nothing at
 *	0x300.
 *	Changed some debug statements.  J.S.
 *	
 *	Revision 1.9  2004/03/14 22:36:43  cvs
 *	Added THIS_MODULE definition for includes which don't
 *	
 *	Revision 1.8  2004/03/14 22:27:43  cvs
 *	Protected event timer from multiple opens.
 *	
 *	Revision 1.7  2004/02/24 01:16:21  cvs
 *	Added spin checks for SPI reg in analog output.
 *	
 *	Revision 1.6  2004/02/23 22:16:38  cvs
 *	Added mask to digital output
 *	
 *	Revision 1.5  2004/02/23 01:39:48  cvs
 *	Analog support
 *	Module reporting ioctl
 *	
 *	Revision 1.4  2004/02/19 20:28:27  cvs
 *	Added readback check in cuio_probe and make sure cutype[i] is zero. J.S.
 *	
 *	Revision 1.3  2004/01/05 01:01:15  cvs
 *	changed io instructions to macros....
 *	
 *	Revision 1.2  2004/01/05 00:46:58  cvs
 *	Chenges after testing. More to come, though.
 *	
 *	Revision 1.1  2003/12/09 00:07:31  cvs
 *	Changed all references from 'UIM' to 'CU' to avoid future confusion.
 *	The controller hardware is known as a Control Unit (or CU),
 *	and is made up from typically several Control Modules (CMs).
 *	The CU handles the direct connection of physical signals to be connected
 *	from the field (digital and analog I/O).
 *	
 *	The UIM (or IU) is actually a different piece of hardware.
 *	
 *	Revision 1.3  2003/07/22 21:10:59  cvs
 *	Added analog out support.
 *	Added analog in support, tested with prototype board.
 *	
 *	Revision 1.2  2003/02/07 00:11:08  cvs
 *	Put in CVS/RCS info, headers, etc.
 *	
 *
 *	Revision 1.1.1.1  2003/02/07 00:00:04  cvs
 *	initial
 *	
 */

static const char rcsid[] __attribute__ ((unused)) = "$Id$";

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/version.h>
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,31)
#include <linux/config.h>
#endif
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

#include "drmcc.h"
#include "ftdi_isa.h"

#define MAXCU 16
#define MAXCUTYPE 3
#define EVMAX 100
#define MAX_AO_CHAN 3

static DEFINE_MUTEX(cuio_state_lock);
static int cuio_open_cnt;	/* #times opened */
static int cuio_open_mode;	/* special open modes */

static unsigned init_ioaddr;
static u8  __iomem *ioaddr[MAXCU] = {(u8 *)0x300, };
static u8  __iomem *ioaddr_map[MAXCU];
static int evmax = EVMAX;
static int cutype[MAXCU];
static int cuboards[MAXCUTYPE+1];
static int cuboardmap[MAXCUTYPE+1][MAXCU];
static int cucount;
static int cuwait;
#ifdef DEBUG
static int cudebug = 1;
#else
static int cudebug;
#endif

static struct cu_eventlist cu_eventlist;
static struct cu_event *cuevents;
struct timer_list cu_timer;
static char cuversion[] = "cu CM io " CU_BUILD_ID;
static char cubuild[] = "BUILD";

struct clientdata {
	int done;
    int len;
    char *buf;
    unsigned long jiffies;
} cu_data;

#ifndef CONFIG_DR_E3PLUS
static unsigned long READB(unsigned *addr) {printk(KERN_INFO "%s(): %d %px\n", __func__, __LINE__, addr); return 0L;};
static unsigned long INB(unsigned *addr) {printk(KERN_INFO "%s(): %d %px\n", __func__, __LINE__, addr); return 0L;};
static void WRITEB(unsigned val, unsigned *addr) {printk(KERN_INFO "%s(): %d 0x%X -> %px\n", __func__, __LINE__, val, addr);};
static void OUTB(unsigned val, unsigned *addr) {printk(KERN_INFO "%s(): %d 0x%X -> %px\n", __func__, __LINE__, val, addr);};
#else
#define READB(addr)			ftdi_isa_read(addr, 0)
#define INB(addr)			ftdi_isa_read(addr, 1)
#define WRITEB(val, addr)		ftdi_isa_write(val, addr, 0)
#define OUTB(val, addr)			ftdi_isa_write(val, addr, 1)
#endif

void
cuio_record_data(void)
{
	int k, j, i, v;
	unsigned long long val;

	val = 0;
	for (j = 0, k = 0; j < MAXCU; j++) {
		if (ioaddr[j] == 0 || cutype[j] != 1)
			continue;
		if (cuwait) 
			udelay(cuwait);
		v = READB(ioaddr_map[j]+1);
		v <<= 8;
		v |= READB(ioaddr_map[j]);
		if (cudebug > 2) {
			printk(KERN_INFO "CU check input %d at 0x%px -> %02x\n", k, ioaddr[j], v);
		}
		val |= ((unsigned long long)v) << (k*12);
		k++;
	}
	if (val == cuevents[cu_eventlist.last].val) {
		return;
	}
	if(cu_eventlist.nevents < cu_eventlist.maxevents) {
		/*
		 * list is not full
		 * just append
		 */
		i = (cu_eventlist.nevents + cu_eventlist.firstevent)%cu_eventlist.maxevents;
		cu_eventlist.nevents++;
	} else {
		/*
		 * list is full
		 * overwrite first event, move pointer
		 */
		i = cu_eventlist.firstevent;
		cu_eventlist.firstevent = (1+cu_eventlist.firstevent)%cu_eventlist.maxevents;
	}
	cuevents[i].val = val;
	cuevents[i].time = jiffies - cu_eventlist.basetime;
	cu_eventlist.last = i;
	if (cudebug > 1) {
		printk(KERN_INFO "CU record event[%d]: T%ld -> %x,%x\n", i, cuevents[i].time, (int)((val>>32)&0xffffffff), (int)(val&0xffffffff));
	}
}

void
cuio_timeout(struct timer_list *arg)
{
	struct clientdata *cp = (struct clientdata *)arg;
	cuio_record_data();
	if (!cp->done) {
		mod_timer(&cu_timer, round_jiffies(jiffies + (HZ/10)));
	}
}

long
cuio_getadc(void __iomem *port)
{
	int j, c1, c2;
	long v = 0;
	unsigned char cv, sv;
	if (cudebug > 2) {
		printk(KERN_INFO "CU getADC: %px\n", port);
	}
	WRITEB(0x20, port+2);
	for (j = 0; j < 4; j++) {
		c1 = 0; c2 = 0;
		while((sv = READB(port+3)) == 0)
			c1++;
		WRITEB(0xaa, port); udelay(50);
		while((sv = READB(port+3)) == 0)
			c2++;
		cv = READB(port);
		if (cudebug > 3) {
			printk(KERN_INFO "CU getADC: byte %d,%d -> %x -> %lx\n", c1, c2, cv, v);
		}
		v = (v<<8) | cv;
	}
	if (cudebug > 2) {
		printk(KERN_INFO "CU getADC: final %lx\n", v);
	}
	return v;
}

static const char *cutypes[]={"Unknown", "DigitalIn", "DigitalOut", "AnalogIO", NULL};

static long
cuio_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
	int v, i, k, l, l_actual;
	int c1, sv;
	unsigned long long val_i;
	unsigned long val_o;
	long cval[6][2];
	unsigned int cchan;
	struct cu_eventlist *up;
	struct cu_analogin aibuf;
	struct cu_analogout aobuf;
	struct cu_digitalout dobuf;
	struct cu_fpgadata fpgabuf;
	struct cu_modules mod_data;
	unsigned char *fpgadata;
	unsigned char v0, v1;
	int ret = 0;

	if (cudebug) printk(KERN_INFO "CU device ioctl: %x -> %lx\n", cmd, arg);

	ret = mutex_lock_interruptible(&cuio_state_lock);
	if (ret)
		return ret;

	switch(cmd) {
		case DRMCC_GET_MODULES:
			if (cudebug) printk(KERN_INFO "CU GET MODULES\n");
			mod_data.modinfo[0] = MAXCUTYPE;
			for (i = 1; i <= MAXCUTYPE; i++) {
				mod_data.modinfo[i] = 0;
			}
			for (i = 0; i < MAXCU; i++) {
				if (ioaddr[i] == 0) continue;
				if (cudebug) printk(KERN_INFO "CU device report %d B: %d -> %d\n", i, cutype[i], mod_data.modinfo[cutype[i]]);
				(mod_data.modinfo[cutype[i]])++;
				if (cudebug) printk(KERN_INFO "CU device report %d A: %d -> %d\n", i, cutype[i], mod_data.modinfo[cutype[i]]);
			}
			if (copy_to_user((void *)arg, &mod_data, sizeof(struct cu_modules))) {
				ret = -EFAULT;
				goto out;
			}
			ret = 0;
			break;

		case DRMCC_GET_DOBITS:
			val_o = 0;
			for (i = 0, k = 0; i < MAXCU; i++) {
				if (ioaddr[i] == 0 || cutype[i] != 2) continue;
				v = READB(ioaddr_map[i]);
				val_o |= ((unsigned long)v) << (k*8);
				k++;
			}
			if (k == 0) {
				ret = -EINVAL;
				goto out;
			}
			ret = put_user(val_o, (unsigned long *)arg);
			break;

		case DRMCC_PUT_DOBITS:
			/*
			 * params are a single input structure
			 */
			if (copy_from_user(&dobuf, (void *)arg, sizeof(struct cu_digitalout))) {
				ret = -EFAULT;
				goto out;
			}
			val_o = 0;
			for (i = 0, k = 0; i < MAXCU; i++) {
				if (ioaddr[i] == 0 || cutype[i] != 2) continue;
				v = READB(ioaddr_map[i]);
				val_o |= ((unsigned long)v) << (k*8);
				k++;
			}
			val_o = (val_o & (dobuf.value|(~dobuf.mask)))|(dobuf.value&dobuf.mask);
			for (i = 0, k = 0; i < MAXCU; i++) {
				if (ioaddr[i] == 0 || cutype[i] != 2) continue;
				WRITEB((val_o>>(k*8))&0xff, ioaddr_map[i]);
				k++;
			}
			if (k == 0) {
				ret = -EINVAL;
				goto out;
			}
			ret = 0;
			break;

		case DRMCC_CLR_EVDATA:
			cu_eventlist.nevents = 0;
			cu_eventlist.firstevent = 0;
			ret = 0;
			break;

		case DRMCC_GET_EVDATA:
			if (copy_to_user((void *)arg, &cu_eventlist, sizeof(struct cu_eventlist))) {
				return -EFAULT;
			}
			up = (struct cu_eventlist *)arg;
			if (cudebug > 1) printk(KERN_INFO "CU GET EVDATA: %px -> %px\n", up, &(up->data));
			if (copy_to_user((void *)(&(up->data)), cuevents, evmax*sizeof(struct cu_event))) {
				return -EFAULT;
			}
			ret = 0;
			break;

		case DRMCC_GET_EVSIZE:
			if (cuevents == NULL) return -EINVAL;
			ret = put_user(cu_eventlist.nevents, (int *)arg);
			break;

		case DRMCC_GET_EVMAX:
			if (cuevents == NULL) return -EINVAL;
			ret = put_user(cu_eventlist.maxevents, (int *)arg);
			break;

		case DRMCC_GET_DIBITS:
			val_i = 0;
			for (i = 0, k = 0; i < MAXCU; i++) {
				if (ioaddr[i] == 0 || cutype[i] != 1) continue;
				//v = inw(ioaddr_map[i]);
				v = READB(ioaddr_map[i]+1);
				v <<= 8;
				v |= READB(ioaddr_map[i]);
				val_i |= ((unsigned long long)v) << (k*12);
				k++;
			}
			if (k == 0) {
				ret = -EINVAL;
				goto out;
			}
			if (cudebug > 1) printk(KERN_INFO "CU GET DIBITS: %Lx\n", val_i);
			ret = copy_to_user((unsigned long long *)arg, &val_i, sizeof(unsigned long long));
			break;

		case DRMCC_PUT_ANALOG:
			/*
			 * params are a single input structure
			 */
			if (copy_from_user(&aobuf, (void *)arg, sizeof(struct cu_analogout))) {
				ret = -EFAULT;
				goto out;
			}
			if (cudebug) {
				printk(KERN_INFO "CU PUT ANALOG: unit %d channel %d value %ld (0x%lx)\n", aobuf.unit, aobuf.channel, aobuf.value, aobuf.value);
			}
			if (aobuf.unit > cuboards[3] || aobuf.unit > MAXCU) {
				ret = -EINVAL;
				goto out;
			}
			if (aobuf.channel > MAX_AO_CHAN) {
				ret = -EINVAL;
				goto out;
			}
			i = cuboardmap[3][aobuf.unit];
			if (i < 0 || i >= MAXCU || ioaddr[i] == 0 || cutype[i] != 3) {
					ret = -EINVAL;
					goto out;
			}
			k = aobuf.channel;
			/*
			 * i is the hardware info index
			 * each DAC has 2 channels;
			 */
			WRITEB(0x20, ioaddr_map[i]+2);
			WRITEB(k+7, ioaddr_map[i]+1);
			v0 = ((aobuf.value>>6)&0x3f)|0x40;
			v1 = ((aobuf.value<<2)&0xfc);
			if (cudebug > 1) {
				printk(KERN_INFO "CU PUT ANALOG DEV@%px: %02x %02x\n", ioaddr[i], v0, v1);
			}
			c1 = 0;
			WRITEB(v0, ioaddr_map[i]);
			c1 = 0;
			while((sv = READB(ioaddr_map[i]+3)) == 0)
				c1++;
			if (cudebug > 2) {
				printk(KERN_INFO "CU PUT ANALOG DEV@%px: SPI ready after %d\n", ioaddr[i], c1);
			}
			WRITEB(v1, ioaddr_map[i]);
			c1 = 0;
			while((sv = READB(ioaddr_map[i]+3)) == 0)
				c1++;
			if (cudebug > 2) {
				printk(KERN_INFO "CU PUT ANALOG DEV@%px: SPI ready after %d\n", ioaddr[i], c1);
			}
			WRITEB(0xff, ioaddr_map[i]+1);
			ret =  0;
			break;

		case DRMCC_GET_ANALOG:
			/*
			 * params are a single input structure
			 */
			if (copy_from_user(&aibuf, (void *)arg, sizeof(struct cu_analogin))) {
				ret = -EFAULT;
				goto out;
			}
			if (cudebug) {
				printk(KERN_INFO "CU GET ANALOG: unit %d\n", aibuf.unit);
			}
			if (aibuf.unit > cuboards[3] || aibuf.unit > MAXCU) {
				ret = -EINVAL;
				goto out;
			}
			i = cuboardmap[3][aibuf.unit];
			if (i < 0 || i >= MAXCU || ioaddr[i] == 0 || cutype[i] != 3) {
					ret = -EINVAL;
					goto out;
			}
			/*
			 * i is the hardware info index
			 * each ADC has 2 channels;
			 * for the RTD we add the values
			 * for the CT we do c1-c0
			 * for DC ch0 is current (I)
			 * for DC ch1 is voltage (E)
			 */
			/*
			 * TODO: return k to 1
			 */
			for (l = 0; l < 2; l++) {
				long ctemp;
				for (k = 1; k <= 6; k++) {
					WRITEB((char)k, ioaddr_map[i]+1); /* select device */
					WRITEB(0xff, ioaddr_map[i]+1);	/* deselect to start adc */
				}
				/* delay min 160 MS */
				current->state = TASK_INTERRUPTIBLE;
				schedule_timeout(HZ/5);
				/*
				 * CT
				 */
				for (k = 0; k < 2; k++) {
					WRITEB(k+1, ioaddr_map[i]+1);
					ctemp = cchan = cuio_getadc(ioaddr_map[i]);
					cchan >>= 28;
					l_actual = (cchan>>2)&1;
					cval[(k+1)-1][l_actual] = (ctemp&0x1fffffff); /* 29 bits */
					cval[(k+1)-1][l_actual] >>= 4; /* 25 bits (24 bits + 'extended range') */
					if ((cchan & 2) == 0) {
						/* apply sign bit by simulating sign-extension */
						cval[(k+1)-1][l_actual] |= 0xff000000;
					}
					WRITEB(0xff, ioaddr_map[i]+1);
					if (cudebug > 1) {
						printk(KERN_INFO "CU GET ANALOG: raw CT[%d] %lx -> (%d) %d %lx %ld\n",
							k, ctemp, cchan, l_actual, cval[(k+1)-1][l_actual], cval[(k+1)-1][l_actual] );
					}
				}
				for (k = 0; k < 2; k++) {
					WRITEB(k+3, ioaddr_map[i]+1);
					ctemp = cchan = cuio_getadc(ioaddr_map[i]);
					cchan >>= 28;
					l_actual = (cchan>>2)&1;
					cval[(k+3)-1][l_actual] = (ctemp&0x1fffffff); /* 29 bits */
					cval[(k+3)-1][l_actual] >>= 4; /* 25 bits */
					if ((cchan & 2) == 0) {
						/* apply sign bit by simulating sign-extension */
						cval[(k+3)-1][l_actual] |= 0xff000000;
					}
					WRITEB(0xff, ioaddr_map[i]+1);
					if (cudebug > 1) {
						printk(KERN_INFO "CU GET ANALOG: raw RTD[%d] %lx -> (%d) %d %lx %ld\n",
							k, ctemp, cchan, l_actual, cval[(k+3)-1][l_actual], cval[(k+3)-1][l_actual] );
					}
				}
				/*
				 * make k 0 when all ports working
				 */
				for (k = 0; k < 2; k++) {
					WRITEB(k+5, ioaddr_map[i]+1);
					ctemp = cchan = cuio_getadc(ioaddr_map[i]);
					cchan >>= 28;
					l_actual = (cchan>>2)&1;
					cval[(k+5)-1][l_actual] = (ctemp&0x1fffffff); /* 29 bits */
					cval[(k+5)-1][l_actual] >>= 4; /* 25 bits */
					if ((cchan & 2) == 0) {
						/* apply sign bit by simulating sign-extension */
						cval[(k+5)-1][l_actual] |= 0xff000000;
					}
					WRITEB(0xff, ioaddr_map[i]+1);
					if (cudebug > 1) {
						printk(KERN_INFO "CU GET ANALOG: raw DC[%d] %lx -> (%d) %d %lx %ld\n",
							k, ctemp, cchan, l_actual, cval[(k+5)-1][l_actual], cval[(k+5)-1][l_actual] );
					}
				}
			}
			aibuf.dcvalueI[0] = cval[4][0];
			aibuf.dcvalueI[1] = cval[5][0];
			aibuf.dcvalueE[0] = cval[4][1];
			aibuf.dcvalueE[1] = cval[5][1];
			if (cudebug) {
				printk(KERN_INFO "CU GET ANALOG: DC[0] %ldA %ldV\n", aibuf.dcvalueI[0], aibuf.dcvalueE[0]);
				printk(KERN_DEBUG "CU GET ANALOG: DC[1] %ldA %ldV\n", aibuf.dcvalueI[1], aibuf.dcvalueE[1]);
			}
			aibuf.ctvalue[0] = cval[0][1] - cval[0][0];
			aibuf.ctvalue[1] = cval[1][1] - cval[1][0];
			if (cudebug) {
				printk(KERN_INFO "CU GET ANALOG: CT[%d] %ld \n", 0, aibuf.ctvalue[0]);
				printk(KERN_INFO "CU GET ANALOG: CT[%d] %ld \n", 1, aibuf.ctvalue[1]);
			}
			aibuf.rtvalue[0] = cval[2][1] + cval[2][0];
			aibuf.rtvalue[1] = cval[3][1] + cval[3][0];
			if (cudebug) {
				printk(KERN_INFO "CU GET ANALOG: RTD[%d] %ld \n", 0, aibuf.rtvalue[0]);
				printk(KERN_INFO "CU GET ANALOG: RTD[%d] %ld \n", 1, aibuf.rtvalue[1]);
			}
			ret = copy_to_user((struct cu_analogin *)arg, &aibuf, sizeof(struct cu_analogin));
			break;

		case DRMCC_LOAD_FPGA:
			/*
			 * params are a single input structure
			 */
			if (copy_from_user(&fpgabuf, (void *)arg, sizeof(struct cu_fpgadata))) {
				ret = -EFAULT;
				goto out;
			}
			if (cudebug) {
				printk(KERN_INFO "CU LOAD FPGA: type %d unit %d length %ld\n", fpgabuf.type, fpgabuf.unit, fpgabuf.length);
			}
			/* TODO: consider type 1 boards too */
			if ((fpgabuf.type != 1 && fpgabuf.type != 3) || fpgabuf.unit > cuboards[fpgabuf.type] || fpgabuf.unit > MAXCU) {
				ret = -EINVAL;
				goto out;
			}
			i = cuboardmap[fpgabuf.type][fpgabuf.unit];
			if (cudebug) {
				printk(KERN_INFO "CU LOAD FPGA: board map -> %d\n", i);
			}
			if (i < 0 || i >= MAXCU || ioaddr[i] == 0 || cutype[i] != fpgabuf.type) {
					ret = -EINVAL;
					goto out;
			}
			fpgadata = (unsigned char *)kmalloc(fpgabuf.length, GFP_KERNEL);
			if (cudebug > 1) {
				printk(KERN_INFO "CU LOAD FPGA: copying buffer from %px to %px\n", fpgabuf.data, fpgadata);
			}
			if (copy_from_user(fpgadata, (void *)fpgabuf.data, fpgabuf.length)) {
				ret = -EFAULT;
				goto out;
			}
			if (cudebug > 2) {
				printk(KERN_INFO "CU LOAD FPGA: buffer\n");
				if (fpgabuf.length < 20) {
					for (k = 0; k < fpgabuf.length; k++) {
						printk(KERN_DEBUG "%d: %02x\n", k, fpgadata[k]);
					}
				} else {
					for (k = 0; k < 10; k++) {
						printk(KERN_DEBUG "%d: %02x\n", k, fpgadata[k]);
					}
					printk(KERN_DEBUG "....\n");
					for (k = fpgabuf.length - 9; k < fpgabuf.length; k++) {
						printk(KERN_DEBUG "%d: %02x\n", k, fpgadata[k]);
					}
				}
				printk(KERN_DEBUG "\n");
			}
			/* Write to Control Index Register */
			WRITEB(0x0f, ioaddr_map[i]+2);
			WRITEB(0, ioaddr_map[i]+3);
			mdelay(10);
			/* Write bit0 = 1 bit1 = 1 to FPGA programming register*/
			WRITEB(3, ioaddr_map[i]+3);
			mdelay(1);

			ftdi_isa_write_multiple(fpgadata, ioaddr_map[i], fpgabuf.length);

			udelay(5);
			/* disable the programming mode */
			WRITEB(2, ioaddr_map[i]+3);
			udelay(5);
			WRITEB(0x10, ioaddr_map[i]+2);
			k = READB(ioaddr_map[i]+3);
			kfree(fpgadata);
			if (cudebug >= 0) {
				printk(KERN_INFO "CU load FPGA: done. Type %d {%s} @ %px rev %d\n", cutype[i],cutypes[cutype[i]], ioaddr[i], k);
 				printk(KERN_INFO "CU load FPGA: done -> addr %px rev %d\n", ioaddr[i], k);
			}

			ret = 0;
			break;

		default:
			ret = -EINVAL;
			break;
	}

out:
	mutex_unlock(&cuio_state_lock);
	return ret;
}

int
cuio_release(struct inode *ip, struct file *fp)
{

	int ret;

	ret = mutex_lock_interruptible(&cuio_state_lock);
	if (ret)
		return ret;

	cuio_open_cnt--;

	if (cuio_open_cnt == 0) {
		/* if only one instance is open, clear the EXCL bit */
		if (cuio_open_mode & O_EXCL)
			cuio_open_mode &= ~O_EXCL;

		cu_data.done = 1;
		if (cuboards[1] > 0) {
			del_timer_sync(&cu_timer);
		}

		if (cudebug) printk(KERN_INFO "CU device release\n");

	} else {
		if (cudebug) printk(KERN_INFO "CU non-final release\n");
	}

	mutex_unlock(&cuio_state_lock);

	return 0;
}

int
cuio_open(struct inode *ip, struct file *fp)
{
	int ret = 0;

	ret = mutex_lock_interruptible(&cuio_state_lock);
	if (ret)
		return ret;

	if ((cuio_open_cnt && (fp->f_flags & O_EXCL)) ||
	    (cuio_open_mode & O_EXCL)) {
		ret = -EBUSY;
		goto out;
	}

	if (cudebug) printk(KERN_INFO "CU device open\n");

	if ((cuboards[1] > 0) || (cuboards[2] > 0) || (cuboards[3] > 0))  {
		if (cuio_open_cnt == 0) {

			cu_timer.expires = round_jiffies(jiffies + (HZ/10));
			timer_setup(&cu_timer, cuio_timeout, (unsigned long) &cu_data);

			cu_eventlist.basetime = jiffies;
			cu_eventlist.nevents = 0;
			cu_eventlist.firstevent = 0;
			cuio_record_data();
		} else {
			if (cudebug) printk(KERN_INFO "CU non-initial open\n");
		}

		if (fp->f_flags & O_EXCL)
			cuio_open_mode |= O_EXCL;
		cuio_open_cnt++;

	} else {
		if (cudebug) printk(KERN_INFO "CU No type 1 boards to open\n");
		ret = -ENODEV;
	}

out:
	mutex_unlock(&cuio_state_lock);
	return ret;
}

int
cuio_probe(void __iomem *port_raw, int range, int type)
{
	int tt;
	u8 *port;

	port = (u8 *)port_raw;
	if (cudebug) printk(KERN_INFO "CUIO Probe: type %d raw %px -> %px\n", type, port_raw, port);

	// WRITEB(0, port+2);
	OUTB(0, port+2);

	/* test to see that we wrote to a register at least */
	/* and that we can read back what was just written */
	// tt = READB(port+2);
	tt = INB(port+2);
	if (cudebug) printk(KERN_INFO "CUIO Probe: read 1 %px -> %d\n", port+2, tt);
	if (tt != 0x0) {
		iounmap(port);
		return -1;
	}
 
	// tt = READB(port+3);
	tt = INB(port+3);
	if (cudebug) printk(KERN_INFO "CUIO Probe: read 2 %px -> %d\n", port+3, tt);
	if (tt == 0xff) {
		iounmap(port);
		return -1;
	} else if ((type && (tt != type)) || (tt > MAXCUTYPE)) {
		/* not the right ID */
		/* or ID out of range */
		if (cudebug) printk(KERN_INFO "CUIO Probe: wrong ID\n");
		iounmap(port);
		return -2;
	} else {
		if (tt == 1 || tt == 3) {
			/* select the FPGA as it doesn't seem to go otherwise */
			/* This should be fixed now - but it won't hurt to */
			/* leave this in. */
			WRITEB(0x10, port+2);
		}
		if (cudebug) printk(KERN_INFO "CUIO Probe: return %d\n", tt);
		return tt;
	}
}

struct file_operations cuio_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cuio_ioctl,
	.open = cuio_open,
	.release = cuio_release
};

#ifdef DRMCC_CU_IN
#define CUIO_MINOR_BASE		(DRMCC_CU_IN)
#else
#define CUIO_MINOR_BASE		(MISC_DYNAMIC_MINOR)
#endif

static struct miscdevice cuio = {
	.minor = CUIO_MINOR_BASE,
	.name = "cuio",
	.fops = &cuio_fops
};

static int cuio_init(struct platform_device *pdev)
{
	int i = 10;
	int ret = 0;
	u8 *boardp = ioaddr[0];
	struct gpio_desc *isa_reset_gpio;
	unsigned isa_sleep_time = 100; /* msec */

	/* Wait for the USB-ISA bridge to come online */
	/* Still a hack, have a better way to do this upcoming */
	if (READB((unsigned *) 0x300) == READ_NO_DEVICE)
			return -EPROBE_DEFER;

	printk(KERN_INFO "DRMCC CU driver %s on %s\n", cuversion, cubuild);

	isa_reset_gpio = devm_gpiod_get(&pdev->dev, "isa-reset", GPIOD_OUT_HIGH);
	if (IS_ERR(isa_reset_gpio))
		return PTR_ERR(isa_reset_gpio);

	if (cudebug)
		dev_info(&pdev->dev, "%s(): resetting ISA Bus\n", __func__);

	msleep(isa_sleep_time);
	gpiod_set_value_cansleep(isa_reset_gpio, 0);

	if (init_ioaddr)
		boardp = (u8 *) init_ioaddr;

	for (i = 0; (i < MAXCU); i++, boardp += 4) {
		ioaddr[i] = boardp;
		if ((cutype[i] = cuio_probe(ioaddr[i], 4, cutype[i])) <= 0) {
			if (cudebug)
				printk(KERN_DEBUG "CU module failed (%d) with bad hardware at 0x%px\n", cutype[i], ioaddr[i]);
			continue;
		}

		cucount++;
		printk(KERN_INFO "DRMCC CU card type %d {%s} found at 0x%px\n", cutype[i],cutypes[cutype[i]], ioaddr[i]);

		if (cutype[i] <= MAXCUTYPE){
			int bi = cuboards[cutype[i]]++;

			if (cudebug) {
				printk(KERN_INFO "CU type %d at %d\n", cutype[i], bi);
			}
			cuboardmap[cutype[i]][bi] = i;
		}
		ioaddr_map[i] = ioaddr[i];
	}

	if (cucount > 0) {
		ret = misc_register(&cuio);
	} else {
		return -ENODEV;
	}

	if (!ret && !cuevents) {
		cu_eventlist.nevents = 0;
		cu_eventlist.maxevents = evmax;
		cu_eventlist.firstevent = 0;
		cuevents = (struct cu_event *)kmalloc(evmax * sizeof(struct cu_event), GFP_KERNEL);
		if (cuevents) {
			printk(KERN_DEBUG "DRMCC CU pre-allocated %d event spaces\n", evmax);
		} else {
			return -ENOMEM;
		}
	}
	
	return ret;
}

static int cuio_exit(struct platform_device *pdev)
{
	del_timer_sync(&cu_timer);
	if (cuevents)
		kfree(cuevents);
	misc_deregister(&cuio);
	printk(KERN_INFO "DRMCC CU module unloaded\n");

	return 0;
}

static struct of_device_id cuio_match[] = {
	{ .compatible = "dynamicratings,cuio", },
	{},
};

static struct platform_driver cuio_driver = {
	.driver = {
		.name = "cuio",
		.of_match_table = cuio_match,
	},
	.probe = cuio_init,
	.remove = cuio_exit,
};

static int __init cuio_driver_init(void)
{
	return platform_driver_register(&cuio_driver);
}

static void __exit cuio_driver_exit(void)
{
	return platform_driver_unregister(&cuio_driver);
}

module_init(cuio_driver_init);
module_exit(cuio_driver_exit);

MODULE_PARM_DESC(cudebug, "the debug level (default 0)");
module_param_named(cudebug, cudebug, uint, 0644);
MODULE_PARM_DESC(ioaddr, "the I/O port for the device (default 0x300)");
module_param_named(ioaddr, init_ioaddr, uint, 0644);
MODULE_PARM_DESC(evmax, "the maximum number of events to save (def 100, max 255)");
module_param_named(evmax, evmax, uint, 0644);
MODULE_PARM_DESC(cuwait, "delay in usec, before reading data (def 0)");
module_param_named(cuwait, cuwait, uint, 0644);
MODULE_AUTHOR("David Keeffe");
MODULE_DESCRIPTION("CUIO Driver");
MODULE_LICENSE("GPL");
