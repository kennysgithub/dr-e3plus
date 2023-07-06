#ifndef __DRMCC_H__
#define __DRMCC_H__

#define DRMCC_MAXMOD 3
#define DRMCC_MODSPACE 6

struct cu_event {
		time_t sec;
		unsigned long	time;
		unsigned long long val;
};

struct cu_eventlist {
		unsigned int nevents;
		unsigned int maxevents;
		unsigned int firstevent;
		unsigned long basetime;
		int last;
		unsigned int data; /* must be last in struct */
};

struct cu_analogin {
		unsigned char unit;	/* card index */
		long ctvalue[2];
		long dcvalueE[2];
		long dcvalueI[2];
		long rtvalue[2];
};

struct cu_analogout {
		unsigned char unit, channel;
		long value;
};

struct cu_digitalout {
		unsigned long mask;
		unsigned long value;
};

struct cu_fpgadata {
		unsigned char type;
		unsigned char unit;
		long length;
		unsigned char *data;
};

struct cu_modules {
		char modinfo[DRMCC_MODSPACE];
};

#define DRMCC_IOCTL_BASE 'U'
#define DRMCC_GET_DIBITS _IOR(DRMCC_IOCTL_BASE, 0, unsigned long long)
#define DRMCC_PUT_DOBITS _IOW(DRMCC_IOCTL_BASE, 1, struct cu_digitalout)
#define DRMCC_GET_DOBITS _IOW(DRMCC_IOCTL_BASE, 2, unsigned long)
#define DRMCC_GET_EVSIZE _IOR(DRMCC_IOCTL_BASE, 3, int)
#define DRMCC_GET_EVMAX _IOR(DRMCC_IOCTL_BASE, 4, int)
#define DRMCC_GET_EVDATA _IOWR(DRMCC_IOCTL_BASE, 5, struct cu_eventlist)
#define DRMCC_CLR_EVDATA _IO(DRMCC_IOCTL_BASE, 6)
#define DRMCC_GET_ANALOG _IOR(DRMCC_IOCTL_BASE, 7, struct cu_analogin)
#define DRMCC_PUT_ANALOG _IOW(DRMCC_IOCTL_BASE, 12, struct cu_analogout)
#define DRMCC_LOAD_FPGA _IOW(DRMCC_IOCTL_BASE, 13, struct cu_fpgadata)
#define DRMCC_GET_MODULES _IOW(DRMCC_IOCTL_BASE, 14, struct cu_modules)

#endif /* __DRMCC_H__ */
