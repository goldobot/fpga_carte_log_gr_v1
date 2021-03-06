
/* control structure */

#ifndef __ASSEMBLER__

struct lregs {
  /* 0x00 */
  volatile unsigned int memcfg1;		
  volatile unsigned int memcfg2;
  volatile unsigned int ectrl;
  volatile unsigned int failaddr;
  /* 0x10 */
  volatile unsigned int memstatus;
  volatile unsigned int cachectrl;
  volatile unsigned int powerdown;
  volatile unsigned int writeprot1;
  /* 0x20 */
  volatile unsigned int writeprot2;	
  volatile unsigned int leonconf;
  volatile unsigned int dummy2;
  volatile unsigned int dummy3;
  /* 0x30 */
  volatile unsigned int dummy4;	
  volatile unsigned int dummy5;
  volatile unsigned int dummy6;
  volatile unsigned int dummy7;
  /* 0x40 */
  volatile unsigned int timercnt1;
  volatile unsigned int timerload1;
  volatile unsigned int timerctrl1;
  volatile unsigned int wdog;
  /* 0x50 */
  volatile unsigned int timercnt2;
  volatile unsigned int timerload2;
  volatile unsigned int timerctrl2;
  volatile unsigned int dummy8;
  /* 0x60 */
  volatile unsigned int scalercnt;
  volatile unsigned int scalerload;
  volatile unsigned int dummy9;
  volatile unsigned int dummy10;
  /* 0x70 */
  volatile unsigned int uartdata1;
  volatile unsigned int uartstatus1;
  volatile unsigned int uartctrl1;
  volatile unsigned int uartscaler1;
  /* 0x80 */
  volatile unsigned int uartdata2;
  volatile unsigned int uartstatus2;
  volatile unsigned int uartctrl2;
  volatile unsigned int uartscaler2;
  /* 0x90 */
  volatile unsigned int irqmask;
  volatile unsigned int irqpend;
  volatile unsigned int irqforce;
  volatile unsigned int irqclear;
  /* 0xa0 */
  volatile unsigned int piodata;
  volatile unsigned int piodir;
  volatile unsigned int pioirq;
  volatile unsigned int dummy11;
  /* 0xb0 */
  volatile unsigned int imask2;
  volatile unsigned int ipend2;
  volatile unsigned int istat2;
  volatile unsigned int iclear2;
  /* 0xc0 */
  volatile unsigned int dcomdata;
  volatile unsigned int dcomstatus;
  volatile unsigned int dcomctrl;
  volatile unsigned int dcomscaler;
};

#endif /* __ASSEMBLER__ */

/* control registers */

#define	PREGS	0x80000000
#define	MCFG1	0x00
#define	MCFG2	0x04
#define	MCFG3	0x08
#define	ECTRL	0x08
#define	FADDR	0x0c
#define	MSTAT	0x10
#define CCTRL	0x14
#define PWDOWN	0x18
#define WPROT1	0x1C
#define WPROT2	0x20
#define LCONF 	0x24
#define	TCNT0	0x40
#define	TRLD0	0x44
#define	TCTRL0	0x48
#define	TCNT1	0x50
#define	TRLD1	0x54
#define	TCTRL1	0x58
#define	SCNT  	0x60
#define	SRLD  	0x64
#define	UDATA0 	0x70
#define	USTAT0 	0x74
#define	UCTRL0 	0x78
#define	USCAL0 	0x7c
#define	UDATA1 	0x80
#define	USTAT1 	0x84
#define	UCTRL1 	0x88
#define	USCAL1 	0x8c
#define	IMASK	0x90
#define	IPEND	0x94
#define	IFORCE	0x98
#define	ICLEAR	0x9c
#define	IOREG	0xA0
#define	IODIR	0xA4
#define	IOICONF	0xA8
#define	IMASK2	0xB0
#define	IPEND2	0xB4
#define	ISTAT2  0xB8
#define	ICLEAR2	0xBC

/* ASI codes */

#define ASI_PCI 	0x4
#define ASI_ITAG	0xC
#define ASI_IDATA	0xD
#define ASI_DTAG	0xE
#define ASI_DDATA	0xF

/* memory areas */

#define CRAM	0x40000000
#define SDRAM	0x60000000
#define IOAREA	0x20000000

/* Some bit field masks */

#define CCTRL_FLUSHING_MASK 0x0c000

#define RFE_CONF_BIT	30
#define RFE_CONF_MASK	3
#define CPP_CONF_BIT	19
#define CPP_CONF_MASK	3
#define FPU_CONF_BIT	4
#define FPU_CONF_MASK	3
#define CPTE_MASK	(3 << 17)
#define MUL_CONF_BIT	8	
#define MAC_CONF_BIT	25	
#define DIV_CONF_BIT	9	
#define REDAC_CONF_BIT	9	
#define PEDAC_CONF_BIT	8	
#define MEDAC_CONF_BIT	27	
#define MMU_CONF_BIT	31
#define ITE_BIT		12
#define IDE_BIT		10
#define DTE_BIT		8
#define DDE_BIT		6
#define CE_CLEAR	0x3fc0;

/* Processor State Register PSR */
#define PSR_ICC_IMPL	0xF0000000
#define PSR_ICC_VER	0x0F000000
#define PSR_ICC_MASK	0x00F00000
#define PSR_ICC_N_MASK	0x00800000
#define PSR_ICC_Z_MASK	0x00400000
#define PSR_ICC_V_MASK	0x00200000
#define PSR_ICC_C_MASK	0x00100000
#define PSR_ICC_EC	0x00002000
#define PSR_ICC_EF	0x00001000
#define PSR_ICC_PIL	0x00000F00
#define PSR_ICC_S	0x00000080
#define PSR_ICC_PS	0x00000040
#define PSR_ICC_ET	0x00000020
#define PSR_ICC_CWP_MASK 0x0000001F

/* Trap Base Register TBR */
/* number of bits to shift left to obtain TT value */
#define TBR_TT_SHIFT	4		
/* TT mask: the mask for TBR is thus TT_MASK << TT_SHIFT */
#define TBR_TT_MASK	0xff

/* Leon Configuration Register */
/* number of bits to shift left to obtain NWINDOWS value */
#define LCONF_NWINDOWS_SHIFT 20  
/* NWINDOWS mask: the mask for LCONF is thus NWINDOWS_MASK << NWINDOWS_SHIFT */
#define LCONF_NWINDOWS_MASK 0x1f

#define IRQ_AHB         1
#define IRQ_UART2       2
#define IRQ_UART1       3
#define IRQ_PARALLEL_0  4
#define IRQ_PARALLEL_1  5
#define IRQ_PARALLEL_2  6
#define IRQ_PARALLEL_3  7
#define IRQ_TIMER1      8
#define IRQ_TIMER2      9

#define TIMER_LD (1<<2)
#define TIMER_RL (1<<1)
#define TIMER_EN (1<<0)
