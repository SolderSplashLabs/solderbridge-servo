
/* SSP Status register */
#define SSPSR_TFE       (0x1<<0)
#define SSPSR_TNF       (0x1<<1)
#define SSPSR_RNE       (0x1<<2)
#define SSPSR_RFF       (0x1<<3)
#define SSPSR_BSY       (0x1<<4)

/* SSP CR0 register */
#define SSPCR0_DSS      (0x1<<0)
#define SSPCR0_FRF      (0x1<<4)
#define SSPCR0_SPO      (0x1<<6)
#define SSPCR0_SPH      (0x1<<7)
#define SSPCR0_SCR      (0x1<<8)

/* SSP CR1 register */
#define SSPCR1_LBM      (0x1<<0)
#define SSPCR1_SSE      (0x1<<1)
#define SSPCR1_MS       (0x1<<2)
#define SSPCR1_SOD      (0x1<<3)

/* SSP Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM   (0x1<<0)
#define SSPIMSC_RTIM    (0x1<<1)
#define SSPIMSC_RXIM    (0x1<<2)
#define SSPIMSC_TXIM    (0x1<<3)

/* SSP0 Interrupt Status register */
#define SSPRIS_RORRIS   (0x1<<0)
#define SSPRIS_RTRIS    (0x1<<1)
#define SSPRIS_RXRIS    (0x1<<2)
#define SSPRIS_TXRIS    (0x1<<3)

/* SSP0 Masked Interrupt register */
#define SSPMIS_RORMIS   (0x1<<0)
#define SSPMIS_RTMIS    (0x1<<1)
#define SSPMIS_RXMIS    (0x1<<2)
#define SSPMIS_TXMIS    (0x1<<3)

/* SSP0 Interrupt clear register */
#define SSPICR_RORIC    (0x1<<0)
#define SSPICR_RTIC     (0x1<<1)


// SolderSplash BaseBoard to SolderBridge Command list
enum SPI_COMMANDS
{
	SB_WHOS_THERE = 0x01,
	SB_SERVO_MOVE = 0x10,
	SB_SERVO_TIME,
	SB_SERVO_HOME,
	SB_SERVO_MANAGE,
	SB_DMX_UPDATE = 0x20,
	SB_DMX_SCENE,
	SB_DMX_MODE,
	SB_DMX_MANAGE,
	SB_ZEROTEN_UPDATE = 0x30
};
