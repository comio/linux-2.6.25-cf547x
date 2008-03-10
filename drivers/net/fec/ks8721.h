
#define   FEC_MII_SPEED              (((unsigned int)(MCF_BUSCLK / 5000000)) << 1)

// Numbers of the transceiver registers 
#define   KS8721_CTRL                0x00
#define   KS8721_ANADV               0x04
#define   KS8721_STAT                0x01

// Register values
#define   KS8721_CTRL_RESET          0x8000
#define   KS8721_ANADV_ADV_ALL       0x01E1
#define   KS8721_CTRL_AN_ENABLE      0x1280
#define   KS8721_CTRL_DEFAULT_MODE   0x2100
#define   KS8721_STAT_ANCOMPLETE     0x0020
#define   KS8721_STAT_LINK           0x0004
#define   KS8721_STAT_FDUPLEX        0x5000

// Timeout for the auto-negotiation mode
#define   KS8721_TIMEOUT             5

int ks8721_init_transceiver(unsigned long base_addr, int *fduplex);
