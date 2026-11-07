// SPDX-License-Identifier: GPL-2.0+
/*
 * Display bring-up for the Apple iPod touch (2nd generation), N72AP.
 *
 * The S5L8720 scans a framebuffer out through the CLCD controller into a
 * MIPI DSI master, which drives the "pinot" panel. iBSS leaves the display
 * off, so this driver does the whole bring-up. The sequence and constants
 * follow openiBoot's plat-s5l8720 (clcd.c, mipi_dsim.c, gpio.c, i2c.c, pmu.c).
 */

#include <dm.h>
#include <video.h>
#include <asm/io.h>
#include <asm/arch-s5l87xx/clk.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <time.h>

#define N72_FB_BASE		0x0fc00000
#define N72_WIDTH		320
#define N72_HEIGHT		480
#define N72_HBP			12
#define N72_HFP			12
#define N72_HSW			16
#define N72_VBP			6
#define N72_VFP			6
#define N72_VSW			8
#define N72_PIXCLK		10800000	/* 0xa4cb80 */
#define N72_DSI_LANES		2
#define N72_DSI_LANE_MASK	(BIT(N72_DSI_LANES) - 1)

/* CLCD */
#define CLCD			0x38900000

/* MIPI DSIM */
#define DSIM			0x3d800000
#define DSIM_STATUS		0x00
#define DSIM_SWRST		0x04
#define DSIM_CLKCTRL		0x08
#define DSIM_CONFIG		0x10
#define DSIM_ESCMODE		0x14
#define DSIM_MDRESOL		0x18
#define DSIM_MVPORCH		0x1c
#define DSIM_MHPORCH		0x20
#define DSIM_MSYNC		0x24
#define DSIM_SDRESOL		0x28
#define DSIM_INTSRC		0x2c
#define DSIM_PKTHDR		0x34
#define DSIM_RXFIFO		0x3c
#define DSIM_FIFOTHLD		0x40
#define DSIM_FIFOCTRL		0x44
#define DSIM_PLLCTRL		0x4c

#define STATUS_STOP		BIT(8)
#define STATUS_ULPS		BIT(9)
#define STATUS_HS_READY		BIT(10)
#define STATUS_SWRST		BIT(20)
#define STATUS_DATA_STOP(x)	((x) & 0xf)
#define STATUS_DATA_ULPS(x)	(((x) & 0xf) << 4)

#define CLKCTRL_ESC_EN_CLK	BIT(19)
#define CLKCTRL_ESC_EN_DATA(x)	(((x) & 0xf) << 20)
#define CLKCTRL_BYTE_CLKEN	BIT(24)
#define CLKCTRL_BYTE_CLK_SRC	BIT(25)
#define CLKCTRL_PLL_BYPASS	BIT(27)
#define CLKCTRL_ESC_CLKEN	BIT(28)
#define CLKCTRL_TX_REQ_HSCLK	BIT(31)

#define ESCMODE_TX_UIPS_CLK_EXIT BIT(0)
#define ESCMODE_TX_UIPS_CLK	BIT(1)
#define ESCMODE_TX_UIPS_EXIT	BIT(2)
#define ESCMODE_TX_UIPS_DAT	BIT(3)
#define ESCMODE_CMD_LP		BIT(7)
#define ESCMODE_FORCE_STOP	BIT(20)

/* GPIO */
#define GPIO_BASE		0x3cf00000
#define GPIO_FSEL		0x1e0
#define GPIOIC			0x39700000
#define N72_PANEL_RESET_GPIO	0x107

/* I2C0, which carries the PMU */
#define I2C0			0x3c600000
#define IICCON			0x00
#define IICSTAT			0x04
#define IICDS			0x0c
#define IICREG20		0x20
#define IICCON_INIT		0x3f00
#define IICCON_ACKGEN		BIT(7)
#define IICCON_FPCLK512		BIT(6)
#define IICCON_INTPENDING	BIT(4)
#define IICSTAT_LASTRCVBIT	BIT(0)
#define I2C_OP_SEND		BIT(8)
#define I2C_OP_CONDCHANGE	BIT(13)
#define I2C0_SCL_GPIO		0x207
#define I2C0_SDA_GPIO		0x300
#define PMU_I2C_ADDR		0xe6	/* 8-bit write address */

#define POLL_US			200000

struct gamma_table {
	u32 match;
	u32 mask;
	u32 data[3][18];
};

/* openiBoot PinotGammaTables, selected by panel ID. */
static const struct gamma_table pinot_gamma[] = {
	{0x970387, 0xFFFFFF, {
		{0x4041D422, 0x4001DD07, 0xDDDC003, 0x1C0, 0xDDDDDC00, 0x7774, 0xDC701C00, 0x74DDC771, 0x704D34D3, 0x7000, 0, 0x1C000000, 0x1DDC7777, 0xD000D00, 0x34D371D, 0x8A5D034D, 0xEB42, 0},
		{0x7474422, 0x1C774074, 0x34D31C1C, 0x1C0, 0xDC770070, 0x4001DDDD, 0xC0000D03, 0xC1C70001, 0x71C71D, 0xD00000, 0x40D00034, 0xD00D03, 0xC74D0700, 0x771DC771, 0x3777, 0xE904AD1C, 0xCFAC, 0},
		{0x101D422, 0x1C7741D, 0x34C70000, 0x34000, 0xDDDC7000, 0x40001DDD, 0xC0700003, 0x37777071, 0x1DD34DD, 0x1C070707, 0x1C1C1C70, 0x71C70707, 0x74DDD377, 0x3470, 0x377774D0, 0x92804D0D, 0xE843, 0},
	}},
	{0xE50387, 0xFFFFFF, {
		{0x40441122, 0xD0D1C40, 0xC34CDC7D, 0x4DC71C71, 0x40D00D03, 0xD34D347, 0xD, 0xC71C71C0, 0x340D0D01, 0x40340340, 0xC0003403, 0x374DD371, 0x701C071D, 0xDC, 0xDC410000, 0xC0041DDD, 0xE8, 0},
		{0x1CAD1122, 0x435C72B1, 0x37DC0037, 0xDC71DC00, 0x343434DD, 0x74, 0xC001C00D, 0xDC71C701, 0xC71D001D, 0xC71DC771, 0x9377771D, 0xD0000DDE, 0xDDD34000, 0x71DDDD, 0x1D00007, 0xA771C000, 0x7A7037, 0},
		{0x40441122, 0x34340740, 0xDC3700, 0x4DC71C77, 0xD00003, 0x34D34D1D, 0xD, 0x71C71C70, 0xDDDDDD00, 0xDDDDDDDD, 0x70D374DD, 0x1DDDDDDC, 0x1C01C007, 0x40D03437, 0xD00003, 0x80157400, 0xE9E, 0},
	}},
	{0xC20385, 0xFFFFFF, {
		{0xB4044122, 0xDDDD0772, 0xC3700371, 0x71, 0x1C0000, 0x34000007, 0x70001C00, 0x7000, 0, 0x70000, 0xD0000000, 0x7000000, 0x1C0, 0x40343400, 0x1C707003, 0xDC4D0007, 0xE9C, 0},
		{0x74044122, 0x771D001D, 0xC0030037, 0x7001D, 0x7001C0, 0, 0xC70701C0, 0x1C01, 0x1C00, 0x1C001C0, 0xC01C01C0, 0x1C01, 0x1C00700, 0x7001C, 0xC0007007, 0x7D011001, 0x3A, 0},
		{0x1D744122, 0xDC74D1C4, 0xC00DD, 7, 0x1C0001C0, 0, 0x701C000, 0x7001C007, 0, 0x700007, 0x70001C0, 0x70701C00, 0x70070, 0x701C007, 0x71C1C1C, 0xCC4411C7, 0xC, 0},
	}},
	{0xD10384, 0xFFFFFF, {
		{0xB4110522, 0x3434772, 0xD374C030, 0x73777771, 0x1C771C, 0x4D343434, 0xD34DD37, 0x7A40000, 0x70000, 0x701C0070, 0xDDC71C70, 0x4343434D, 0xDDC77477, 0x71C707A4, 0xD0D371C, 0x71D34, 0xCDF4, 0},
		{0xB4110522, 0xC0D00132, 0x1C00C0D, 0xD34D0D34, 0x400D0000, 0x4D3434D3, 0x403434D3, 3, 0x1C000, 0x1C00000, 0x77070700, 0x1374D377, 0xD00D07, 0xD3771DDC, 0x771C71DD, 0xD000137, 0xF7, 0},
		{0x1110522, 0x371C7401, 0x10035F70, 0xD0D37003, 0x771C74D0, 0xDDDDDDDC, 0xD0034, 0x700000, 0xC1C07070, 0x71C71DC1, 0xDDDDDDDC, 0x3471DD, 0xD0D3000, 0xD0D0D, 0x40000340, 0xC0D00D03, 0, 0},
	}},
	{0x970787, 0xFFFFFF, {
		{0x40135022, 0x1D001D00, 0x71D3400, 0x71C07007, 0x771C0D37, 0x34D0D3, 0x71C00000, 0xD01DC71C, 0xC774D374, 0x71DDDDDD, 0xD0CD377, 0xC1DDD34D, 0xDC701C01, 0x37774DDD, 0x34D0D34D, 0x4040340D, 3, 0},
		{0xD1DD1022, 0x7401DD71, 0x74D00007, 0xD34DDDD3, 0xDDDC00, 0x4D34D340, 0x34003, 0x1C0070, 0x40D00000, 0xD034343, 0x7343434D, 0x71DC770, 0xD0343400, 0x7000001C, 0x1C01C000, 0xCC70000, 0, 0},
		{0x1D01022, 0x1D34710, 0x1C74D00, 0x701C0007, 0x7A40D31C, 0x1C00, 0x70D0034, 0xDC7001C0, 0x1C7771DD, 0, 0, 0xC0000D00, 0x77771DDD, 0x34340D0, 0x340, 0xA174D0D0, 0xC44, 0},
	}},
	{0xD10784, 0xFFFFFF, {
		{0x2B1D1022, 0x74013, 0x4D035C1D, 0x374DDD3, 0xDDC70D, 0xD03401C, 0xD374DD0D, 0x340D, 0, 0x1C001C, 0x700070, 0x1C0007, 0xD00032B0, 0xDDDDC700, 0x1C713774, 0x1DD071C7, 0xDC7, 0},
		{0x40135022, 0x74D71D07, 0xD34D0007, 0xD34DDD, 0xDC00DDC0, 1, 0x34D34D0D, 0xD000D, 0x70000000, 0, 0, 0x34000D0, 0x34DD374, 0x1C71DC70, 0x1DC771C7, 0x70707007, 0xC, 0},
		{0x741022, 0x1D35C4, 0xD0701C4D, 0x34000, 0xC00D31C7, 0xD3740001, 0x77771374, 0x340D377, 0xD, 0x734, 0x7000000, 0, 0x40D0D000, 0xDC771C73, 0x70000071, 0xD0340D0, 0, 0},
	}},
	{0xC20785, 0xFFFFFF, {
		{0x74740422, 0x1D00740, 0xC01C7740, 0xDC700001, 0x34DC734D, 0xC0000000, 0x7771C1C1, 0x774DD377, 0x71C71C1C, 0x1C71C71C, 0xC71C71C7, 0x701C071, 0xD0007, 0x7070000, 0x71C1C707, 0x70401C, 0xC, 0},
		{0x74740422, 0x74004D7, 0x1C777400, 0x371C7070, 0x34C700D, 0xC0000700, 0xC71C71C1, 0xDD3774DD, 0xC0701C1D, 0x1C0701C1, 0x70070070, 0x1C1C0700, 0xD00D070, 0x1C000000, 0x7070700, 0x704007, 0xC, 0},
		{0x40135022, 0xDD01DD07, 0x774D0001, 0x34DDDC77, 0x377000, 0x3400, 0xC0701C00, 0x77777771, 0x1C077, 0x7001C007, 0x7001C00, 0xC01C0070, 0xD0340001, 0x340, 0xC07001C0, 0x1C1D001, 0xC, 0},
	}},
	{0xC20784, 0xFFFFFF, {
		{0x1D01022, 0x1D0074, 0x341C1DD0, 0xD34D34D0, 0x70000034, 0x7070701C, 0x377771DC, 0x3400D00D, 0x70001C, 0xDC71C1C7, 0x7771DC71, 0x77771DDC, 0xD034007, 0x71C71C00, 0x1DC771C7, 0x701D077, 0xC, 0},
		{0x1D01022, 0x1C740010, 0xB0071D34, 0xD00032, 0x771C0000, 0x77774DD3, 0xD3777777, 0xD0034D, 0x1C0000, 0x701C0070, 0x1C0701C0, 0x1C1C1C1C, 0xDC74001C, 0x1C000DD, 0x7771C707, 0x7171DC, 0, 0},
		{0x7401022, 0x74D0774, 0x1C1DDD00, 0xD000D000, 0x71C1C000, 0x71C71DC7, 0x7777771C, 0x3400D037, 0x7000000, 0x70701C0, 0xC7070707, 0xC71C1C71, 0x4DD34071, 0xC0700003, 0xC1C1C1C1, 0x1C75C1, 0xC, 0},
	}},
};

/* Poll until (readl(reg) & mask) == want, or time out. */
static int n72_poll(ulong reg, u32 mask, u32 want, const char *what)
{
	ulong start = timer_get_us();

	while ((readl(reg) & mask) != want) {
		if (timer_get_us() - start > POLL_US) {
			printf("n72-display: timeout waiting for %s (reg %08lx = %08x)\n",
			       what, reg, readl(reg));
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static void n72_frame_delay(int frames)
{
	/* (6+6+8+480) * (12+12+16+320) / 10.8 MHz = 16.7 ms per frame */
	mdelay(frames * 17);
}

/* GPIO */

static void n72_gpio_custom_io(u32 port, u32 bits)
{
	writel((((port >> 8) & 0x1f) << 16) | ((port & 0x7) << 8) | (bits & 0xf),
	       GPIO_BASE + GPIO_FSEL);
}

static void n72_gpio_output(u32 port, int value)
{
	n72_gpio_custom_io(port, 0xe | !!value);
}

static int n72_gpio_setup(void)
{
	/* iBoot writes these on the S5L8720 before using the GPIO block. */
	writel(0x32, GPIOIC + 0x70);
	writel(3, GPIOIC + 0x6c);
	if (n72_poll(GPIOIC + 0x7c, 1, 1, "GPIOIC ready"))
		return -ETIMEDOUT;
	writel(2, GPIOIC + 0x6c);
	return 0;
}

/* I2C0 master transmit, polled, following openiBoot's i2c.c state machine. */

static int n72_i2c_wait(u32 op)
{
	ulong start = timer_get_us();
	u32 status;

	for (;;) {
		status = readl(I2C0 + IICREG20);
		if (status)
			writel(status, I2C0 + IICREG20);
		if (status & op)
			return 0;
		if (timer_get_us() - start > POLL_US) {
			printf("n72-display: I2C timeout (op %x, stat %08x)\n",
			       op, readl(I2C0 + IICSTAT));
			return -ETIMEDOUT;
		}
	}
}

static void n72_i2c_init(void)
{
	int i;

	s5l87xx_enable_clkgate_bit(1, 4);	/* gate 7: I2C0 */
	s5l87xx_enable_clkgate_bit(4, 11);

	/* Clock out any stuck transfer, then hand the pins to the controller. */
	n72_gpio_custom_io(I2C0_SDA_GPIO, 0xe);
	for (i = 0; i < 19; i++) {
		n72_gpio_custom_io(I2C0_SCL_GPIO, (i % 2) ? 0xe : 0x0);
		udelay(5);
	}
	n72_gpio_custom_io(I2C0_SCL_GPIO, 0x2);
	n72_gpio_custom_io(I2C0_SDA_GPIO, 0x2);
}

static int n72_i2c_tx(u8 addr, const u8 *buf, int len)
{
	/* Slowest divider: the peripheral clock is not known here. */
	u32 con = IICCON_INIT | IICCON_FPCLK512 | 0xf;
	u32 stat = (3 << 6) | (1 << 5) | (1 << 4);	/* master TX, START, out */
	int i, ret = 0;

	writel(con, I2C0 + IICCON);
	con |= IICCON_ACKGEN;
	writel(IICCON_INIT, I2C0 + IICREG20);

	writel(con, I2C0 + IICCON);
	writel(addr & ~1, I2C0 + IICDS);
	writel(stat, I2C0 + IICSTAT);
	ret = n72_i2c_wait(I2C_OP_SEND);

	for (i = 0; !ret && i <= len; i++) {
		if (readl(I2C0 + IICSTAT) & IICSTAT_LASTRCVBIT) {
			printf("n72-display: I2C NACK at byte %d\n", i);
			ret = -EIO;
			break;
		}
		if (i == len)
			break;
		writel(buf[i], I2C0 + IICDS);
		writel(con | IICCON_INTPENDING, I2C0 + IICCON);
		ret = n72_i2c_wait(I2C_OP_SEND);
	}

	/* STOP */
	stat &= ~(1 << 5);
	stat &= ~(1 << 6);
	writel(stat, I2C0 + IICSTAT);
	writel(con | IICCON_INTPENDING, I2C0 + IICCON);
	if (n72_i2c_wait(I2C_OP_CONDCHANGE) && !ret)
		ret = -ETIMEDOUT;

	return ret;
}

static int n72_pmu_write(u8 reg, u8 val)
{
	u8 cmd[2] = { reg, val };

	return n72_i2c_tx(PMU_I2C_ADDR, cmd, sizeof(cmd));
}

/* MIPI DSIM */

static int n72_dsim_write(u8 id, u8 d0, u8 d1)
{
	writel(id | (d0 << 8) | (d1 << 16), DSIM + DSIM_PKTHDR);
	return n72_poll(DSIM + DSIM_FIFOCTRL, BIT(22), BIT(22), "DSIM header FIFO");
}

static int n72_dsim_read(u8 id, u8 *buf, u32 *len)
{
	u32 intsrc, rx, size, want = *len;
	int i;

	writel(0xffffffff, DSIM + DSIM_INTSRC);
	if (n72_dsim_write(id, buf[0], buf[1]))
		return -ETIMEDOUT;

	for (i = 0; ; i++) {
		intsrc = readl(DSIM + DSIM_INTSRC);
		if (intsrc & (BIT(16) | BIT(21))) {
			printf("n72-display: DSIM read error, INTSRC %08x\n", intsrc);
			return -EIO;
		}
		if (intsrc & BIT(18))
			break;
		if (i == 4) {
			printf("n72-display: DSIM read timeout, INTSRC %08x\n", intsrc);
			return -ETIMEDOUT;
		}
		mdelay(50);
	}

	if (n72_poll(DSIM + DSIM_FIFOCTRL, BIT(24), 0, "DSIM RX FIFO"))
		return -ETIMEDOUT;
	rx = readl(DSIM + DSIM_RXFIFO);
	if ((rx & 0x3f) != 26 && (rx & 0x3f) != 28) {
		printf("n72-display: DSIM unexpected response %08x\n", rx);
		return -EIO;
	}

	size = (rx >> 8) & 0xffff;
	if (want > size)
		*len = size;
	for (i = 0; i < size; i++) {
		if (!(i % 4)) {
			if (n72_poll(DSIM + DSIM_FIFOCTRL, BIT(24), 0, "DSIM RX FIFO"))
				return -ETIMEDOUT;
			rx = readl(DSIM + DSIM_RXFIFO);
		}
		if (i < want)
			buf[i] = rx & 0xff;
		rx >>= 8;
	}
	return 0;
}

static int n72_dsim_init(void)
{
	const u32 stop = STATUS_STOP | STATUS_DATA_STOP(N72_DSI_LANE_MASK);
	const u32 ulps = STATUS_ULPS | STATUS_DATA_ULPS(N72_DSI_LANE_MASK);
	ulong d = DSIM;

	s5l87xx_enable_clkgate_bit(1, 19);	/* gate 0xa: MIPI DSIM */

	/* N72 table word 0x222: 2 lanes, escape prescaler 2, PLL bypassed. */
	writel((2 << 0) | CLKCTRL_ESC_CLKEN | CLKCTRL_PLL_BYPASS |
	       CLKCTRL_BYTE_CLK_SRC, d + DSIM_CLKCTRL);
	writel(0x02000000, d + DSIM_PLLCTRL);
	writel(1, d + DSIM_SWRST);
	if (n72_poll(d + DSIM_STATUS, STATUS_SWRST, STATUS_SWRST, "DSIM reset"))
		return -ETIMEDOUT;

	writel(BIT(31) | (N72_HEIGHT << 16) | N72_WIDTH, d + DSIM_MDRESOL);
	writel((0xd << 28) | (N72_VFP << 16) | N72_VBP, d + DSIM_MVPORCH);
	writel(N72_VSW << 22, d + DSIM_MSYNC);
	writel((N72_HFP << 16) | N72_HBP, d + DSIM_MHPORCH);
	writel((N72_VSW << 22) | N72_HSW, d + DSIM_MSYNC);
	writel(10, d + DSIM_SDRESOL);
	writel(0x7807027, d + DSIM_CONFIG);
	setbits_le32(d + DSIM_CLKCTRL, CLKCTRL_BYTE_CLKEN | CLKCTRL_ESC_EN_CLK |
		     CLKCTRL_ESC_EN_DATA(0x3));
	writel(0x1f, d + DSIM_FIFOCTRL);
	writel(0x1ff, d + DSIM_FIFOTHLD);

	writel(ESCMODE_FORCE_STOP | ESCMODE_CMD_LP, d + DSIM_ESCMODE);
	udelay(1000);
	clrbits_le32(d + DSIM_ESCMODE, ESCMODE_FORCE_STOP);
	if (n72_poll(d + DSIM_STATUS, stop, stop, "DSIM stop state"))
		return -ETIMEDOUT;

	writel(ESCMODE_CMD_LP | ESCMODE_TX_UIPS_DAT | ESCMODE_TX_UIPS_CLK,
	       d + DSIM_ESCMODE);
	if (n72_poll(d + DSIM_STATUS, ulps, ulps, "DSIM ULPS entry"))
		return -ETIMEDOUT;

	clrsetbits_le32(d + DSIM_CONFIG, (0xf << 1) | (0x3 << 5),
			(N72_DSI_LANE_MASK << 1) | ((N72_DSI_LANES - 1) << 5));
	clrsetbits_le32(d + DSIM_CLKCTRL, CLKCTRL_ESC_EN_DATA(0xf),
			CLKCTRL_ESC_EN_DATA(N72_DSI_LANE_MASK));

	setbits_le32(d + DSIM_ESCMODE, ESCMODE_TX_UIPS_EXIT |
		     ESCMODE_TX_UIPS_CLK_EXIT);
	if (n72_poll(d + DSIM_STATUS, ulps, 0, "DSIM ULPS exit"))
		return -ETIMEDOUT;
	clrbits_le32(d + DSIM_ESCMODE, 0x5);
	udelay(1000);
	clrbits_le32(d + DSIM_ESCMODE, 0xa);
	return n72_poll(d + DSIM_STATUS, stop, stop, "DSIM stop after ULPS");
}

/* CLCD */

static void n72_clcd_enable(void)
{
	setbits_le32(CLCD, 1);
	/* openiBoot: the 2G also needs this second enable bit. */
	setbits_le32(CLCD + 0x1b10, 1);
}

static u8 gamma_bits(const u8 *table, int *pos, int *shift, u8 *cur)
{
	u8 v;

	if (*shift == 0)
		*cur = table[(*pos)++];
	v = (*cur >> *shift) & 0x3;
	*shift += 2;
	if (*shift == 8)
		*shift = 0;
	return v;
}

/* Decode one openiBoot gamma table into 257 cumulative entries. */
static void gamma_decode(const u32 *data, u32 *out)
{
	const u8 *table = (const u8 *)data;
	int pos = 0, shift = 0, i;
	u8 cur = 0, step = 0, r8 = 0;
	u16 r6 = 0;

	out[0] = 0;
	for (i = 1; i < 257; i++) {
		switch (gamma_bits(table, &pos, &shift, &cur)) {
		case 0:
			step = 0;
			break;
		case 1:
			step = 1;
			break;
		case 2: {
			u8 a = gamma_bits(table, &pos, &shift, &cur);
			u8 b = gamma_bits(table, &pos, &shift, &cur);

			step = a | (b << 2);
			if (step & BIT(3))
				step |= 0xf0;
			break;
		}
		case 3:
			step = 0xff;
			break;
		}
		if (i == 1)
			step += 8;
		r8 += step;
		r6 += r8;
		out[i] = r6;
	}
}

static void n72_set_window_lut(int window, const u32 *buf)
{
	int i;

	writel((window << 12) | 0x10000, CLCD + 0x408);
	for (i = 0; i < 256; i++) {
		u32 v = (buf[i + 1] & 0x3ff) | ((buf[i] << 10) & 0xffc00);

		writel(v | BIT(31), CLCD + 0x40c);
	}
	writel(0, CLCD + 0x408);
}

static void n72_install_gamma(u32 panel_id)
{
	static u32 lut[3][257];
	const struct gamma_table *g = NULL;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(pinot_gamma); i++) {
		if ((pinot_gamma[i].mask & panel_id) == pinot_gamma[i].match) {
			g = &pinot_gamma[i];
			break;
		}
	}

	for (j = 0; j < 3; j++) {
		if (g) {
			gamma_decode(g->data[j], lut[j]);
		} else {
			lut[j][0] = 0;
			for (i = 1; i < 257; i++)
				lut[j][i] = 4 * (i - 1);
		}
		n72_set_window_lut(4 + j, lut[j]);
	}
	if (g)
		printf("n72-display: gamma table %06x\n", g->match);
	else
		printf("n72-display: no gamma table for panel %08x, using linear\n",
		       panel_id);

	writel(1, CLCD + 0x400);
	writel(128, CLCD + 0x1b38);
}

static int n72_clcd_init(void)
{
	s5l87xx_enable_clkgate_bit(0, 14);	/* gate 2: CLCD */
	s5l87xx_enable_clkgate_bit(3, 9);
	s5l87xx_enable_clkgate_bit(3, 10);
	s5l87xx_enable_clkgate_bit(3, 11);
	s5l87xx_enable_clkgate_bit(3, 14);

	writel(0x100, CLCD);
	if (n72_poll(CLCD, 0x100, 0, "CLCD reset"))
		return -ETIMEDOUT;
	udelay(1);

	writel(0x20084, CLCD);
	writel(0x80000001, CLCD + 0x300);
	writel(0, CLCD + 0x1b10);
	writel(0, CLCD + 0x1b14);
	writel(((N72_VBP - 1) << 16) | ((N72_VFP - 1) << 8) | (N72_VSW - 1),
	       CLCD + 0x1b1c);
	writel(((N72_HBP - 1) << 16) | ((N72_HFP - 1) << 8) | (N72_HSW - 1),
	       CLCD + 0x1b20);
	writel(((N72_WIDTH - 1) << 16) | (N72_HEIGHT - 1), CLCD + 0x1b24);

	/* Window 0: 32 bpp XRGB at the framebuffer. */
	writel((7 << 8) | 0x200000, CLCD + 0x20);
	writel(N72_FB_BASE, CLCD + 0x24);
	writel(N72_WIDTH, CLCD + 0x28);
	writel(0, CLCD + 0x2c);
	writel((N72_WIDTH << 16) | N72_HEIGHT, CLCD + 0x30);
	writel(0, CLCD + 0x38);
	writel(0, CLCD + 0x3c);
	writel(0, CLCD + 0x34);
	setbits_le32(CLCD + 0x4, 0x10);
	setbits_le32(CLCD + 0x10, 0x80);
	return 0;
}

static int n72_pinot_init(u32 *panel_id)
{
	u8 id[16] = { 0xb1 };
	u32 len = 15, kind;
	int ret;

	n72_gpio_output(N72_PANEL_RESET_GPIO, 0);
	mdelay(10);
	ret = n72_dsim_init();
	if (ret)
		return ret;
	n72_gpio_output(N72_PANEL_RESET_GPIO, 1);
	mdelay(6);
	ret = n72_dsim_write(5, 0, 0);
	if (ret)
		return ret;
	udelay(10);

	/* Start the high-speed clock. */
	setbits_le32(DSIM + DSIM_CLKCTRL, CLKCTRL_TX_REQ_HSCLK);
	ret = n72_poll(DSIM + DSIM_STATUS, STATUS_HS_READY, STATUS_HS_READY,
		       "DSIM HS clock");
	if (ret)
		return ret;
	mdelay(25);

	ret = n72_dsim_read(0x14, id, &len);
	if (ret || len <= 2) {
		printf("n72-display: panel ID read failed (%d, %u bytes)\n", ret, len);
		return ret ? ret : -EIO;
	}
	*panel_id = (id[0] << 24) | (id[1] << 16) | ((id[3] & 0xf0) << 8) |
		    ((id[2] & 0xf8) << 4) | ((id[3] & 0xf) << 3) | (id[2] & 0x7);
	printf("n72-display: panel ID %08x, backlight cal %02x\n", *panel_id, id[5]);

	kind = (*panel_id >> 8) & 0xff;
	if (kind - 3 <= 1 || kind == 7 || kind == 8)
		setbits_le32(DSIM + DSIM_MDRESOL, BIT(31));

	n72_clcd_enable();
	udelay(100);

	ret = n72_dsim_write(5, 0x11, 0);	/* exit sleep */
	if (ret)
		return ret;
	n72_frame_delay(7);
	ret = n72_dsim_write(5, 0x29, 0);	/* display on */
	if (ret)
		return ret;
	n72_frame_delay(4);
	setbits_le32(DSIM + DSIM_MDRESOL, BIT(31));
	return 0;
}

static int n72_display_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	u32 panel_id = 0;
	int ret;

	plat->base = N72_FB_BASE;
	plat->size = N72_WIDTH * N72_HEIGHT * 4;
	memset((void *)plat->base, 0, plat->size);

	ret = n72_gpio_setup();
	if (ret)
		return ret;
	ret = n72_clcd_init();
	if (ret)
		return ret;
	ret = n72_pinot_init(&panel_id);
	if (ret)
		return ret;
	n72_install_gamma(panel_id);

	n72_i2c_init();
	ret = n72_pmu_write(0x30, 0x60);	/* backlight level */
	if (!ret)
		ret = n72_pmu_write(0x31, 0x05);	/* backlight on */
	if (ret)
		printf("n72-display: backlight PMU write failed (%d)\n", ret);

	uc_priv->xsize = N72_WIDTH;
	uc_priv->ysize = N72_HEIGHT;
	uc_priv->bpix = VIDEO_BPP32;
	uc_priv->format = VIDEO_X8R8G8B8;
	video_set_flush_dcache(dev, true);

	printf("n72-display: %dx%d at %08lx\n", N72_WIDTH, N72_HEIGHT, plat->base);
	return 0;
}

static const struct udevice_id n72_display_ids[] = {
	{ .compatible = "apple,n72-display" },
	{ }
};

U_BOOT_DRIVER(n72_display) = {
	.name		= "n72_display",
	.id		= UCLASS_VIDEO,
	.of_match	= n72_display_ids,
	.probe		= n72_display_probe,
};
