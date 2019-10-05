/*	$NetBSD$	*/

/**
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2017 aQuantia Corporation. All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   (1) Redistributions of source code must retain the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer.
 *
 *   (2) Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 *
 *   (3) The name of the author may not be used to endorse or promote
 *   products derived from this software without specific prior
 *   written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*-
 * Copyright (c) 2019 Ryo Shimizu <ryo@nerv.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/types.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/cprng.h>
#include <sys/device.h>
#include <sys/errno.h>
#include <sys/intr.h>
#include <sys/ioctl.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/systm.h>

#include <machine/endian.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_ether.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

#define GLB_STANDARD_CTL1_ADR			0x0000
#define GLB_SOFT_RES_ADR			GLB_STANDARD_CTL1_ADR
#define  GLB_SOFT_RES_MSK			__BIT(15) /* soft reset bit */
#define GLB_REG_RES_DIS_ADR			GLB_STANDARD_CTL1_ADR
#define  GLB_REG_RES_DIS_MSK			__BIT(14) /* reset disable */

#define HW_ATL_GLB_SOFT_RES_ADR			0x0000
#define HW_ATL_MPI_FW_VERSION			0x0018
#define  FW2X_FW_MIN_VER_LED			0x03010026
#define GLB_FW_IMAGE_ID1_ADR			0x0018
#define HW_ATL_GLB_MIF_ID_ADR			0x001c
#define GLB_NVR_INTERFACE1_ADR			0x0100
#define HW_ATL_MIF_CMD				0x0200
#define  HW_ATL_MIF_CMD_EXECUTE				0x00008000
#define  HW_ATL_MIF_CMD_BUSY				0x00000100
#define HW_ATL_MIF_ADDR				0x0208
#define HW_ATL_MIF_VAL				0x020c
#define HW_ATL_GLB_CPU_SCRATCH_SCP_ADR(i)	(0x0300 + (i) * 0x4)

#define FW2X_MPI_LED_ADDR			0x031c
#define  FW2X_LED_BLINK				0x2
#define  FW2X_LED_DEFAULT			0x0

#define HW_ATL_FW2X_MPI_EFUSE_ADDR		0x0364
#define FW2X_MPI_CONTROL_ADDR			0x0368	/* 64bit */
//#define FW2X_MPI_CONTROL2_ADDR  		0x036c
#define FW2X_MPI_STATE_ADDR			0x0370	/* 64bit */
//#define FW2X_MPI_STATE2_ADDR			0x0374
#define HW_ATL_UCP_0X370_REG			FW2X_MPI_STATE_ADDR
#define HW_ATL_FW1X_MPI_EFUSE_ADDR		0x0374
#define HW_ATL_MPI_BOOT_EXIT_CODE		0x0388
#define   RBL_STATUS_DEAD				0xdead
#define   RBL_STATUS_SUCCESS				0xabba
#define   RBL_STATUS_FAILURE				0x0bad
#define   RBL_STATUS_HOST_BOOT				0xf1a7

//#define GLB_CPU_NO_RESET_SCRATCHPAD_ADR(i)	(0x0380 + (i) * 0x4)
//#define  NO_RESET_SCRATCHPAD_RBL_STATUS		2	// = 0x0388 = HW_ATL_MPI_BOOT_EXIT_CODE
//#define   NO_RESET_SCRATCHPAD_RBL_STATUS_MAGIC_DEAD	0xdead
//#define   RBL_STATUS_SUCCESS				0xabba
//#define   RBL_STATUS_FAILURE				0x0bad
//#define   RBL_STATUS_HOST_BOOT				0xf1a7

#define HW_ATL_GLB_CPU_SEM_ADR(i)		(0x03a0 + (i) * 0x4)
#define  HW_ATL_FW_SM_RAM			2	// = 0x03a8

#define HW_ATL_MCP_UP_FORCE_INTERRUPT_ADR	0x0404
#define GLB_CTL2_ADR				0x0404

#define GLB_GENERAL_PROVISIONING9_ADR		0x0520
#define GLB_NVR_PROVISIONING2_ADR		0x0534

#define HW_ATL_MPI_DAISY_CHAIN_STATUS		0x0704

#define AQ_HW_PCI_REG_CONTROL_6_ADR		0x1014

#define ITR_ISRLSW_ADR				0x2000	/* intr status */
#define ITR_ISCRLSW_ADR				0x2050	/* intr status clear */
#define ITR_IMSRLSW_ADR				0x2060	/* intr set */
#define ITR_IMCRLSW_ADR				0x2070	/* intr clear */

#define AQ_INTR_CTRL				0x2300
//#define ITR_REG_RES_DSBL_ADR			AQ_INTR_CTRL
//#define ITR_RES_ADR				AQ_INTR_CTRL

#define MIF_POWER_GATING_ENABLE_CONTROL_ADR	0x32a8

#define MPI_TX_REG_RES_DIS_ADR			0x4000
#define HW_ATL_MAC_PHY_CONTROL			0x4000
#define  HW_ATL_MAC_PHY_MPI_RESET_BIT			0x1d
#define HW_ATL_RX_REG_RES_DSBL_ADR		0x5000

#define RPFL2UC_DAFLSW_ADR(idx)			(0x5110 + (idx) * 0x8)
#define RPFL2UC_DAFMSW_ADR(idx)			(0x5114 + (idx) * 0x8)
#define RPFL2UC_ENF_ADR(idx)			RPFL2UC_DAFMSW_ADR(idx)
#define  RPFL2UC_EN				__BIT(31)
#define  AQ_HW_MAC				0	/* own address */
#define  AQ_HW_MAC_MIN			1
#define  AQ_HW_MAC_MAX			33

#define RPB_RXBXOFF_EN_ADR			0x5714
#define  RPB_RXBXOFF_EN				__BIT(31)

#define RX_DMA_STAT_COUNTER7_ADR		0x6818
#define STATS_RX_LO_COALESCED_PKT_COUNT0_ADDR	0x6820

#define HW_ATL_TX_REG_RES_DSBL_ADR		0x7000

#define AQ_HW_TX_DMA_TOTAL_REQ_LIMIT_ADR	0x7b20
#define TDM_INT_DESC_WRB_EN_ADR			0x7b40
#define  TDM_INT_DESC_WRB_EN			__BIT(1)


#define THM_LSO_TCP_FLAG_FIRST_ADR		0x7820
#define THM_LSO_TCP_FLAG_MID_ADR		THM_LSO_TCP_FLAG_FIRST_ADR
#define  THM_LSO_TCP_FLAG_FIRST_MSK		__BITS(11,0)
#define  THM_LSO_TCP_FLAG_MID_MSK		__BITS(27,16)
#define THM_LSO_TCP_FLAG_LAST_ADR		0x7824
#define  THM_LSO_TCP_FLAG_LAST_MSK		__BITS(11,0)

#define TPB_TX_BUF_ADR				0x7900
#define  TPB_TX_BUF_EN				__BIT(0)
#define  TPB_TX_BUF_SCP_INS_EN			__BIT(2)
#define  TPB_TX_BUF_TC_MODE_EN			__BIT(8)

#define TX_DMA_DESC_BASE_ADDRLSW_ADR(n)		(0x7c00 + (n) * 0x40)
#define TX_DMA_DESC_BASE_ADDRMSW_ADR(n)		(0x7c04 + (n) * 0x40)
#define TX_DMA_DESC_LEN_ADR(n)			(0x7c08 + (n) * 0x40)
#define  TX_DMA_DESC_LEN_MSK			__BITS(12, 0)	/* must be 2^3 */
#define  TX_DMA_DESC_LEN_ENABLE			__BIT(31)
#define TX_DMA_DESC_HEAD_PTR_ADR(n)		(0x7c0c + (n) * 0x40)	/* index of desc */
#define  TX_DMA_DESC_DHD_MASK			__BITS(12,0)

#define TX_DMA_DESC_TAIL_PTR_ADR(n)		(0x7c10 + (n) * 0x40)	/* index of desc */

#define TX_DMA_DESC_WRWB_THRESH_ADR(n)		(0x7c18 + (n) * 0x40)
#define  TX_DMA_DESC_WRWB_THRESH		__BITS(14,8)

#define TDM_DCADCPUID_ADR(n)			(0x8400 + (n) * 0x4)
#define  TDM_DCADCPUID_MSK			__BITS(7,0)
#define  TDM_DCADCPUID_EN			__BIT(31)

#define TDM_DCA_ADR				0x8480
#define  TDM_DCA_EN				__BIT(31)
#define  TDM_DCA_MODE				__BITS(3,0)



#define AQ_LINK_UNKNOWN	0x00000000
#define AQ_LINK_100M	0x00000001
#define AQ_LINK_1G	0x00000002
#define AQ_LINK_2G5	0x00000004
#define AQ_LINK_5G	0x00000008
#define AQ_LINK_10G	0x00000010
#define AQ_LINK_ALL	(AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | \
			 AQ_LINK_5G | AQ_LINK_10G )

#define FW2X_CTRL_10BASET_HD		__BIT(0)
#define FW2X_CTRL_10BASET_FD		__BIT(1)
#define FW2X_CTRL_100BASETX_HD		__BIT(2)
#define FW2X_CTRL_100BASET4_HD		__BIT(3)
#define FW2X_CTRL_100BASET2_HD		__BIT(4)
#define FW2X_CTRL_100BASETX_FD		__BIT(5)
#define FW2X_CTRL_100BASET2_FD		__BIT(6)
#define FW2X_CTRL_1000BASET_HD		__BIT(7)
#define FW2X_CTRL_1000BASET_FD		__BIT(8)
#define FW2X_CTRL_2P5GBASET_FD		__BIT(9)
#define FW2X_CTRL_5GBASET_FD		__BIT(10)
#define FW2X_CTRL_10GBASET_FD		__BIT(11)
#define FW2X_CTRL_RESERVED1		__BIT(32)
#define FW2X_CTRL_10BASET_EEE		__BIT(33)
#define FW2X_CTRL_RESERVED2		__BIT(34)
#define FW2X_CTRL_PAUSE			__BIT(35)
#define FW2X_CTRL_ASYMMETRIC_PAUSE	__BIT(36)
#define FW2X_CTRL_100BASETX_EEE		__BIT(37)
#define FW2X_CTRL_RESERVED3		__BIT(38)
#define FW2X_CTRL_RESERVED4		__BIT(39)
#define FW2X_CTRL_1000BASET_FD_EEE	__BIT(40)
#define FW2X_CTRL_2P5GBASET_FD_EEE	__BIT(41)
#define FW2X_CTRL_5GBASET_FD_EEE	__BIT(42)
#define FW2X_CTRL_10GBASET_FD_EEE	__BIT(43)
#define FW2X_CTRL_RESERVED5		__BIT(44)
#define FW2X_CTRL_RESERVED6		__BIT(45)
#define FW2X_CTRL_RESERVED7		__BIT(46)
#define FW2X_CTRL_RESERVED8		__BIT(47)
#define FW2X_CTRL_RESERVED9		__BIT(48)
#define FW2X_CTRL_CABLE_DIAG		__BIT(49)
#define FW2X_CTRL_TEMPERATURE		__BIT(50)
#define FW2X_CTRL_DOWNSHIFT		__BIT(51)
#define FW2X_CTRL_PTP_AVB_EN		__BIT(52)
#define FW2X_CTRL_MEDIA_DETECT		__BIT(53)
#define FW2X_CTRL_LINK_DROP		__BIT(54)
#define FW2X_CTRL_SLEEP_PROXY		__BIT(55)
#define FW2X_CTRL_WOL			__BIT(56)
#define FW2X_CTRL_MAC_STOP		__BIT(57)
#define FW2X_CTRL_EXT_LOOPBACK		__BIT(58)
#define FW2X_CTRL_INT_LOOPBACK		__BIT(59)
#define FW2X_CTRL_EFUSE_AGENT		__BIT(60)
#define FW2X_CTRL_WOL_TIMER		__BIT(61)
#define FW2X_CTRL_STATISTICS		__BIT(62)
#define FW2X_CTRL_TRANSACTION_ID	__BIT(63)

#define FW2X_CTRL_RATE_100M		FW2X_CTRL_100BASETX_FD
#define FW2X_CTRL_RATE_1G		FW2X_CTRL_1000BASET_FD
#define FW2X_CTRL_RATE_2G5		FW2X_CTRL_2P5GBASET_FD
#define FW2X_CTRL_RATE_5G		FW2X_CTRL_5GBASET_FD
#define FW2X_CTRL_RATE_10G		FW2X_CTRL_10GBASET_FD
#define FW2X_CTRL_RATE_MASK		\
	(FW2X_CTRL_RATE_100M |		\
	 FW2X_CTRL_RATE_1G |		\
	 FW2X_CTRL_RATE_2G5 |		\
	 FW2X_CTRL_RATE_5G |		\
	 FW2X_CTRL_RATE_10G)
#define FW2X_CTRL_EEE_MASK		\
	(FW2X_CTRL_10BASET_EEE |	\
	 FW2X_CTRL_100BASETX_EEE |	\
	 FW2X_CTRL_1000BASET_FD_EEE |	\
	 FW2X_CTRL_2P5GBASET_FD_EEE |	\
	 FW2X_CTRL_5GBASET_FD_EEE |	\
	 FW2X_CTRL_10GBASET_FD_EEE)

typedef enum aq_fw_bootloader_mode {
	BOOT_MODE_UNKNOWN = 0,
	BOOT_MODE_FLB,
	BOOT_MODE_RBL_FLASH,
	BOOT_MODE_RBL_HOST_BOOTLOAD
} aq_fw_bootloader_mode_t;

#define AQ_WRITE_REG(sc, reg, val)				\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, (reg), (val))

#define AQ_READ_REG(sc, reg)					\
	bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, (reg))

#define AQ_READ64_REG(sc, reg)					\
	((uint64_t)AQ_READ_REG(sc, reg) |			\
	(((uint64_t)AQ_READ_REG(sc, (reg) + 4)) << 32))

#define AQ_WRITE64_REG(sc, reg, val)				\
	do {							\
		AQ_WRITE_REG(sc, reg, (uint32_t)val);		\
		AQ_WRITE_REG(sc, reg + 4, (uint32_t)(val >> 32)); \
	} while (/* CONSTCOND */0)

#define AQ_OR_REG(sc, reg, val)					\
	do {							\
		uint32_t _v;					\
		_v = AQ_READ_REG((sc), (reg));			\
		AQ_WRITE_REG((sc), (reg), _v | (val));		\
	} while (/* CONSTCOND */ 0)

#define AQ_AND_REG(sc, reg, val)				\
	do {							\
		uint32_t _v;					\
		_v = AQ_READ_REG((sc), (reg));			\
		AQ_WRITE_REG((sc), (reg), _v & (val));		\
	} while (/* CONSTCOND */ 0)

#define AQ_READ_REG_BIT(sc, reg, mask)				\
	__SHIFTOUT(AQ_READ_REG(sc, reg), mask)

#define AQ_WRITE_REG_BIT(sc, reg, mask, val)			\
	do {							\
		uint32_t _v;					\
		_v = AQ_READ_REG((sc), (reg));			\
		_v &= ~(mask);					\
		_v |= __SHIFTIN((val), (mask));			\
		AQ_WRITE_REG((sc), (reg), _v);			\
	} while (/* CONSTCOND */ 0)

#define WAIT_FOR(_EXPR_, _US_, _N_, _ERRP_)			\
	do {							\
		unsigned int _n;				\
		for (_n = _N_; (!(_EXPR_)) && _n != 0; --_n) {	\
			delay((_US_));				\
		}						\
		if ((_ERRP_ != NULL)) {				\
			if (_n == 0)				\
				*(_ERRP_) = ETIMEDOUT;		\
			else					\
				*(_ERRP_) = 0;			\
		}						\
	} while (/* CONSTCOND */ 0)

#define msec_delay(x)	DELAY(1000 * (x))

typedef struct aq_mailbox_header {
	uint32_t version;
	uint32_t transaction_id;
	int32_t error;
} __packed aq_mailbox_header_t;

typedef struct aq_hw_stats_s {
	uint32_t uprc;
	uint32_t mprc;
	uint32_t bprc;
	uint32_t erpt;
	uint32_t uptc;
	uint32_t mptc;
	uint32_t bptc;
	uint32_t erpr;
	uint32_t mbtc;
	uint32_t bbtc;
	uint32_t mbrc;
	uint32_t bbrc;
	uint32_t ubrc;
	uint32_t ubtc;
	uint32_t ptc;
	uint32_t prc;
	uint32_t dpc;	/* not exists in fw2x_msm_statistics */
	uint32_t cprc;	/* not exists in fw2x_msm_statistics */
} __packed aq_hw_stats_s_t;

typedef struct fw1x_mailbox {
	aq_mailbox_header_t header;
	aq_hw_stats_s_t msm;
} __packed fw1x_mailbox_t;

typedef struct fw2x_msm_statistics {
	uint32_t uprc;
	uint32_t mprc;
	uint32_t bprc;
	uint32_t erpt;
	uint32_t uptc;
	uint32_t mptc;
	uint32_t bptc;
	uint32_t erpr;
	uint32_t mbtc;
	uint32_t bbtc;
	uint32_t mbrc;
	uint32_t bbrc;
	uint32_t ubrc;
	uint32_t ubtc;
	uint32_t ptc;
	uint32_t prc;
} __packed fw2x_msm_statistics_t;

typedef struct fw2x_phy_cable_diag_data {
	uint32_t lane_data[4];
} __packed fw2x_phy_cable_diag_data_t;

typedef struct fw2x_capabilities {
	uint32_t caps_lo;
	uint32_t caps_hi;
} __packed fw2x_capabilities_t;

typedef struct fw2x_mailbox {		/* struct fwHostInterface */
	aq_mailbox_header_t header;
	fw2x_msm_statistics_t msm;	/* msmStatistics_t msm; */
	uint16_t phy_h_bit;
	uint16_t phy_fault_code;
	int16_t phy_temperature;
	uint8_t cable_len;
	uint8_t reserved1;
	fw2x_phy_cable_diag_data_t diag_data;
	uint32_t reserved[8];

	fw2x_capabilities_t caps;

	/* ... */
} __packed fw2x_mailbox_t;

typedef enum aq_fw_link_speed {
	AQ_FW_NONE	= 0,
	AQ_FW_100M	= (1 << 0),
	AQ_FW_1G	= (1 << 1),
	AQ_FW_2G5	= (1 << 2),
	AQ_FW_5G	= (1 << 3),
	AQ_FW_10G	= (1 << 4)
} aq_fw_link_speed_t;

#define AQ_FW_SPEED_AUTO	\
	(AQ_FW_100M | AQ_FW_1G | AQ_FW_2G5 | AQ_FW_5G | AQ_FW_10G)

typedef enum aq_fw_link_fc {
	AQ_FW_FC_NONE  = 0,
	AQ_FW_FC_ENABLE_RX = __BIT(0),
	AQ_FW_FC_ENABLE_TX = __BIT(1),
	AQ_FW_FC_ENABLE_ALL = AQ_FW_FC_ENABLE_RX | AQ_FW_FC_ENABLE_TX
} aq_fw_link_fc_t;

typedef enum aq_hw_fw_mpi_state_e {
	MPI_DEINIT	= 0,
	MPI_RESET	= 1,
	MPI_INIT	= 2,
	MPI_POWER	= 4
} aq_hw_fw_mpi_state_e_t;

enum aq_media_type {
	AQ_MEDIA_TYPE_UNKNOWN = 0,
	AQ_MEDIA_TYPE_FIBRE,
	AQ_MEDIA_TYPE_TP
};

struct aq_rx_desc_read {
	uint64_t buf_addr;
	uint64_t hdr_addr;
} __packed;

struct aq_rx_desc_wb {
	uint32_t type;
	uint32_t rss_hash;
	uint16_t status;
	uint16_t pkt_len;
	uint16_t next_desc_ptr;
	uint16_t vlan;
} __packed;

typedef union aq_rx_desc {
	struct aq_rx_desc_read read;
	struct aq_rx_desc_wb wb;
} __packed aq_rx_desc_t;

typedef struct aq_tx_desc {
	uint64_t buf_addr;
	uint32_t ctl;
#define AQ_TXDESC_CTL_TYPE_MASK		0x00000003
#define AQ_TXDESC_CTL_TYPE_TXD		0x00000001
#define AQ_TXDESC_CTL_TYPE_TXC		0x00000002
#define AQ_TXDESC_CTL_BLEN		__BITS(19,4)
#define AQ_TXDESC_CTL_DD		__BIT(20)
#define AQ_TXDESC_CTL_EOP		__BIT(21)
#define AQ_TXDESC_CTL_CMD_VLAN		__BIT(22)
#define AQ_TXDESC_CTL_CMD_FCS		__BIT(23)
#define AQ_TXDESC_CTL_CMD_IPCSO 	__BIT(24)
#define AQ_TXDESC_CTL_CMD_TUCSO 	__BIT(25)
#define AQ_TXDESC_CTL_CMD_LSO		__BIT(26)
#define AQ_TXDESC_CTL_CMD_WB		__BIT(27)
#define AQ_TXDESC_CTL_CMD_VXLAN 	__BIT(28)
#define AQ_TXDESC_CTL_CMD_IPV6		__BIT(21) /* vs FSC */
#define AQ_TXDESC_CTL_CMD_TCP		__BIT(22) /* vs VLAN */
	uint32_t ctl2;
#define AQ_TXDESC_CTL2_LEN		__BITS(31,14)
#define AQ_TXDESC_CTL2_CTX_EN		__BIT(13)
#define AQ_TXDESC_CTL2_CTX_IDX		__BIT(12)
} __packed aq_tx_desc_t;

#define AQ_RINGS_MAX	32
#define AQ_RXD_MIN	32
#define AQ_TXD_MIN	32
#define AQ_RXD_MAX	4096	/* in fact up to 8184 */
#define AQ_TXD_MAX	4096	/* in fact up to 8184 */

//#define AQ_TXRING_NUM	16	/* <= AQ_RINGS_MAX */
//#define AQ_RXRING_NUM	16	/* <= AQ_RINGS_MAX */
#define AQ_TXRING_NUM	1	/* <= AQ_RINGS_MAX */
#define AQ_RXRING_NUM	1	/* <= AQ_RINGS_MAX */
#define AQ_TXD_NUM	32	/* per ring */
#define AQ_RXD_NUM	32	/* per ring */

struct aq_txring {
	struct aq_softc *ring_sc;
	int ring_index;

	aq_tx_desc_t *ring_txdesc;	/* aq_tx_desc_t[AQ_TXD_NUM] */
	bus_dmamap_t ring_txdesc_dmamap;
	bus_dma_segment_t ring_txdesc_seg[1];
	bus_size_t ring_txdesc_size;

	struct {
		struct mbuf *m;
		bus_dmamap_t dmamap;
	} ring_mbufs[AQ_TXD_NUM];
	int ring_prodidx;
	int ring_considx;
	int ring_nfree;
};

struct aq_rxring {
	struct aq_softc *ring_sc;
	int ring_index;

	aq_rx_desc_t *ring_rxdesc;	/* aq_rx_desc_t[AQ_RXD_NUM] */
	bus_dmamap_t ring_rxdesc_dmamap;
	bus_dma_segment_t ring_rxdesc_seg[1];
	bus_size_t ring_rxdesc_size;
	struct {
		struct mbuf *m;
		bus_dmamap_t dmamap;
	} ring_mbufs[AQ_RXD_NUM];
};

struct aq_softc;
struct aq_firmware_ops {
	int (*reset)(struct aq_softc *);
	int (*set_mode)(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
	    aq_fw_link_speed_t);
	int (*get_mode)(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
	    aq_fw_link_speed_t *, aq_fw_link_fc_t *);
	int (*get_stats)(struct aq_softc *, aq_hw_stats_s_t *);
	int (*led_control)(struct aq_softc *, uint32_t);
};

struct aq_hw_fc_info {
	bool fc_rx;
	bool fc_tx;
};

struct aq_softc {
	device_t sc_dev;

	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	bus_size_t sc_iosize;
	bus_dma_tag_t sc_dmat;;

	void *sc_intrhand;

	struct aq_txring sc_txring[AQ_TXRING_NUM];
	struct aq_rxring sc_rxring[AQ_RXRING_NUM];

	pci_chipset_tag_t sc_pc;
	pcitag_t sc_pcitag;
	uint16_t sc_product;
	uint16_t sc_revision;

	kmutex_t sc_mutex;

	struct aq_firmware_ops *sc_fw_ops;
	uint64_t sc_fw_caps;
	enum aq_media_type sc_media_type;
	uint32_t sc_available_rates;
	uint32_t sc_link_rate;		/* specified link rate (AQ_LINK_*) */
	uint32_t sc_link_speed;		/* actual link speed */
	uint32_t sc_fw_version;
#define FW_VERSION_MAJOR(sc)	(((sc)->sc_fw_version >> 24) & 0xff)
#define FW_VERSION_MINOR(sc)	(((sc)->sc_fw_version >> 16) & 0xff)
#define FW_VERSION_BUILD(sc)	((sc)->sc_fw_version & 0xffff)
	uint32_t sc_features;
#define FEATURES_MIPS		0x00000001
#define FEATURES_TPO2		0x00000002
#define FEATURES_RPF2		0x00000004
#define FEATURES_MPI_AQ		0x00000008
#define FEATURES_REV_A0		0x10000000
#define FEATURES_REV_B0		0x20000000
#define FEATURES_REV_B1		0x40000000
	uint32_t sc_mbox_addr;

	bool sc_rbl_enabled;
	bool sc_fast_start_enabled;	/* XXX: always zero */
	bool sc_flash_present;
	int sc_media_active;
	struct aq_hw_fc_info sc_fc;

	callout_t sc_tick_ch;
	struct ethercom sc_ethercom;
	struct ifmedia sc_media;
	struct ether_addr sc_enaddr;

	aq_hw_stats_s_t sc_statistics[2];
	int sc_statistics_idx;
	bool sc_statistics_enable;

	uint64_t sc_statistics_uprc;
	uint64_t sc_statistics_mprc;
	uint64_t sc_statistics_bprc;
	uint64_t sc_statistics_erpt;
	uint64_t sc_statistics_uptc;
	uint64_t sc_statistics_mptc;
	uint64_t sc_statistics_bptc;
	uint64_t sc_statistics_erpr;
	uint64_t sc_statistics_mbtc;
	uint64_t sc_statistics_bbtc;
	uint64_t sc_statistics_mbrc;
	uint64_t sc_statistics_bbrc;
	uint64_t sc_statistics_ubrc;
	uint64_t sc_statistics_ubtc;
	uint64_t sc_statistics_ptc;
	uint64_t sc_statistics_prc;
	uint64_t sc_statistics_dpc;
	uint64_t sc_statistics_cprc;
};

static int aq_match(device_t, cfdata_t, void *);
static void aq_attach(device_t, device_t, void *);
static int aq_detach(device_t, int);

static int aq_ifmedia_change(struct ifnet * const);
static void aq_ifmedia_status(struct ifnet * const, struct ifmediareq *);
static int aq_init(struct ifnet *);
static void aq_start(struct ifnet *);
static void aq_stop(struct ifnet *, int);
static void aq_watchdog(struct ifnet *);
static int aq_ioctl(struct ifnet *, unsigned long, void *);

static int aq_hw_set_link_speed(struct aq_softc *, uint32_t);

static int aq_txring_alloc(struct aq_softc *, struct aq_txring *);
static void aq_txring_free(struct aq_softc *, struct aq_txring *);
static int aq_rxring_alloc(struct aq_softc *, struct aq_rxring *);
static void aq_rxring_free(struct aq_softc *, struct aq_rxring *);

static int fw1x_reset(struct aq_softc *);
static int fw1x_set_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
    aq_fw_link_speed_t);
static int fw1x_get_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
    aq_fw_link_speed_t *, aq_fw_link_fc_t *);
static int fw1x_get_stats(struct aq_softc *, aq_hw_stats_s_t *);
static int fw2x_reset(struct aq_softc *);
static int fw2x_set_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
    aq_fw_link_speed_t);
static int fw2x_get_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
    aq_fw_link_speed_t *, aq_fw_link_fc_t *);
static int fw2x_get_stats(struct aq_softc *, aq_hw_stats_s_t *);
static int fw2x_led_control(struct aq_softc *, uint32_t);

static struct aq_firmware_ops aq_fw1x_ops = {
	.reset = fw1x_reset,
	.set_mode = fw1x_set_mode,
	.get_mode = fw1x_get_mode,
	.get_stats = fw1x_get_stats,
	.led_control = NULL
};

static struct aq_firmware_ops aq_fw2x_ops = {
	.reset = fw2x_reset,
	.set_mode = fw2x_set_mode,
	.get_mode = fw2x_get_mode,
	.get_stats = fw2x_get_stats,
	.led_control = fw2x_led_control
};

CFATTACH_DECL3_NEW(aq, sizeof(struct aq_softc),
    aq_match, aq_attach, aq_detach, NULL, NULL, NULL, DVF_DETACH_SHUTDOWN);

static const struct aq_product {
	pci_vendor_id_t aq_vendor;
	pci_product_id_t aq_product;
	const char *aq_name;
	enum aq_media_type aq_media_type;
	uint32_t aq_available_rates;
} aq_products[] = {
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC107,
	  "Aquantia AQC107 10 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC108,
	  "Aquantia AQC108 5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~AQ_LINK_10G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC109,
	  "Aquantia AQC109 2.5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~(AQ_LINK_10G | AQ_LINK_5G)
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC111,
	  "Aquantia AQC111 5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~AQ_LINK_10G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC112,
	  "Aquantia AQC112 2.5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~(AQ_LINK_10G | AQ_LINK_5G)
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC107S,
	  "Aquantia AQC107S 10 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC108S,
	  "Aquantia AQC108S 5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~AQ_LINK_10G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC109S,
	  "Aquantia AQC109S 2.5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~(AQ_LINK_10G | AQ_LINK_5G)
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC111S,
	  "Aquantia AQC111S 5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~AQ_LINK_10G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC112S,
	  "Aquantia AQC112S 2.5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~(AQ_LINK_10G | AQ_LINK_5G)
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_D107,
	  "Aquantia D107 10 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_D108,
	  "Aquantia D108 5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~AQ_LINK_10G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_D109,
	  "Aquantia D109 2.5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL & ~(AQ_LINK_10G | AQ_LINK_5G)
	}
};

static int
aq_fw_downld_dwords(struct aq_softc *sc, uint32_t addr, uint32_t *p,
    uint32_t cnt)
{
	uint32_t v;
	int error = 0;

	WAIT_FOR(AQ_READ_REG(sc, HW_ATL_GLB_CPU_SEM_ADR(HW_ATL_FW_SM_RAM)) == 1,
	    1, 10000, &error);
	if (error != 0) {
		AQ_WRITE_REG(sc, HW_ATL_GLB_CPU_SEM_ADR(HW_ATL_FW_SM_RAM), 1);
		v = AQ_READ_REG(sc, HW_ATL_GLB_CPU_SEM_ADR(HW_ATL_FW_SM_RAM));
		if (v == 0) {
			aprint_debug_dev(sc->sc_dev,
			    "%s:%d: timeout\n", __func__, __LINE__);
			return ETIMEDOUT;
		}
	}

	AQ_WRITE_REG(sc, HW_ATL_MIF_ADDR, addr);

	error = 0;
	for (; cnt > 0 && error == 0; cnt--) {
		/* execute mailbox interface */
		AQ_OR_REG(sc, HW_ATL_MIF_CMD, HW_ATL_MIF_CMD_EXECUTE);
		if (sc->sc_features & FEATURES_REV_B1) {
			WAIT_FOR(
			    AQ_READ_REG(sc, HW_ATL_MIF_ADDR) != addr,
			    1, 1000, &error);
		} else {
			WAIT_FOR(
			    (AQ_READ_REG(sc, HW_ATL_MIF_CMD) &
			    HW_ATL_MIF_CMD_BUSY) == 0,
			    1, 1000, &error);
		}
		*p++ = AQ_READ_REG(sc, HW_ATL_MIF_VAL);
		addr += sizeof(uint32_t);
	}
	AQ_WRITE_REG(sc, HW_ATL_GLB_CPU_SEM_ADR(HW_ATL_FW_SM_RAM), 1);

	if (error != 0)
		aprint_debug_dev(sc->sc_dev,
		    "%s:%d: timeout\n", __func__, __LINE__);

	return error;
}

static int
wait_init_mac_firmware(struct aq_softc *sc)
{
	int i, error = EBUSY;
#define MAC_FW_START_TIMEOUT_MS	10000
	for (i = 0; i < MAC_FW_START_TIMEOUT_MS; i++) {
		sc->sc_fw_version = AQ_READ_REG(sc, HW_ATL_MPI_FW_VERSION);
		if (sc->sc_fw_version != 0) {
			error = 0;
			break;
		}
		delay(1000);
	}
	return error;
}

static int
aq_hw_get_link_state(struct aq_softc *sc, uint32_t *link_speed, struct aq_hw_fc_info *fc_neg)
{
	int error = 0;

	aq_hw_fw_mpi_state_e_t mode;
	aq_fw_link_speed_t speed = AQ_FW_NONE;
	aq_fw_link_fc_t fc;

	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->get_mode != NULL) {
		error = sc->sc_fw_ops->get_mode(sc, &mode, &speed, &fc);
	} else {
		aprint_error_dev(sc->sc_dev, "get_mode() not supported by F/W\n");
		return ENOTSUP;
	}
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "get_mode() failed, error %d\n", error);
		return error;
	}
	*link_speed = 0;
	if (mode != MPI_INIT)
		return 0;

	switch (speed) {
	case AQ_FW_10G:
		*link_speed = 10000;
		break;
	case AQ_FW_5G:
		*link_speed = 5000;
		break;
	case AQ_FW_2G5:
		*link_speed = 2500;
		break;
	case AQ_FW_1G:
		*link_speed = 1000;
		break;
	case AQ_FW_100M:
		*link_speed = 100;
		break;
	default:
		*link_speed = 0;
		break;
	}

	fc_neg->fc_rx = !!(fc & AQ_FW_FC_ENABLE_RX);
	fc_neg->fc_tx = !!(fc & AQ_FW_FC_ENABLE_TX);

	return 0;
}

/* read my mac address */
static int
aq_get_mac_addr(struct aq_softc *sc)
{
	uint32_t mac_addr[2];
	uint32_t efuse_shadow_addr;
	int err;

	efuse_shadow_addr = 0;
	if (FW_VERSION_MAJOR(sc) >= 2)
		efuse_shadow_addr = AQ_READ_REG(sc, HW_ATL_FW2X_MPI_EFUSE_ADDR);
	else
		efuse_shadow_addr = AQ_READ_REG(sc, HW_ATL_FW1X_MPI_EFUSE_ADDR);

	if (efuse_shadow_addr == 0) {
		aprint_error_dev(sc->sc_dev, "cannot get efuse addr\n");
		return ENXIO;
	}

	memset(mac_addr, 0, sizeof(mac_addr));
	err = aq_fw_downld_dwords(sc, efuse_shadow_addr + (40 * 4),
	    mac_addr, __arraycount(mac_addr));
	if (err < 0)
		return err;

	if (mac_addr[0] == 0 && mac_addr[1] == 0) {
		aprint_error_dev(sc->sc_dev, "mac address not found\n");
		return ENXIO;
	}

	mac_addr[0] = bswap32(mac_addr[0]);
	mac_addr[1] = bswap32(mac_addr[1]);

	memcpy(sc->sc_enaddr.ether_addr_octet,
	    (uint8_t *)mac_addr, ETHER_ADDR_LEN);
	aprint_normal_dev(sc->sc_dev, "Etheraddr: %s\n",
	    ether_sprintf(sc->sc_enaddr.ether_addr_octet));

	return 0;
}

/* set multicast filter, or own address */
static int
aq_set_mac_addr(struct aq_softc *sc, int index, uint8_t *enaddr)
{
	uint32_t h, l;

	if (index > AQ_HW_MAC_MAX)
		return EINVAL;

	h = (enaddr[0] << 8) |
	    (enaddr[1]);
	l = (enaddr[2] << 24) |
	    (enaddr[3] << 16) |
	    (enaddr[4] << 8) |
	    (enaddr[5]);

	/* disable, set, and enable */
	AQ_AND_REG(sc, RPFL2UC_ENF_ADR(index), ~RPFL2UC_EN);
	AQ_WRITE_REG(sc, RPFL2UC_DAFLSW_ADR(index), l);
	AQ_WRITE_REG(sc, RPFL2UC_DAFMSW_ADR(index), h);
	AQ_OR_REG(sc, RPFL2UC_ENF_ADR(index), RPFL2UC_EN);

	return 0;
}


static void
aq_mediastatus_update(struct aq_softc *sc, uint32_t link_speed, const struct aq_hw_fc_info *fc_neg)
{
	sc->sc_media_active = 0;

	if (fc_neg->fc_rx)
		sc->sc_media_active |= IFM_ETH_RXPAUSE;
	if (fc_neg->fc_tx)
		sc->sc_media_active |= IFM_ETH_TXPAUSE;

	switch(link_speed) {
	case 100:
		sc->sc_media_active |= IFM_100_TX | IFM_FDX;
		break;
	case 1000:
		sc->sc_media_active |= IFM_1000_T | IFM_FDX;
		break;
	case 2500:
		sc->sc_media_active |= IFM_2500_T | IFM_FDX;
		break;
	case 5000:
		sc->sc_media_active |= IFM_5000_T | IFM_FDX;
		break;
	case 10000:
		sc->sc_media_active |= IFM_10G_T | IFM_FDX;
		break;
	case 0:
	default:
		sc->sc_media_active |= IFM_NONE;
		break;
	}
	if (sc->sc_link_rate == AQ_FW_SPEED_AUTO)
		sc->sc_media_active |= IFM_AUTO;

}

static void
aq_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct aq_softc *sc = ifp->if_softc;

	ifmr->ifm_active = IFM_ETHER;
	ifmr->ifm_status = IFM_AVALID;

	if (sc->sc_link_speed != 0)
		ifmr->ifm_status |= IFM_ACTIVE;

	ifmr->ifm_active |= sc->sc_media_active;
}

static int
aq_mediachange(struct ifnet *ifp)
{
	struct aq_softc *sc = ifp->if_softc;

	if (IFM_TYPE(sc->sc_media.ifm_media) != IFM_ETHER)
		return EINVAL;

	switch (IFM_SUBTYPE(sc->sc_media.ifm_media)) {
	case IFM_AUTO:
		sc->sc_link_rate = AQ_FW_SPEED_AUTO;
		break;
	case IFM_NONE:
		sc->sc_link_rate = 0;
		break;
	case IFM_100_TX:
		sc->sc_link_rate = AQ_FW_100M;
		break;
	case IFM_1000_T:
		sc->sc_link_rate = AQ_FW_1G;
		break;
	case IFM_2500_T:
		sc->sc_link_rate = AQ_FW_2G5;
		break;
	case IFM_5000_T:
		sc->sc_link_rate = AQ_FW_5G;
		break;
	case IFM_10G_T:
		sc->sc_link_rate = AQ_FW_10G;
		break;
	default:
		aprint_error_dev(sc->sc_dev, "unknown media: 0x%X\n", IFM_SUBTYPE(sc->sc_media.ifm_media));
		return 0;
	}

	if (sc->sc_media.ifm_media & IFM_FLOW)
		sc->sc_fc.fc_rx = sc->sc_fc.fc_tx = true;
	else
		sc->sc_fc.fc_rx = sc->sc_fc.fc_tx = false;

	/* re-initialize hardware with new parameters */
	aq_hw_set_link_speed(sc, sc->sc_link_rate);

	return 0;
}

static void
aq_add_media_types(struct aq_softc *sc, int speed)
{
	ifmedia_add(&sc->sc_media, IFM_ETHER | speed, 0, NULL);
	ifmedia_add(&sc->sc_media, IFM_ETHER | speed | IFM_FLOW, 0, NULL);
}

static void
aq_initmedia(struct aq_softc *sc)
{
	ifmedia_add(&sc->sc_media, IFM_ETHER | IFM_NONE, 0, NULL);

	if (sc->sc_available_rates & AQ_LINK_100M)
		aq_add_media_types(sc, IFM_100_TX);
	if (sc->sc_available_rates & AQ_LINK_1G)
		aq_add_media_types(sc, IFM_1000_T);
	if (sc->sc_available_rates & AQ_LINK_2G5)
		aq_add_media_types(sc, IFM_2500_T);
	if (sc->sc_available_rates & AQ_LINK_5G)
		aq_add_media_types(sc, IFM_5000_T);
	if (sc->sc_available_rates & AQ_LINK_10G)
		aq_add_media_types(sc, IFM_10G_T);

	aq_add_media_types(sc, IFM_AUTO);

	ifmedia_set(&sc->sc_media, IFM_ETHER | IFM_AUTO | IFM_FLOW);
}

static void
global_software_reset(struct aq_softc *sc)
{
	uint32_t v;

	AQ_AND_REG(sc, HW_ATL_RX_REG_RES_DSBL_ADR, ~__BIT(29));	/* RX disable */
	AQ_AND_REG(sc, HW_ATL_TX_REG_RES_DSBL_ADR, ~__BIT(29));	/* TX disable */

	AQ_AND_REG(sc, MPI_TX_REG_RES_DIS_ADR, ~__BIT(29));

	v = AQ_READ_REG(sc, GLB_STANDARD_CTL1_ADR);
	v &= ~GLB_REG_RES_DIS_MSK;
	v |= GLB_SOFT_RES_MSK;
	AQ_WRITE_REG(sc, GLB_STANDARD_CTL1_ADR, v);
}

static int
mac_soft_reset_rbl(struct aq_softc *sc, aq_fw_bootloader_mode_t *mode)
{
	int timo;

	aprint_debug_dev(sc->sc_dev, "RBL> MAC reset STARTED!\n");

	AQ_WRITE_REG(sc, HW_ATL_MCP_UP_FORCE_INTERRUPT_ADR, 0x40e1);
	AQ_WRITE_REG(sc, HW_ATL_GLB_CPU_SEM_ADR(0), 1);
	AQ_WRITE_REG(sc, MIF_POWER_GATING_ENABLE_CONTROL_ADR, 0);

	/* MAC FW will reload PHY FW if 1E.1000.3 was cleaned - #undone */
	AQ_WRITE_REG(sc, HW_ATL_MPI_BOOT_EXIT_CODE, RBL_STATUS_DEAD);

	global_software_reset(sc);

	AQ_WRITE_REG(sc, GLB_CTL2_ADR, 0x40e0);

	/* Wait for RBL to finish boot process. */
#define RBL_TIMEOUT_MS	10000
	uint16_t rbl_status;
	for (timo = RBL_TIMEOUT_MS; timo > 0; timo--) {
		rbl_status =
		    AQ_READ_REG(sc, HW_ATL_MPI_BOOT_EXIT_CODE) & 0xffff;
		if (rbl_status != 0 && rbl_status != RBL_STATUS_DEAD)
			break;
		msec_delay(1);
	}
	if (timo <= 0) {
		aprint_error_dev(sc->sc_dev,
		    "RBL> RBL restart failed: timeout\n");
		return EBUSY;
	}
	switch (rbl_status) {
	case RBL_STATUS_SUCCESS:
		if (mode != NULL)
			*mode = BOOT_MODE_RBL_FLASH;
		aprint_debug_dev(sc->sc_dev,
		    "RBL> reset complete! [Flash]\n");
		break;
	case RBL_STATUS_HOST_BOOT:
		if (mode != NULL)
			*mode = BOOT_MODE_RBL_HOST_BOOTLOAD;
		aprint_debug_dev(sc->sc_dev,
		    "RBL> reset complete! [Host Bootload]\n");
		break;
	case RBL_STATUS_FAILURE:
	default:
		aprint_error_dev(sc->sc_dev,
		    "unknown RBL status 0x%x\n", rbl_status);
		return EBUSY;
	}

	return 0;
}

static int
mac_soft_reset_flb(struct aq_softc *sc)
{
	uint32_t v;
	int timo;

	AQ_WRITE_REG(sc, GLB_CTL2_ADR, 0x40e1);
	/*
	 * Let Felicity hardware to complete SMBUS transaction before
	 * Global software reset.
	 */
	msec_delay(50);

	/*
	 * If SPI burst transaction was interrupted(before running the script),
	 * global software reset may not clear SPI interface.
	 * Clean it up manually before global reset.
	 */
	AQ_WRITE_REG(sc, GLB_NVR_PROVISIONING2_ADR, 0x00a0);
	AQ_WRITE_REG(sc, GLB_NVR_INTERFACE1_ADR, 0x009f);
	AQ_WRITE_REG(sc, GLB_NVR_INTERFACE1_ADR, 0x809f);
	msec_delay(50);

	v = AQ_READ_REG(sc, GLB_STANDARD_CTL1_ADR);
	v &= ~GLB_REG_RES_DIS_MSK;
	v |= GLB_SOFT_RES_MSK;
	AQ_WRITE_REG(sc, GLB_STANDARD_CTL1_ADR, v);

	/* Kickstart. */
	AQ_WRITE_REG(sc, GLB_CTL2_ADR, 0x80e0);
	AQ_WRITE_REG(sc, MIF_POWER_GATING_ENABLE_CONTROL_ADR, 0);
	if (!sc->sc_fast_start_enabled)
		AQ_WRITE_REG(sc, GLB_GENERAL_PROVISIONING9_ADR, 1);

	/*
	 * For the case SPI burst transaction was interrupted (by MCP reset
	 * above), wait until it is completed by hardware.
	 */
	msec_delay(50);

	/* MAC Kickstart */
	if (!sc->sc_fast_start_enabled) {
		AQ_WRITE_REG(sc, GLB_CTL2_ADR, 0x180e0);

		uint32_t flb_status;
		for (timo = 0; timo < 1000; timo++) {
			flb_status = AQ_READ_REG(sc,
			    HW_ATL_MPI_DAISY_CHAIN_STATUS) & 0x10;
			if (flb_status != 0)
				break;
			msec_delay(1);
		}
		if (flb_status == 0) {
			aprint_error_dev(sc->sc_dev,
			    "FLB> MAC kickstart failed: timed out\n");
			return ETIMEDOUT;
		}
		aprint_debug_dev(sc->sc_dev,
		    "FLB> MAC kickstart done, %d ms\n", timo);
		/* FW reset */
		AQ_WRITE_REG(sc, GLB_CTL2_ADR, 0x80e0);
		/*
		 * Let Felicity hardware complete SMBUS transaction before
		 * Global software reset.
		 */
		msec_delay(50);
	}
	AQ_WRITE_REG(sc, HW_ATL_GLB_CPU_SEM_ADR(0), 1);

	/* PHY Kickstart: #undone */
	global_software_reset(sc);

	for (timo = 0; timo < 1000; timo++) {
		if (AQ_READ_REG(sc, GLB_FW_IMAGE_ID1_ADR) != 0)
			break;
		msec_delay(10);
	}
	if (timo >= 1000) {
		aprint_error_dev(sc->sc_dev, "FLB> Global Soft Reset failed\n");
		return ETIMEDOUT;
	}
	aprint_debug_dev(sc->sc_dev, "FLB> F/W restart: %d ms\n", timo * 10);
	return 0;

}

static int
mac_soft_reset(struct aq_softc *sc, aq_fw_bootloader_mode_t *mode)
{
	if (sc->sc_rbl_enabled)
		return mac_soft_reset_rbl(sc, mode);

	if (mode != NULL)
		*mode = BOOT_MODE_FLB;
	return mac_soft_reset_flb(sc);
}

static int
aq_fw_reset(struct aq_softc *sc)
{
	uint32_t ver, v, bootExitCode;
	int i, error;

	ver = AQ_READ_REG(sc, HW_ATL_MPI_FW_VERSION);

	for (i = 1000; i > 0; i--) {
		v = AQ_READ_REG(sc, HW_ATL_MPI_DAISY_CHAIN_STATUS);
		bootExitCode = AQ_READ_REG(sc, HW_ATL_MPI_BOOT_EXIT_CODE);
		if (v != 0x06000000 || bootExitCode != 0)
			break;
	}
	if (i <= 0) {
		aprint_error_dev(sc->sc_dev,
		    "F/W reset failed. Neither RBL nor FLB started\n");
		return ETIMEDOUT;
	}
	sc->sc_rbl_enabled = (bootExitCode != 0);

	/*
	 * Having FW version 0 is an indicator that cold start
	 * is in progress. This means two things:
	 * 1) Driver have to wait for FW/HW to finish boot (500ms giveup)
	 * 2) Driver may skip reset sequence and save time.
	 */
	if (sc->sc_fast_start_enabled && (ver != 0)) {
		error = wait_init_mac_firmware(sc);
		/* Skip reset as it just completed */
		if (error == 0)
			return 0;
	}

	aq_fw_bootloader_mode_t mode = BOOT_MODE_UNKNOWN;
	error = mac_soft_reset(sc, &mode);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "MAC reset failed: %d\n", error);
		return error;
	}

	switch (mode) {
	case BOOT_MODE_FLB:
		aprint_debug_dev(sc->sc_dev,
		    "FLB> F/W successfully loaded from flash.\n");
		sc->sc_flash_present = true;
		return wait_init_mac_firmware(sc);
	case BOOT_MODE_RBL_FLASH:
		aprint_debug_dev(sc->sc_dev,
		    "RBL> F/W loaded from flash. Host Bootload disabled.\n");
		sc->sc_flash_present = true;
		return wait_init_mac_firmware(sc);
	case BOOT_MODE_UNKNOWN:
		aprint_error_dev(sc->sc_dev,
		    "F/W bootload error: unknown bootloader type\n");
		return ENOTSUP;
	case BOOT_MODE_RBL_HOST_BOOTLOAD:
#if 0 /* AQ_CFG_HOST_BOOT_DISABLE */
		aprint_error_dev(sc->sc_dev, "RBL> Host Bootload mode: "
		    "this driver does not support Host Boot\n");
		return ENOTSUP;
#else
		aprint_debug_dev(sc->sc_dev, "RBL> Host Bootload mode\n");
		break;
#endif
	}

	/*
	 * XXX: todo: Host Boot
	 */
	aprint_error_dev(sc->sc_dev,
	    "RBL> F/W Host Bootload not implemented\n");
	return ENOTSUP;
}

static int
aq_hw_reset(struct aq_softc *sc)
{
	int error;

	/* disable irq */
	AQ_AND_REG(sc, AQ_INTR_CTRL, ~__BIT(29));

	/* apply */
	AQ_OR_REG(sc, AQ_INTR_CTRL, __BIT(31));

	/* wait ack 10 times by 1ms */
	WAIT_FOR((AQ_READ_REG(sc, AQ_INTR_CTRL) & __BIT(31)) == 0,
	    1000, 10, &error);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "atlantic: IRQ reset failed: %d\n", error);
		return error;
	}

	error = 0;
	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->reset != NULL)
		error = sc->sc_fw_ops->reset(sc);

	return error;
}

static int
aq_hw_init_ucp(struct aq_softc *sc)
{
	int timo;

	if (FW_VERSION_MAJOR(sc) == 1) {
		if (AQ_READ_REG(sc, HW_ATL_UCP_0X370_REG) == 0) {
			uint32_t data;
			cprng_fast(&data, sizeof(data));
			data &= 0xfefefefe;
			data |= 0x02020202;
			AQ_WRITE_REG(sc, HW_ATL_UCP_0X370_REG, data);
		}
		AQ_WRITE_REG(sc, HW_ATL_GLB_CPU_SCRATCH_SCP_ADR(25), 0);
	}

	for (timo = 100; timo > 0; timo--) {
		/*
		 * XXX: linux uses (25), FreeBSD uses (24).
		 *I don't know which is correct...
		 */
		sc->sc_mbox_addr =
		    AQ_READ_REG(sc, HW_ATL_GLB_CPU_SCRATCH_SCP_ADR(25));
		if (sc->sc_mbox_addr != 0)
			break;
		delay(1000);
	}

#define AQ_FW_MIN_VERSION	0x01050006
#define AQ_FW_MIN_VERSION_STR	"1.5.6"
	if (sc->sc_fw_version < AQ_FW_MIN_VERSION) {
		aprint_error_dev(sc->sc_dev,
		    "atlantic: wrong FW version: expected:"
		    AQ_FW_MIN_VERSION_STR
		    " actual:%d.%d.%d\n",
		    FW_VERSION_MAJOR(sc),
		    FW_VERSION_MINOR(sc),
		    FW_VERSION_BUILD(sc));
		return ENOTSUP;
	}

	return 0;
}

static int
aq_hw_mpi_set(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t state, uint32_t speed)
{
	int error;

	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->set_mode != NULL) {
		error = sc->sc_fw_ops->set_mode(sc, state, speed);
	} else {
		aprint_error_dev(sc->sc_dev, "set_mode() not supported by F/W\n");
		error = ENOTSUP;
	}
	return error;
}

static int
aq_hw_set_link_speed(struct aq_softc *sc, uint32_t speed)
{
	return aq_hw_mpi_set(sc, MPI_INIT, speed);
}

static int
aq_hw_chip_features_init(struct aq_softc *sc)
{
	int error = 0;
	char fw_vers[sizeof("FW version xxxxx.xxxxx.xxxxx")];

	snprintf(fw_vers, sizeof(fw_vers), "FW version %d.%d.%d",
	    FW_VERSION_MAJOR(sc), FW_VERSION_MINOR(sc), FW_VERSION_BUILD(sc));

	/* detect revision */
	uint32_t hwrev = AQ_READ_REG(sc, HW_ATL_GLB_MIF_ID_ADR);
	switch (hwrev & 0x0000000f) {
	case 0x01:
		aprint_verbose_dev(sc->sc_dev, "Atlantic revision A0, %s\n",
		    fw_vers);
		sc->sc_features |= FEATURES_REV_A0 |
		    FEATURES_MPI_AQ | FEATURES_MIPS;
		break;
	case 0x02:
		aprint_verbose_dev(sc->sc_dev, "Atlantic revision B0, %s\n",
		    fw_vers);
		sc->sc_features |= FEATURES_REV_B0 |
		    FEATURES_MPI_AQ | FEATURES_MIPS |
		    FEATURES_TPO2 | FEATURES_RPF2;
		break;
	case 0x0A:
		aprint_verbose_dev(sc->sc_dev, "Atlantic revision B1, %s\n",
		    fw_vers);
		sc->sc_features |= FEATURES_REV_B1 |
		    FEATURES_MPI_AQ | FEATURES_MIPS |
		    FEATURES_TPO2 | FEATURES_RPF2;
		break;
	default:
		aprint_error_dev(sc->sc_dev,
		    "Unknown revision (0x%08x)\n", hwrev);
		error = ENOTSUP;
		break;
	}
	return error;
}

static int
aq_fw_ops_init(struct aq_softc *sc)
{
	if (FW_VERSION_MAJOR(sc) == 1) {
		sc->sc_fw_ops = &aq_fw1x_ops;
	} else if (FW_VERSION_MAJOR(sc) >= 2) {
		sc->sc_fw_ops = &aq_fw2x_ops;
	} else {
		aprint_error_dev(sc->sc_dev,
		    "invalid F/W version %d.%d.%d\n",
		    FW_VERSION_MAJOR(sc), FW_VERSION_MINOR(sc),
		    FW_VERSION_BUILD(sc));
		return ENOTSUP;
	}
	return 0;
}

static int
fw1x_reset(struct aq_softc *sc)
{
	struct aq_mailbox_header mbox;
	const int retryCount = 1000;
	uint32_t tid0;
	int i;

	tid0 = ~0;	/*< Initial value of MBOX transactionId. */

	for (i = 0; i < retryCount; ++i) {
		/*
		 * Read the beginning of Statistics structure to capture
		 * the Transaction ID.
		 */
		aq_fw_downld_dwords(sc, sc->sc_mbox_addr,
		    (uint32_t *)&mbox, sizeof(mbox) / sizeof(uint32_t));

		/* Successfully read the stats. */
		if (tid0 == ~0U) {
			/* We have read the initial value. */
			tid0 = mbox.transaction_id;
			continue;
		} else if (mbox.transaction_id != tid0) {
			/*
			 * Compare transaction ID to initial value.
			 * If it's different means f/w is alive.
			 * We're done.
			 */
			return 0;
		}

		/*
		 * Transaction ID value haven't changed since last time.
		 * Try reading the stats again.
		 */
		delay(10);
	}
	aprint_error_dev(sc->sc_dev,
	    "F/W 1.x reset finalize timeout\n");
	return EBUSY;
}

static int
fw1x_set_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t mode,
    aq_fw_link_speed_t speed)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	return -1;
}

static int
fw1x_get_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t *mode,
    aq_fw_link_speed_t *speed, aq_fw_link_fc_t *fc)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	return -1;
}

static int
fw1x_get_stats(struct aq_softc *sc, aq_hw_stats_s_t *stats)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	return -1;
}

static int
fw2x_reset(struct aq_softc *sc)
{
	fw2x_capabilities_t caps = { 0 };
	int error;

	error = aq_fw_downld_dwords(sc,
	    sc->sc_mbox_addr + offsetof(fw2x_mailbox_t, caps),
	    (uint32_t *)&caps, sizeof caps / sizeof(uint32_t));
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "fw2x> can't get F/W capabilities mask, error %d\n",
		    error);
		return error;
	}
	sc->sc_fw_caps = caps.caps_lo | ((uint64_t)caps.caps_hi << 32);
	aprint_debug_dev(sc->sc_dev,
	    "fw2x> F/W capabilities mask = %llx\n",
	    (unsigned long long)sc->sc_fw_caps);

	return 0;
}

static uint32_t
link_speed_mask_to_fw2x(aq_fw_link_speed_t speed)
{
	uint32_t rate = 0;

	if (speed & AQ_FW_10G)
		rate |= FW2X_CTRL_RATE_10G;
	if (speed & AQ_FW_5G)
		rate |= FW2X_CTRL_RATE_5G;
	if (speed & AQ_FW_2G5)
		rate |= FW2X_CTRL_RATE_2G5;
	if (speed & AQ_FW_1G)
		rate |= FW2X_CTRL_RATE_1G;
	if (speed & AQ_FW_100M)
		rate |= FW2X_CTRL_RATE_100M;

	return rate;
}

static int
fw2x_set_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t mode,
    aq_fw_link_speed_t speed)
{
	uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_ADDR);

	switch (mode) {
	case MPI_INIT:
		mpi_ctrl &= ~FW2X_CTRL_RATE_MASK;
		mpi_ctrl |= link_speed_mask_to_fw2x(speed);
		mpi_ctrl &= ~FW2X_CTRL_LINK_DROP;
#if 0 /* todo #eee */
		mpi_ctrl &= ~FW2X_CTRL_EEE_MASK;
		if (sc->sc_eee)
			mpi_ctrl |= FW2X_CTRL_EEE_MASK;
#endif
		mpi_ctrl &= ~(FW2X_CTRL_PAUSE | FW2X_CTRL_ASYMMETRIC_PAUSE);
		if (sc->sc_fc.fc_rx)
			mpi_ctrl |= FW2X_CTRL_PAUSE;
		if (sc->sc_fc.fc_tx)
			mpi_ctrl |= FW2X_CTRL_ASYMMETRIC_PAUSE;
		break;
	case MPI_DEINIT:
		mpi_ctrl &= ~(FW2X_CTRL_RATE_MASK | FW2X_CTRL_EEE_MASK);
		mpi_ctrl &= ~(FW2X_CTRL_PAUSE | FW2X_CTRL_ASYMMETRIC_PAUSE);
		break;
	default:
		aprint_error_dev(sc->sc_dev,
		    "fw2x> unknown MPI state %d\n", mode);
		return EINVAL;
	}

	AQ_WRITE64_REG(sc, FW2X_MPI_CONTROL_ADDR, mpi_ctrl);
	return 0;
}

static int
fw2x_get_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t *mode,
    aq_fw_link_speed_t *speedp, aq_fw_link_fc_t *fc)
{
	uint64_t mpi_state = AQ_READ64_REG(sc, FW2X_MPI_STATE_ADDR);
	uint32_t rates = mpi_state & FW2X_CTRL_RATE_MASK;

	if (mode != NULL) {
		uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_ADDR);
		if (mpi_ctrl & FW2X_CTRL_RATE_MASK)
			*mode = MPI_INIT;
		else
			*mode = MPI_DEINIT;
	}

	aq_fw_link_speed_t speed = AQ_FW_NONE;
	if (rates & FW2X_CTRL_RATE_10G)
		speed = AQ_FW_10G;
	else if (rates & FW2X_CTRL_RATE_5G)
		speed = AQ_FW_5G;
	else if (rates & FW2X_CTRL_RATE_2G5)
		speed = AQ_FW_2G5;
	else if (rates & FW2X_CTRL_RATE_1G)
		speed = AQ_FW_1G;
	else if (rates & FW2X_CTRL_RATE_100M)
		speed = AQ_FW_100M;
	if (speedp != NULL)
		*speedp = speed;

	if (fc != NULL) {
		*fc = 0;
		if (mpi_state & FW2X_CTRL_PAUSE)
			*fc |= AQ_FW_FC_ENABLE_RX;
		if (mpi_state & FW2X_CTRL_ASYMMETRIC_PAUSE)
			*fc |= AQ_FW_FC_ENABLE_TX;
	}
	return 0;
}

static int
toggle_mpi_ctrl_and_wait(struct aq_softc *sc, uint64_t mask,
    uint32_t timeout_ms, uint32_t try_count)
{
	uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_ADDR);
	uint64_t mpi_state = AQ_READ64_REG(sc, FW2X_MPI_STATE_ADDR);
	int error;

	/* First, check that control and state values are consistent */
	if ((mpi_ctrl & mask) != (mpi_state & mask)) {
		aprint_error_dev(sc->sc_dev,
		    "fw2x> MPI control (%#llx) and state (%#llx)"
		    " are not consistent for mask %#llx!\n",
		    (unsigned long long)mpi_ctrl, (unsigned long long)mpi_state,
		    (unsigned long long)mask);
		return EINVAL;
	}

	/* Invert bits (toggle) in control register */
	mpi_ctrl ^= mask;
	AQ_WRITE64_REG(sc, FW2X_MPI_CONTROL_ADDR, mpi_ctrl);

	/* Clear all bits except masked */
	mpi_ctrl &= mask;

	/* Wait for FW reflecting change in state register */
	WAIT_FOR((AQ_READ64_REG(sc, FW2X_MPI_CONTROL_ADDR) & mask) == mpi_ctrl,
	    1000 * timeout_ms, try_count, &error);
	if (error != 0) {
		aprint_debug_dev(sc->sc_dev,
		    "f/w2x> timeout while waiting for response"
		    " in state register for bit %#llx!",
		    (unsigned long long)mask);
		return error;
	}
	return 0;
}

static int
fw2x_get_stats(struct aq_softc *sc, aq_hw_stats_s_t *stats)
{
	int error;

	/* Say to F/W to update the statistics */
	error = toggle_mpi_ctrl_and_wait(sc, FW2X_CTRL_STATISTICS, 1, 25);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "fw2x> statistics update error %d\n", error);
		return error;
	}

	CTASSERT(sizeof(fw2x_msm_statistics_t) <= sizeof(struct aq_hw_stats_s));
	error = aq_fw_downld_dwords(sc,
	    sc->sc_mbox_addr + offsetof(fw2x_mailbox_t, msm),
	    (uint32_t *)stats, sizeof(fw2x_msm_statistics_t) / sizeof(uint32_t));
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "fw2x> download statistics data FAILED, error %d", error);
		return error;
	}
	stats->dpc = AQ_READ_REG(sc, RX_DMA_STAT_COUNTER7_ADR);
	stats->cprc = AQ_READ_REG(sc, STATS_RX_LO_COALESCED_PKT_COUNT0_ADDR);

	return 0;
}

static int
fw2x_led_control(struct aq_softc *sc, uint32_t onoff)
{
	if (sc->sc_fw_version >= FW2X_FW_MIN_VER_LED) {
		AQ_WRITE_REG(sc, FW2X_MPI_LED_ADDR, onoff ?
		    ((FW2X_LED_BLINK) | (FW2X_LED_BLINK << 2) |
		    (FW2X_LED_BLINK << 4)) :
		    (FW2X_LED_DEFAULT));
	}
	return 0;
}

static void
aq_hw_init_tx_path(struct aq_softc *sc)
{
	/* Tx TC/RSS number config */
	AQ_OR_REG(sc, TPB_TX_BUF_ADR, TPB_TX_BUF_TC_MODE_EN);

	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG_FIRST_ADR, THM_LSO_TCP_FLAG_FIRST_MSK, 0x0ff6);
	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG_MID_ADR,   THM_LSO_TCP_FLAG_MID_MSK,   0x0ff6);
	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG_LAST_ADR,  THM_LSO_TCP_FLAG_LAST_MSK,  0x0f7f);

	/* Tx interrupts */
	AQ_OR_REG(sc, TDM_INT_DESC_WRB_EN_ADR, TDM_INT_DESC_WRB_EN);

	/* misc */
	AQ_WRITE_REG(sc, 0x7040, (sc->sc_features & FEATURES_TPO2) ? __BIT(16) : 0);
	AQ_AND_REG(sc, TDM_DCA_ADR, ~TDM_DCA_EN);
	AQ_AND_REG(sc, TDM_DCA_ADR, ~TDM_DCA_MODE);

	AQ_OR_REG(sc, TPB_TX_BUF_ADR, TPB_TX_BUF_SCP_INS_EN);
}

#define TPS_DESC_VM_ARB_MODE_ADR		0x7300
#define  TPS_DESC_VM_ARB_MODE_MSK		__BIT(0)
#define TPS_DESC_RATE_REG			0x7310
#define  TPS_DESC_RATE_TA_RST			__BIT(31)
#define  TPS_DESC_RATE_LIM			__BITS(10,0)
#define TPS_DESC_TC_ARB_MODE_ADR		0x7200
#define  TPS_DESC_TC_ARB_MODE_MSK		__BITS(1,0)
#define TPS_DATA_TC_ARB_MODE_ADR		0x7100
#define  TPS_DATA_TC_ARB_MODE_MSK		__BIT(0)
#define TPS_DATA_TCTCREDIT_MAX_ADR(tc)		(0x7110 + (tc) * 0x4)
#define  TPS_DATA_TCTCREDIT_MAX_MSK		__BITS(16,27)
#define TPS_DATA_TCTWEIGHT_ADR(tc)		TPS_DATA_TCTCREDIT_MAX_ADR(tc)
#define  TPS_DATA_TCTWEIGHT_MSK			__BITS(8,0)
#define TPS_DESC_TCTCREDIT_MAX_ADR(tc)		(0x7210 + (tc) * 0x4)
#define  TPS_DESC_TCTCREDIT_MAX_MSK		__BITS(16,27)
#define TPS_DESC_TCTWEIGHT_ADR(tc)		TPS_DESC_TCTCREDIT_MAX_ADR(tc)
#define  TPS_DESC_TCTWEIGHT_MSK			__BITS(8,0)

#define AQ_HW_TXBUF_MAX		160
#define AQ_HW_RXBUF_MAX		320

#define TPB_TXBBUF_SIZE_ADR(buffer)		(0x7910 + (buffer) * 0x10)
#define  TPB_TXBBUF_SIZE_MSK			__BITS(7,0)
#define TPB_TXBHI_THRESH_ADR(buffer)		(0x7914 + (buffer) * 0x10)
#define  TPB_TXBHI_THRESH_MSK			__BITS(16,28)
#define TPB_TXBLO_THRESH_ADR(buffer)		(0x7914 + (buffer) * 0x10)
#define  TPB_TXBLO_THRESH_MSK			__BITS(12,0)


static void
aq_hw_qos_set(struct aq_softc *sc)
{
	uint32_t tc = 0;
	uint32_t buff_size = 0;

	/* TPS Descriptor rate init */
	AQ_WRITE_REG_BIT(sc, TPS_DESC_RATE_REG, TPS_DESC_RATE_TA_RST, 0);
	AQ_WRITE_REG_BIT(sc, TPS_DESC_RATE_REG, TPS_DESC_RATE_LIM, 0xa);

	/* TPS VM init */
	AQ_WRITE_REG_BIT(sc, TPS_DESC_VM_ARB_MODE_ADR, TPS_DESC_VM_ARB_MODE_MSK, 0);

	/* TPS TC credits init */
	AQ_WRITE_REG_BIT(sc, TPS_DESC_TC_ARB_MODE_ADR, TPS_DESC_TC_ARB_MODE_MSK, 0);
	AQ_WRITE_REG_BIT(sc, TPS_DATA_TC_ARB_MODE_ADR, TPS_DATA_TC_ARB_MODE_MSK, 0);

	AQ_WRITE_REG_BIT(sc, TPS_DATA_TCTCREDIT_MAX_ADR(tc), TPS_DATA_TCTCREDIT_MAX_MSK, 0xfff);
	AQ_WRITE_REG_BIT(sc, TPS_DATA_TCTWEIGHT_ADR(tc), TPS_DATA_TCTWEIGHT_MSK, 0x64);
	AQ_WRITE_REG_BIT(sc, TPS_DESC_TCTCREDIT_MAX_ADR(tc), TPS_DESC_TCTCREDIT_MAX_MSK, 0x50);
	AQ_WRITE_REG_BIT(sc, TPS_DESC_TCTWEIGHT_ADR(tc), TPS_DESC_TCTWEIGHT_MSK, 0x1e);

	/* Tx buf size */
	buff_size = AQ_HW_TXBUF_MAX;

	AQ_WRITE_REG_BIT(sc, TPB_TXBBUF_SIZE_ADR(tc), TPB_TXBBUF_SIZE_MSK, buff_size);
	AQ_WRITE_REG_BIT(sc, TPB_TXBHI_THRESH_ADR(tc), TPB_TXBHI_THRESH_MSK, (buff_size * (1024 / 32U) * 66U) / 100);
	AQ_WRITE_REG_BIT(sc, TPB_TXBLO_THRESH_ADR(tc), TPB_TXBLO_THRESH_MSK, (buff_size * (1024 / 32U) * 50U) / 100);

//	/* QoS Rx buf size per TC */
//	tc = 0;
//	buff_size = AQ_HW_RXBUF_MAX;
//
//	rpb_rx_pkt_buff_size_per_tc_set(hw, buff_size, tc);
//	rpb_rx_buff_hi_threshold_per_tc_set(hw,
//						(buff_size *
//						(1024U / 32U) * 66U) /
//						100U, tc);
//	rpb_rx_buff_lo_threshold_per_tc_set(hw,
//						(buff_size *
//						(1024U / 32U) * 50U) /
//						100U, tc);

//	/* QoS 802.1p priority -> TC mapping */
//	unsigned int i_priority;
//	for (i_priority = 0; i_priority < 8; i_priority++)
//		rpf_rpb_user_priority_tc_map_set(hw, i_priority, 0);


}

static int
aq_hw_init(struct aq_softc *sc)
{
	uint32_t v;

	/* Force limit MRRS on RDM/TDM to 2K */
	v = AQ_READ_REG(sc, AQ_HW_PCI_REG_CONTROL_6_ADR);
	AQ_WRITE_REG(sc, AQ_HW_PCI_REG_CONTROL_6_ADR,
	    (v & ~0x0707) | 0x0404);

	/*
	 * TX DMA total request limit. B0 hardware is not capable to
	 * handle more than (8K-MRRS) incoming DMA data.
	 * Value 24 in 256byte units
	 */
	AQ_WRITE_REG(sc, AQ_HW_TX_DMA_TOTAL_REQ_LIMIT_ADR, 24);

	aq_hw_init_tx_path(sc);
//	aq_hw_init_rx_path(sc);

	aq_set_mac_addr(sc, AQ_HW_MAC, sc->sc_enaddr.ether_addr_octet);
	aq_hw_mpi_set(sc, MPI_INIT, sc->sc_link_rate);

	aq_hw_qos_set(sc);

//	/* Enable interrupt */
//	itr_irq_status_cor_en_set(hw, 0); //Disable clear-on-read for status
//	itr_irq_auto_mask_clr_en_set(hw, 1); // Enable auto-mask clear.
//		if (msix)
//				itr_irq_mode_set(hw, 0x6); //MSIX + multi vector
//		else
//				itr_irq_mode_set(hw, 0x5); //MSI + multi vector
//
//	reg_gen_irq_map_set(hw, 0x80 | adm_irq, 3);

//	aq_hw_offload_set(hw);

	return 0;
}

static void
aq_if_update_admin_status(struct aq_softc *sc)
{
	struct aq_hw_fc_info fc_neg;
	uint32_t link_speed = 0;

	aq_hw_get_link_state(sc, &link_speed, &fc_neg);
	if ((sc->sc_link_speed != link_speed) && (link_speed != 0)) {
		/* link DOWN -> UP */
		aprint_debug_dev(sc->sc_dev, "link UP: speed=%d\n", link_speed);
		sc->sc_link_speed = link_speed;

		/* turn on/off RX Pause in RPB */
		if (fc_neg.fc_rx)
			AQ_OR_REG(sc, RPB_RXBXOFF_EN_ADR, RPB_RXBXOFF_EN);
		else
			AQ_AND_REG(sc, RPB_RXBXOFF_EN_ADR, ~RPB_RXBXOFF_EN);

		aq_mediastatus_update(sc, link_speed, &fc_neg);

		/* update ITR settings according new link speed */
//		aq_hw_interrupt_moderation_set(sc);

	} else if (link_speed == 0 && sc->sc_link_speed != 0) {
		/* link UP -> DOWN */
		aprint_debug_dev(sc->sc_dev, "link DOWN\n");
		sc->sc_link_speed = 0;

		/* turn off RX Pause in RPB */
		AQ_AND_REG(sc, RPB_RXBXOFF_EN_ADR, ~RPB_RXBXOFF_EN);

		aq_mediastatus_update(sc, link_speed, &fc_neg);
	}

	if (sc->sc_statistics_enable) {
		int prev = sc->sc_statistics_idx;
		int cur = prev ^ 1;

		sc->sc_fw_ops->get_stats(sc, &sc->sc_statistics[cur]);

#define ADD_DELTA(cur,prev,name,descr)	\
		do {															\
			uint64_t n = (uint32_t)(sc->sc_statistics[cur].name - sc->sc_statistics[prev].name);				\
			/* printf("# %s: %s: %u\n", descr, #name, sc->sc_statistics[cur].name); /**/					\
			if (n != 0) {													\
				printf("====================%s: %s: %lu -> %lu (+%lu)\n", descr, #name, sc->sc_statistics_ ## name, sc->sc_statistics_ ## name + n, n);	\
			}														\
			sc->sc_statistics_ ## name += n;										\
		} while (/*CONSTCOND*/0);

		ADD_DELTA(cur, prev, uprc, "rx ucast");
		ADD_DELTA(cur, prev, mprc, "rx mcast");
		ADD_DELTA(cur, prev, bprc, "rx bcast");
		ADD_DELTA(cur, prev, prc, "rx good");
		ADD_DELTA(cur, prev, erpr, "rx error");
		ADD_DELTA(cur, prev, uptc, "tx ucast");
		ADD_DELTA(cur, prev, mptc, "tx mcast");
		ADD_DELTA(cur, prev, bptc, "tx bcast");
		ADD_DELTA(cur, prev, ptc, "tx good");
		ADD_DELTA(cur, prev, erpt, "tx error");
		ADD_DELTA(cur, prev, mbtc, "tx mcast bytes");
		ADD_DELTA(cur, prev, bbtc, "tx bcast bytes");
		ADD_DELTA(cur, prev, mbrc, "rx mcast bytes");
		ADD_DELTA(cur, prev, bbrc, "rx bcast bytes");
		ADD_DELTA(cur, prev, ubrc, "rx ucast bytes");
		ADD_DELTA(cur, prev, ubtc, "tx ucast bytes");
		ADD_DELTA(cur, prev, dpc, "dma drop");
		ADD_DELTA(cur, prev, cprc, "rx coalesced");

		sc->sc_statistics_idx = cur;
	}

}

/* allocate and map one DMA blocks */
static int
_alloc_dma(struct aq_softc *sc, bus_size_t size, bus_size_t *sizep,
    void **addrp, bus_dmamap_t *mapp, bus_dma_segment_t *seg)
{
	int nsegs, error;

	if ((error = bus_dmamem_alloc(sc->sc_dmat, size, PAGE_SIZE, 0, seg,
	    1, &nsegs, M_NOWAIT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to allocate DMA buffer, error=%d\n", error);
		goto fail_alloc;
	}

	if ((error = bus_dmamem_map(sc->sc_dmat, seg, 1, size, addrp,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to map DMA buffer, error=%d\n", error);
		goto fail_map;
	}

	if ((error = bus_dmamap_create(sc->sc_dmat, size, 1, size, 0,
	    BUS_DMA_NOWAIT, mapp)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to create DMA map, error=%d\n", error);
		goto fail_create;
	}

	if ((error = bus_dmamap_load(sc->sc_dmat, *mapp, *addrp, size, NULL,
	    BUS_DMA_NOWAIT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to load DMA map, error=%d\n", error);
		goto fail_load;
	}

	*sizep = size;
	return 0;

 fail_load:
	bus_dmamap_destroy(sc->sc_dmat, *mapp);
	*mapp = NULL;
 fail_create:
	bus_dmamem_unmap(sc->sc_dmat, *addrp, size);
	*addrp = NULL;
 fail_map:
	bus_dmamem_free(sc->sc_dmat, seg, 1);
	memset(seg, 0, sizeof(*seg));
 fail_alloc:
	*sizep = 0;
	return error;
}

static void
_free_dma(struct aq_softc *sc, bus_size_t *sizep, void **addrp, bus_dmamap_t *mapp, bus_dma_segment_t *seg)
{
	if (*mapp != NULL) {
		bus_dmamap_destroy(sc->sc_dmat, *mapp);
		*mapp = NULL;
	}
	if (*addrp != NULL) {
		bus_dmamem_unmap(sc->sc_dmat, *addrp, *sizep);
		*addrp = NULL;
	}
	if (*sizep != 0) {
		bus_dmamem_free(sc->sc_dmat, seg, 1);
		memset(seg, 0, sizeof(*seg));
		*sizep = 0;
	}
}

/* allocate a tx ring */
static int
aq_txring_alloc(struct aq_softc *sc, struct aq_txring *txring)
{
	int i, error;

	/* allocate tx descriptors */
	error = _alloc_dma(sc, sizeof(aq_tx_desc_t) * AQ_TXD_NUM,
	    &txring->ring_txdesc_size, (void **)&txring->ring_txdesc,
	    &txring->ring_txdesc_dmamap, txring->ring_txdesc_seg);
	if (error != 0)
		return error;

	memset(txring->ring_txdesc, 0, sizeof(aq_tx_desc_t) * AQ_TXD_NUM);

	/* fill tx ring with dmamap */
	for (i = 0; i < AQ_TXD_NUM; i++) {
#define AQ_MAXDMASIZE	(16 * 1024)
#define AQ_NTXSEGS	32
		bus_dmamap_create(sc->sc_dmat, AQ_MAXDMASIZE, AQ_NTXSEGS,
		    AQ_MAXDMASIZE, 0, 0,
		    &txring->ring_mbufs[i].dmamap);
	}
	return 0;
}

static void
aq_txring_free(struct aq_softc *sc, struct aq_txring *txring)
{
	int i;

	_free_dma(sc, &txring->ring_txdesc_size, (void **)&txring->ring_txdesc,
	    &txring->ring_txdesc_dmamap, txring->ring_txdesc_seg);

	for (i = 0; i < AQ_TXD_NUM; i++) {
		if (txring->ring_mbufs[i].dmamap != NULL) {
			if (txring->ring_mbufs[i].m != NULL) {
				bus_dmamap_unload(sc->sc_dmat, txring->ring_mbufs[i].dmamap);
				m_freem(txring->ring_mbufs[i].m);
				txring->ring_mbufs[i].m = NULL;
			}
			bus_dmamap_destroy(sc->sc_dmat, txring->ring_mbufs[i].dmamap);
			txring->ring_mbufs[i].dmamap = NULL;
		}
	}
}

static int
aq_rx_slot_ready(struct aq_softc *sc, struct aq_rxring *rxring, int idx)
{
	struct mbuf *m;
	int error;

	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL)
		return ENOBUFS;

	MCLGET(m, M_DONTWAIT);
	if ((m->m_flags & M_EXT) == 0) {
		m_freem(m);
		return ENOBUFS;
	}

	if (rxring->ring_mbufs[idx].m != NULL)
		bus_dmamap_unload(sc->sc_dmat, rxring->ring_mbufs[idx].dmamap);

	rxring->ring_mbufs[idx].m = m;

	m->m_len = m->m_pkthdr.len = m->m_ext.ext_size;
	error = bus_dmamap_load_mbuf(sc->sc_dmat, rxring->ring_mbufs[idx].dmamap,
	    m, BUS_DMA_NOWAIT);
	if (error) {
		aprint_error_dev(sc->sc_dev,
		    "unable to load rx DMA map %d, error = %d\n", idx, error);
		panic("%s: unable to load rx DMA map. error=%d", __func__, error);
	}

	bus_dmamap_sync(sc->sc_dmat, rxring->ring_mbufs[idx].dmamap, 0,
	    rxring->ring_mbufs[idx].dmamap->dm_mapsize, BUS_DMASYNC_PREREAD);

	return 0;
}

/* allocate a rx ring */
static int
aq_rxring_alloc(struct aq_softc *sc, struct aq_rxring *rxring)
{
	int i, error;

	/* allocate rx descriptors */
	error = _alloc_dma(sc, sizeof(aq_rx_desc_t) * AQ_RXD_NUM,
	    &rxring->ring_rxdesc_size, (void **)&rxring->ring_rxdesc,
	    &rxring->ring_rxdesc_dmamap, rxring->ring_rxdesc_seg);
	if (error != 0)
		return error;

	memset(rxring->ring_rxdesc, 0, sizeof(aq_rx_desc_t) * AQ_RXD_NUM);

	/* fill rx ring with dmamaps and mbufs */
	for (i = 0; i < AQ_RXD_NUM; i++) {
		bus_dmamap_create(sc->sc_dmat, MCLBYTES, 1,
		    MCLBYTES, 0, 0,
		    &rxring->ring_mbufs[i].dmamap);

		error = aq_rx_slot_ready(sc, rxring, i);
		if (error != 0) {
			aq_rxring_free(sc, rxring);
			return error;
		}
	}
	return 0;
}

/* free a rx ring */
static void
aq_rxring_free(struct aq_softc *sc, struct aq_rxring *rxring)
{
	int i;

	_free_dma(sc, &rxring->ring_rxdesc_size, (void **)&rxring->ring_rxdesc,
	    &rxring->ring_rxdesc_dmamap, rxring->ring_rxdesc_seg);

	for (i = 0; i < AQ_RXD_NUM; i++) {
		if (rxring->ring_mbufs[i].dmamap != NULL) {
			if (rxring->ring_mbufs[i].m != NULL) {
				bus_dmamap_unload(sc->sc_dmat, rxring->ring_mbufs[i].dmamap);
				m_freem(rxring->ring_mbufs[i].m);
				rxring->ring_mbufs[i].m = NULL;
			}
			bus_dmamap_destroy(sc->sc_dmat, rxring->ring_mbufs[i].dmamap);
			rxring->ring_mbufs[i].dmamap = NULL;
		}
	}
}

static int
aq_txrx_rings_alloc(struct aq_softc *sc)
{
	int n, error;

	for (n = 0; n < AQ_TXRING_NUM; n++) {
		sc->sc_txring[n].ring_sc = sc;
		sc->sc_txring[n].ring_index = n;
		error = aq_txring_alloc(sc, &sc->sc_txring[n]);
		if (error != 0)
			goto failure;
	}

	for (n = 0; n < AQ_RXRING_NUM; n++) {
		sc->sc_rxring[n].ring_sc = sc;
		sc->sc_rxring[n].ring_index = n;
		error = aq_rxring_alloc(sc, &sc->sc_rxring[n]);
		if (error != 0)
			break;
	}

 failure:
	return error;
}

static void
dump_txrings(struct aq_softc *sc)
{
	struct aq_txring *txring;
	int n, i;

	for (n = 0; n < AQ_TXRING_NUM; n++) {
		txring = &sc->sc_txring[n];

		printf("# txring=%p (index=%d)\n", txring, txring->ring_index);
		printf("txring->ring_txdesc        = %p\n", txring->ring_txdesc);
		printf("txring->ring_txdesc_dmamap = %p\n", txring->ring_txdesc_dmamap);
		printf("txring->ring_txdesc_size   = %lu\n", txring->ring_txdesc_size);

		for (i = 0; i < AQ_TXD_NUM; i++) {
			if (txring->ring_txdesc[i].buf_addr == 0)
				continue;

			printf("txring->ring_mbufs [%d].m        = %p\n", i, txring->ring_mbufs[i].m);
			printf("txring->ring_txdesc[%d].buf_addr = %08lx\n", i, txring->ring_txdesc[i].buf_addr);
			printf("txring->ring_txdesc[%d].ctl  = %08x%s\n", i, txring->ring_txdesc[i].ctl,
			    (txring->ring_txdesc[i].ctl & AQ_TXDESC_CTL_EOP) ? " EOP" : "");
			printf("txring->ring_txdesc[%d].ctl2 = %08x\n", i, txring->ring_txdesc[i].ctl2);
		}
	}
}

static void
dump_rxrings(struct aq_softc *sc)
{
	struct aq_rxring *rxring;
	int n, i;

	for (n = 0; n < AQ_RXRING_NUM; n++) {
		rxring = &sc->sc_rxring[n];

		printf("# rxring=%p (index=%d)\n", rxring, rxring->ring_index);
		printf("rxring->ring_rxdesc        = %p\n", rxring->ring_rxdesc);
		printf("rxring->ring_rxdesc_dmamap = %p\n", rxring->ring_rxdesc_dmamap);
		printf("rxring->ring_rxdesc_size   = %lu\n", rxring->ring_rxdesc_size);
		for (i = 0; i < AQ_RXD_NUM; i++) {
			printf("rxring->ring_mbufs[%d].m      = %p\n", i, rxring->ring_mbufs[i].m);
			printf("rxring->ring_mbufs[%d].dmamap = %p\n", i, rxring->ring_mbufs[i].dmamap);
		}
	}
}

static void
aq_txrx_rings_free(struct aq_softc *sc)
{
	int n;

	for (n = 0; n < AQ_TXRING_NUM; n++)
		aq_txring_free(sc, &sc->sc_txring[n]);

	for (n = 0; n < AQ_RXRING_NUM; n++)
		aq_rxring_free(sc, &sc->sc_rxring[n]);
}

static void
aq_tick(void *arg)
{
	struct aq_softc *sc = arg;

	aq_if_update_admin_status(sc);

	callout_reset(&sc->sc_tick_ch, hz, aq_tick, sc);
}

static int
aq_intr(void *arg)
{
	struct aq_softc *sc __unused = arg;

	printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!%s\n", __func__);
	return 0;
}


static const struct aq_product *
aq_lookup(const struct pci_attach_args *pa)
{
	unsigned int i;

	for (i = 0; i < __arraycount(aq_products); i++) {
		if (PCI_VENDOR(pa->pa_id)  == aq_products[i].aq_vendor &&
		    PCI_PRODUCT(pa->pa_id) == aq_products[i].aq_product)
			return &aq_products[i];
	}
	return NULL;
}

static int
aq_match(device_t parent, cfdata_t cf, void *aux)
{
	struct pci_attach_args *pa = aux;

	if (aq_lookup(pa) != NULL)
		return 1;

	return 0;
}

static void
aq_attach(device_t parent, device_t self, void *aux)
{
	struct aq_softc *sc = device_private(self);
	struct pci_attach_args *pa = aux;
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	pci_chipset_tag_t pc;
	pcitag_t tag;
	pcireg_t memtype, bar;
	const struct aq_product *aqp;
	pci_intr_handle_t ih;
	const char *intrstr;
	char intrbuf[PCI_INTRSTR_LEN];
	int error;

	sc->sc_dev = self;
	mutex_init(&sc->sc_mutex, MUTEX_DEFAULT, IPL_NET);
	callout_init(&sc->sc_tick_ch, 0);	/* XXX: CALLOUT_MPSAFE */
	sc->sc_pc = pc = pa->pa_pc;
	sc->sc_pcitag = tag = pa->pa_tag;
	sc->sc_dmat = pci_dma64_available(pa) ? pa->pa_dmat64 : pa->pa_dmat;

	sc->sc_product = PCI_PRODUCT(pa->pa_id);
	sc->sc_revision = PCI_REVISION(pa->pa_class);

	aqp = aq_lookup(pa);
	KASSERT(aqp != NULL);

	pci_aprint_devinfo_fancy(pa, "Ethernet controller", aqp->aq_name, 1);

	bar = pci_conf_read(pc, tag, PCI_BAR(0));
	if ((PCI_MAPREG_MEM_ADDR(bar) == 0) ||
	    (PCI_MAPREG_TYPE(bar) != PCI_MAPREG_TYPE_MEM)) {
		aprint_error_dev(sc->sc_dev, "wrong BAR type\n");
		return;
	}
	memtype = pci_mapreg_type(pc, tag, PCI_BAR(0));
	if (pci_mapreg_map(pa, PCI_BAR(0), memtype, 0, &sc->sc_iot, &sc->sc_ioh,
	    NULL, &sc->sc_iosize) != 0) {
		aprint_error_dev(sc->sc_dev, "unable to map register\n");
		return;
	}

	if (pci_intr_map(pa, &ih)) {
		aprint_error_dev(sc->sc_dev, "unable to map interrupt\n");
		return;
	}
	intrstr = pci_intr_string(pc, ih, intrbuf, sizeof(intrbuf));
	sc->sc_intrhand = pci_intr_establish_xname(pc, ih, IPL_NET, aq_intr,
	    sc, device_xname(self));
	if (sc->sc_intrhand == NULL) {
		aprint_error_dev(self, "unable to establish interrupt");
		if (intrstr != NULL)
			aprint_error(" at %s", intrstr);
		aprint_error("\n");
		return;
	}
	aprint_normal_dev(self, "interrupting at %s\n", intrstr);

	error = aq_txrx_rings_alloc(sc);
	if (error != 0)
		goto attach_failure;

	error = aq_fw_reset(sc);
	if (error != 0)
		goto attach_failure;

	error = aq_hw_chip_features_init(sc);
	if (error != 0)
		goto attach_failure;

	error = aq_fw_ops_init(sc);
	if (error != 0)
		goto attach_failure;

	error = aq_hw_init_ucp(sc);
	if (error < 0)
		goto attach_failure;

	KASSERT(sc->sc_mbox_addr != 0);
	error = aq_hw_reset(sc);
	if (error != 0)
		goto attach_failure;

	sc->sc_media_type = aqp->aq_media_type;
	sc->sc_available_rates = aqp->aq_available_rates;
	sc->sc_link_rate = AQ_FW_SPEED_AUTO;
	sc->sc_fc.fc_rx = true;
	sc->sc_fc.fc_tx = true;
	sc->sc_link_speed = 0;

	aq_get_mac_addr(sc);

	sc->sc_ethercom.ec_ifmedia = &sc->sc_media;
	ifmedia_init(&sc->sc_media, IFM_IMASK,
	    aq_ifmedia_change, aq_ifmedia_status);
	aq_initmedia(sc);

	strlcpy(ifp->if_xname, device_xname(self), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_baudrate = IF_Gbps(10);
	ifp->if_init = aq_init;
	ifp->if_ioctl = aq_ioctl;
	ifp->if_start = aq_start;
	ifp->if_stop = aq_stop;
	ifp->if_watchdog = aq_watchdog;
	IFQ_SET_READY(&ifp->if_snd);

	//XXX: notyet
	ifp->if_capabilities = 0;
	ifp->if_capenable = 0;

	sc->sc_ethercom.ec_capabilities = ETHERCAP_VLAN_MTU;

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr.ether_addr_octet);

	/* media update */
	aq_hw_set_link_speed(sc, sc->sc_link_rate);

	/* get starting statistics values */
	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->get_stats != NULL &&
	    (sc->sc_fw_ops->get_stats(sc, &sc->sc_statistics[0]) == 0)) {
		sc->sc_statistics_enable = true;
	}

	callout_reset(&sc->sc_tick_ch, hz, aq_tick, sc);

	return;

 attach_failure:
	aq_detach(self, 0);
}

static int
aq_detach(device_t self, int flags __unused)
{
	struct aq_softc *sc = device_private(self);
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	int s;

	if (sc->sc_iosize != 0) {
		s = splnet();
		aq_stop(ifp, 0);
		splx(s);

		if (sc->sc_intrhand != NULL) {
			pci_intr_disestablish(sc->sc_pc, sc->sc_intrhand);
			sc->sc_intrhand = NULL;
		}

		aq_txrx_rings_free(sc);

		ether_ifdetach(ifp);
		if_detach(ifp);

		// XXX: free mbufs

		aprint_debug_dev(sc->sc_dev, "%s: bus_space_unmap\n", __func__);
		bus_space_unmap(sc->sc_iot, sc->sc_ioh, sc->sc_iosize);
		sc->sc_iosize = 0;
	}

	mutex_destroy(&sc->sc_mutex);

	return 0;
}

static void
aq_iff(struct aq_softc *sc)
{
}

static int
aq_ifmedia_change(struct ifnet * const ifp)
{
	return aq_mediachange(ifp);
}

static void
aq_ifmedia_status(struct ifnet * const ifp, struct ifmediareq *req)
{
	aq_mediastatus(ifp, req);
}

///* Interrupt enable / disable */
//static void
//aq_if_enable_intr(struct aq_softc *sc)
//{
//	/* Enable interrupts */
//	AQ_WRITE_REG(sc, ITR_IMSRLSW_ADR, __BIT(0));	// XXX
//}

static void
aq_if_disable_intr(struct aq_softc *sc)
{
	/* mask interrupts */
	AQ_WRITE_REG(sc, ITR_IMCRLSW_ADR, 0xffffffff);
}

static void
aq_txring_init(struct aq_softc *sc, struct aq_txring *txring, bool enable_dma)
{
	const int ringidx = txring->ring_index;
	int i;

	txring->ring_prodidx = 0;
	txring->ring_considx = 0;
	txring->ring_nfree = AQ_TXD_NUM;

	/* free mbufs untransmitted */
	for (i = 0; i < AQ_TXD_NUM; i++) {
		if (txring->ring_mbufs[i].m != NULL) {
			m_freem(txring->ring_mbufs[i].m);
			txring->ring_mbufs[i].m = NULL;
		}
	}

	/* disable DMA once */
	AQ_AND_REG(sc, TX_DMA_DESC_LEN_ADR(ringidx), ~TX_DMA_DESC_LEN_ENABLE);
	if (enable_dma) {
		/* TX descriptor physical address */
		paddr_t paddr = txring->ring_txdesc_dmamap->dm_segs[0].ds_addr;
		AQ_WRITE_REG(sc, TX_DMA_DESC_BASE_ADDRLSW_ADR(ringidx), paddr);
		AQ_WRITE_REG(sc, TX_DMA_DESC_BASE_ADDRMSW_ADR(ringidx), paddr >> 32);

		/* TX descriptor size */
		AQ_WRITE_REG(sc, TX_DMA_DESC_LEN_ADR(ringidx),
		    __SHIFTIN(sizeof(aq_tx_desc_t) * AQ_TXD_NUM, TX_DMA_DESC_LEN_MSK));

		AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(ringidx), 0);
		AQ_WRITE_REG(sc, TX_DMA_DESC_WRWB_THRESH_ADR(ringidx), 0);

		/* enable DMA */
		AQ_OR_REG(sc, TX_DMA_DESC_LEN_ADR(ringidx),
		    TX_DMA_DESC_LEN_ENABLE);

		const int cpuid = 0;	//XXX
		AQ_WRITE_REG(sc, TDM_DCADCPUID_ADR(ringidx),
		    __SHIFTIN(cpuid, TDM_DCADCPUID_MSK));
	}
}

static void
aq_txring_start(struct aq_softc *sc, struct aq_txring *txring)
{
	printf("%s:%d: txring[%d] index -> %d\n", __func__, __LINE__, txring->ring_index, txring->ring_prodidx);

	AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(txring->ring_index),
	    txring->ring_prodidx);
}

static void
aq_rxring_init(struct aq_softc *sc, struct aq_rxring *rxring, bool enable_dma)
{
#if 0//notyet
	uint64_t paddr = rxring->ring_rxdesc_dmamap->dm_segs[0].ds_addr;
	uint32_t paddr_lo = (uint32_t)paddr;
	uint32_t paddr_hi = (uint32_t)(paddr >> 32);
	int idx = rxring->ring_index;

	rdm_rx_desc_en_set(sc, 0, idx);
	rdm_rx_desc_head_splitting_set(sc, 0, idx);
	reg_rx_dma_desc_base_addresslswset(sc, paddr_lo, idx);
	reg_rx_dma_desc_base_addressmswset(sc, paddr_hi, idx);
	rdm_rx_desc_len_set(sc, rxring->ring_size / 8, idx);

	rdm_rx_desc_data_buff_size_set(sc, rxring->rx_max_frame_size / 1024, idx);

	rdm_rx_desc_head_buff_size_set(sc, 0, idx);
	rdm_rx_desc_head_splitting_set(sc, 0, idx);
	rpo_rx_desc_vlan_stripping_set(sc, 0, idx);

	/* Rx ring set mode */

	/* Mapping interrupt vector */
	itr_irq_map_rx_set(sc, rxring->ring_msix, idx);
	itr_irq_map_en_rx_set(sc, 1, idx);

	rdm_cpu_id_set(sc, 0, idx);
	rdm_rx_desc_dca_en_set(sc, 0, idx);
	rdm_rx_head_dca_en_set(sc, 0, idx);
	rdm_rx_pld_dca_en_set(sc, 0, idx);
#endif

}

static void
aq_rxring_start(struct aq_softc *sc, struct aq_rxring *rxring)
{
	//XXX
}

static void
aq_txring_intr_enable(struct aq_softc *sc, struct aq_txring *txring)
{
	//XXX
}

static void
aq_rxring_intr_enable(struct aq_softc *sc, struct aq_rxring *rxring)
{
	//XXX
}

#define TXRING_NEXTIDX(idx)	\
	(((idx) >= (AQ_TXD_NUM - 1)) ? 0 : ((idx) + 1))
#define RXRING_NEXTIDX(ring,idx)	\
	(((idx) >= (AQ_RXD_NUM - 1)) ? 0 : ((idx) + 1))

static int
aq_encap_txring(struct aq_softc *sc, struct aq_txring *txring, struct mbuf **mp)
{
	bus_dmamap_t map;
	struct mbuf *m;
	int idx, i, error;

	idx = txring->ring_prodidx;
	map = txring->ring_mbufs[idx].dmamap;

	m = *mp;

	error = bus_dmamap_load_mbuf(sc->sc_dmat, map, m,
	    BUS_DMA_NOWAIT);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "Error mapping mbuf into TX chain: error=%d\n", error);
		m_freem(m);
		return error;
	}

	if (map->dm_nsegs > txring->ring_nfree) {
		bus_dmamap_unload(sc->sc_dmat, map);
		device_printf(sc->sc_dev,
		    "too many mbuf chain %d\n", map->dm_nsegs);
		m_freem(m);
		return ENOBUFS;
	}

	bus_dmamap_sync(sc->sc_dmat, map, 0, map->dm_mapsize,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	/* fill descriptor(s) */
	for (i = 0; i < map->dm_nsegs; i++) {
		txring->ring_txdesc[idx].buf_addr = htole64(map->dm_segs[i].ds_addr);
		txring->ring_txdesc[idx].ctl =
		    AQ_TXDESC_CTL_TYPE_TXD |
		    __SHIFTIN(map->dm_segs[i].ds_len, AQ_TXDESC_CTL_BLEN);
		txring->ring_txdesc[idx].ctl2 =
		    __SHIFTIN(m->m_pkthdr.len, AQ_TXDESC_CTL2_LEN);

		if (i == 0) {
			/* remember mbuf of these descriptors */
			txring->ring_mbufs[idx].m = m;
		} else {
			txring->ring_mbufs[idx].m = NULL;
		}

		if (i == map->dm_nsegs - 1) {
			/* EndOfPacket. mark last segment */
			txring->ring_txdesc[idx].ctl |=
			    AQ_TXDESC_CTL_EOP;
		}

		 printf("%s:%d: write txdesc[%3d] %d/%d buf_addr=%012lx, len=%-5lu ctl=%08x ctl2=%08x%s\n", __func__, __LINE__, idx,
		    i, map->dm_nsegs - 1,
		     map->dm_segs[i].ds_addr,
		     map->dm_segs[i].ds_len,
		    txring->ring_txdesc[idx].ctl,
		    txring->ring_txdesc[idx].ctl2,
		     (i == map->dm_nsegs - 1) ? " EOP" : "");


		bus_dmamap_sync(sc->sc_dmat, txring->ring_txdesc_dmamap,
		    sizeof(aq_tx_desc_t) * idx, sizeof(aq_tx_desc_t),
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

		idx = TXRING_NEXTIDX(idx);
		txring->ring_nfree--;
	}

	txring->ring_prodidx = idx;

	return 0;
}


static int
aq_init(struct ifnet *ifp)
{
	struct aq_softc *sc = ifp->if_softc;
	int n, error;

	//XXX: need lock
	printf("%s:%d\n", __func__, __LINE__);

	error = aq_hw_init(sc);	/* initialize, and enable intr */
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "aq_init failure\n");
		goto done;
	}

//	aq_update_vlan_filters()

	for (n = 0; n < AQ_TXRING_NUM; n++) {
		aq_txring_init(sc, &sc->sc_txring[n], true);
		aq_txring_intr_enable(sc, &sc->sc_txring[n]);
	}

	AQ_OR_REG(sc, TPB_TX_BUF_ADR, TPB_TX_BUF_EN);

	for (n = 0; n < AQ_RXRING_NUM; n++) {
		aq_rxring_init(sc, &sc->sc_rxring[n], true);
		aq_rxring_intr_enable(sc, &sc->sc_rxring[n]);
		aq_rxring_start(sc, &sc->sc_rxring[n]);
	}

	//XXX
	(void)&dump_txrings;
	(void)&dump_rxrings;
//	dump_txrings(sc);
//	dump_rxrings(sc);


//	aq_hw_start();
//	aq_if_enable_intr();
//	aq_hw_rss_hash_set();
//	aq_hw_rss_set();
//	aq_hw_udp_rss_enable();

	/* ready */
	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

 done:
	return error;
}

static void
aq_start(struct ifnet *ifp)
{
	struct aq_softc *sc;
	struct mbuf *m;
	int npkt;
	struct aq_txring *txring;

	if ((ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING)
		return;

	sc = ifp->if_softc;

	if (sc->sc_link_speed == 0)
		return;

	txring = &sc->sc_txring[0];	// select TX ring

 printf("%s:%d: ringidx=%d, HEAD_PTR=%lu\n", __func__, __LINE__, txring->ring_index, AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_ADR(txring->ring_index), TX_DMA_DESC_DHD_MASK));

	for (npkt = 0; ; npkt++) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		if (txring->ring_nfree <= 0) {
			/* no tx descriptor now... */
			ifp->if_flags |= IFF_OACTIVE;
			aprint_debug_dev(sc->sc_dev, "TX descriptor is full\n");
			break;
		}

		IFQ_DEQUEUE(&ifp->if_snd, m);

		if (aq_encap_txring(sc, txring, &m) != 0) {
			/* too many mbuf chains? */
			ifp->if_flags |= IFF_OACTIVE;
			aprint_error_dev(sc->sc_dev,
			    "TX descriptor is full. dropping packet\n");
			m_freem(m);
			ifp->if_oerrors++;
			break;
		}

		/* Pass the packet to any BPF listeners */
		bpf_mtap(ifp, m, BPF_D_OUT);
	}

	if (npkt) {
		/* start TX DMA */
		aq_txring_start(sc, txring);
		ifp->if_timer = 5;
	}
}

static void
aq_stop(struct ifnet *ifp, int disable)
{
	struct aq_softc *sc = ifp->if_softc;
	int i;

	//XXX: need lock
	printf("%s:%d\n", __func__, __LINE__);

	aq_if_disable_intr(sc);

	for (i = 0; i < AQ_TXRING_NUM; i++) {
		aq_txring_init(sc, &sc->sc_txring[i], false);
	}
	for (i = 0; i < AQ_RXRING_NUM; i++) {
		aq_rxring_init(sc, &sc->sc_rxring[i], false);
	}

	aq_hw_reset(sc);
	ifp->if_timer = 0;

	//XXX

	callout_stop(&sc->sc_tick_ch);

	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
}

static void
aq_watchdog(struct ifnet *ifp)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	//XXX
}

static int
aq_ioctl(struct ifnet *ifp, unsigned long cmd, void *data)
{
	struct aq_softc *sc __unused;
	struct ifreq *ifr __unused;
	int error, s;

	sc = (struct aq_softc *)ifp->if_softc;
	ifr = (struct ifreq *)data;
	error = 0;

	//XXX: need lock
	s = splnet();
	switch (cmd) {
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		//XXX
		printf("%s:%d: XXX: cmd=%08lx(SIOCADDMULTI/SIOCDELMULTI)\n", __func__, __LINE__, cmd);
		error = 0;
		break;
	default:
		error = ether_ioctl(ifp, cmd, data);
		break;
	}
	if (error == ENETRESET) {
		if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
		    (IFF_UP | IFF_RUNNING))
			aq_iff(sc);
		error = 0;
	}

	splx(s);
	// XXX: unlock

	return error;
}


MODULE(MODULE_CLASS_DRIVER, if_aq, "pci");

#ifdef _MODULE
#include "ioconf.c"
#endif

static int
if_aq_modcmd(modcmd_t cmd, void *opaque)
{
	int error = 0;

	switch (cmd) {
	case MODULE_CMD_INIT:
#ifdef _MODULE
		error = config_init_component(cfdriver_ioconf_if_aq,
		    cfattach_ioconf_if_aq, cfdata_ioconf_if_aq);
#endif
		return error;
	case MODULE_CMD_FINI:
#ifdef _MODULE
		error = config_fini_component(cfdriver_ioconf_if_aq,
		    cfattach_ioconf_if_aq, cfdata_ioconf_if_aq);
#endif
		return error;
	default:
		return ENOTTY;
	}
}
