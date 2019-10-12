//
// TODO
//	multicast
//	ioctl
//	lock
//	hardware offloading
//	tuning
//	interrupt moderation
//	rss
//	msix
//	vlan
//	cleanup source
//	fw1x (revision A0)
//

//#define XXX_FORCE_32BIT_PA
//#define XXX_DEBUG_PMAP_EXTRACT
#undef USE_CALLOUT_TICK
#define XXX_INTR_DEBUG
//#define XXX_RXINTR_DEBUG
//#define XXX_DUMP_RX_COUNTER
//#define XXX_DUMP_RX_MBUF
//#define XXX_DUMP_MACTABLE

//
// terminology
//
//	MPI = MAC PHY INTERFACE?
//	RPO = RX Protocol Offloading?
//	TPO = TX Protocol Offloading?
//


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
#include <sys/cpu.h>
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

#ifdef XXX_DEBUG_PMAP_EXTRACT
#include <uvm/uvm_extern.h>
#include <uvm/uvm.h>
#endif

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_ether.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

#define CONFIG_LRO_ENABLE	0
#define CONFIG_RSS_ENABLE	0



#define HW_ATL_RSS_HASHKEY_SIZE			40
#define HW_ATL_RSS_INDIRECTION_TABLE_MAX	64


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

// msix bitmap */
#define AQ_INTR_STATUS				0x2000	/* intr status */
#define AQ_INTR_STATUS_CLR			0x2050	/* intr status clear */
#define AQ_INTR_MASK				0x2060	/* intr mask set */
#define AQ_INTR_MASK_CLR			0x2070	/* intr mask clear */
#define AQ_INTR_AUTOMASK			0x2090

#define AQ_INTR_IRQ_MAP_ADR(i)			(0x2100 + ((i) / 2) * 4)
#define AQ_INTR_IRQ_MAP_TX_ADR(tx)		AQ_INTR_IRQ_MAP_ADR(tx)
#define AQ_INTR_IRQ_MAP_TX_MSK(tx)		(__BITS(28,24) >> (((tx) & 1) * 8))
#define AQ_INTR_IRQ_MAP_TX_EN_MSK(tx)		(__BIT(31)     >> (((tx) & 1) * 8))
#define AQ_INTR_IRQ_MAP_RX_ADR(rx)		AQ_INTR_IRQ_MAP_ADR(rx)
#define AQ_INTR_IRQ_MAP_RX_MSK(rx)		(__BITS(12,8)  >> (((rx) & 1) * 8))
#define AQ_INTR_IRQ_MAP_RX_EN_MSK(rx)		(__BIT(15)     >> (((rx) & 1) * 8))

#define AQ_GEN_INTR_MAP_ADR(i)			(0x2180 + (i) * 0x4)
#define  HW_ATL_B0_ERR_INT			8

#define AQ_INTR_CTRL				0x2300
#define  AQ_INTR_CTRL_IRQMODE			__BITS(1,0)
#define  AQ_INTR_CTRL_IRQMODE_LEGACY		0
#define  AQ_INTR_CTRL_IRQMODE_MSI		1
#define  AQ_INTR_CTRL_IRQMODE_MSIX		2
#define  AQ_INTR_CTRL_MULTIVEC			__BIT(2)	//?
#define  AQ_INTR_CTRL_AUTO_MASK			__BIT(5)
#define  AQ_INTR_CTRL_CLR_ON_READ		__BIT(7)
#define  AQ_INTR_CTRL_RESET_DIS			__BIT(29)
#define  AQ_INTR_CTRL_RESET_IRQ			__BIT(31)

//#define ITR_REG_RES_DSBL_ADR			AQ_INTR_CTRL
//#define ITR_RES_ADR				AQ_INTR_CTRL

#define MIF_POWER_GATING_ENABLE_CONTROL_ADR	0x32a8

#define MPI_RESETCTRL_ADR			0x4000
#define  MPI_RESETCTRL_RESET_DIS		__BIT(29)

#define RX_SYSCONTROL_ADR			0x5000
#define  RPB_DMA_SYS_LOOPBACK			__BIT(6)
#define  RPF_TPO_RPF_SYS_LOOPBACK		__BIT(8)
#define  RX_REG_RESET_DIS			__BIT(29)

#define HW_ATL_RX_TCP_RSS_HASH			0x5040

#define RPFL2BC_EN_ADR				0x5100
#define  RPFL2BC_EN				__BIT(0)
#define  RPFL2BC_PROMISC_MODE			__BIT(3)
#define  RPFL2BC_ACT_MSK			__BITS(12,14)
#define  RPFL2BC_THRESH_MSK			__BITS(31,16)

#define RPFL2UC_DAFLSW_ADR(idx)			(0x5110 + (idx) * 0x8)
#define RPFL2UC_DAFMSW_ADR(idx)			(0x5114 + (idx) * 0x8)
#define  RPFL2UC_DAFMSW_MACADDR_HI		__BITS(15,0)
#define  RPFL2UC_DAFMSW_ACTF			__BITS(18,16)
#define  RPFL2UC_DAFMSW_EN			__BIT(31)
#define AQ_HW_MAC			0	/* own address */
#define AQ_HW_MAC_MIN			1
#define AQ_HW_MAC_MAX			33

#define RX_FLR_MCST_FLR_ADR(idx)		(0x5250 + (idx) * 0x4)
#define RX_FLR_MCST_FLR_MSK_ADR			0x5270

#define RPF_VL_ACCEPT_UNTAGGED_MODE_ADR		0x5280
#define  RPF_VL_PROMISC_MODE_MSK		__BIT(1)
#define  RPF_VL_ACCEPT_UNTAGGED_MODE_MSK	__BIT(2)
#define  RPF_VL_UNTAGGED_ACT_MSK		__BITS(5,3)
#define   RPF_VL_UNTAGGED_ACT_DISCARD		0
#define   RPF_VL_UNTAGGED_ACT_HOST		1
#define   RPF_VL_UNTAGGED_ACT_MANAGEMENT	2
#define   RPF_VL_UNTAGGED_ACT_HOST_MANAGEMENT	3
#define   RPF_VL_UNTAGGED_ACT_WOL		4




#define RPF_VL_TPID_ADR				0x5284
#define  RPF_VL_TPID_OUTER_MSK			__BITS(31,16)
#define  RPF_VL_TPID_INNER_MSK			__BITS(15,0)

#define RPF_ET_ENF_ADR(n)			(0x5300 + (n) * 0x4)
#define  RPF_ET_ENF_MSK				__BIT(31)
#define  RPF_ET_UPFEN_MSK			__BIT(30)
#define  RPF_ET_RXQFEN_MSK			__BIT(29)
#define  RPF_ET_UPF_MSK				__BITS(28,26)
#define  RPF_ET_RXQF_MSK			__BITS(24,20)
#define  RPF_ET_MNG_RXQF_MSK			__BIT(19)
#define  RPF_ET_ACTF_MSK			__BITS(18,16)
#define  RPF_ET_VALF_MSK			__BITS(15,0)



#define HW_ATL_RPF_L3_L4_ENF_ADR(f)		(0x5380 + (f) * 0x4)
#define  HW_ATL_RPF_L3_L4_ENF_MSK		__BIT(31)
#define  HW_ATL_RPF_L4_PROTF_EN_MSK		__BIT(25)
#define  HW_ATL_RPF_L3_L4_RXQF_EN_MSK		__BIT(23)
#define  HW_ATL_RPF_L3_L4_ACTF_MSK		__BITS(16,18)
#define   L2_FILTER_ACTION_DISCARD			0
#define   L2_FILTER_ACTION_HOST				1
#define  HW_ATL_RPF_L3_L4_RXQF_MSK		__BITS(12,8)
#define  HW_ATL_RPF_L4_PROTF_MSK		__BITS(2,0)


#define RX_FLR_RSS_CONTROL1_ADR			0x54c0

#define RPF_RPB_RX_TC_UPT_ADR			0x54c4
#define RPF_RPB_RX_TC_UPT_MASK(tc)		(0x00000007 << ((tc) * 4))

#define RPF_RSS_REDIR_ADDR_ADR			0x54e0
#define  RPF_RSS_REDIR_ADDR_MSK			__BITS(3,0)
#define  RPF_RSS_REDIR_WR_EN			__BIT(4)


#define RPF_RSS_REDIR_WR_DATA_ADR		0x54e4
#define  RPF_RSS_REDIR_WR_DATA_MSK		__BITS(15,0)

#define RPO_IPV4_ADR				0x5580
#define  RPO_IPV4_CHK_EN			__BIT(1)
#define  RPO_IPV4_L4_CHECK_EN			__BIT(0)	/* TCP, UDP */

#define RPO_LRO_EN_ADR				0x5590
#define RPO_LRO_QSES_LMT_ADR			0x5594
#define  RPO_LRO_QSES_LMT_MSK			__BITS(13,12)
#define RPO_LRO_TOT_DSC_LMT_ADR			0x5594
#define  RPO_LRO_TOT_DSC_LMT_MSK		__BITS(6,5)
#define RPO_LRO_PTOPT_EN_ADR			0x5594
#define  RPO_LRO_PTOPT_EN_MSK			__BIT(15)
#define RPO_LRO_PKT_MIN_ADR			0x5594
#define  RPO_LRO_PKT_MIN_MSK			__BITS(4,0)
#define RPO_LRO_RSC_MAX_ADR			0x5598
#define RPO_LRO_LDES_MAX_ADR(i)			(0x55a0 + (i / 8) * 4)
#define  RPO_LRO_LDES_MAX_MSK(i)		(0x00000003 << ((i & 7) * 4))
#define RPO_LRO_TB_DIV_ADR			0x5620
#define  RPO_LRO_TB_DIV_MSK			__BITS(20,31)
#define RPO_LRO_INA_IVAL_ADR			0x5620
#define  RPO_LRO_INA_IVAL_MSK			__BITS(10,19)
#define RPO_LRO_MAX_IVAL_ADR			0x5620
#define  RPO_LRO_MAX_IVAL_MSK			__BITS(9,0)

#define RPB_RPF_RX_ADR				0x5700
#define  RPB_RPF_RX_TC_MODE			__BIT(8)
#define  RPB_RPF_RX_FC_MODE			__BITS(5,4)
#define  RPB_RPF_RX_BUF_EN			__BIT(0)


#define RPB_RXBBUF_SIZE_ADR(n)			(0x5710 + (n) * 0x10)
#define  RPB_RXBBUF_SIZE_MSK			__BITS(8,0)

#define RPB_RXB_XOFF_ADR(n)			(0x5714 + (n) * 0x10)
#define  RPB_RXB_XOFF_EN			__BIT(31)
#define  RPB_RXB_XOFF_THRESH_HI			__BITS(29,16)
#define  RPB_RXB_XOFF_THRESH_LO			__BITS(13,0)




#define RX_DMA_DESC_CACHE_INIT_ADR		0x5a00
#define  RX_DMA_DESC_CACHE_INIT_MSK		__BIT(0)

#define RX_DMA_INT_DESC_WRWB_EN_ADR		0x05a30
#define  RX_DMA_INT_DESC_WRWB_EN		__BIT(2)
#define  RX_DMA_INT_DESC_MODERATE_EN		__BIT(3)

#define RX_INTR_MODERATION_CTL_ADR(n)		(0x5a40 + (n) * 0x4)

#define RX_DMA_DESC_BASE_ADDRLSW_ADR(n)		(0x5b00 + (n) * 0x20)
#define RX_DMA_DESC_BASE_ADDRMSW_ADR(n)		(0x5b04 + (n) * 0x20)
#define RX_DMA_DESC_LEN_ADR(n)			(0x5b08 + (n) * 0x20)
#define  RX_DMA_DESC_LEN_MSK			__BITS(12,3)	/* RXD_NUM/8 */
#define  RX_DMA_DESC_RESET			__BIT(25)
#define  RX_DMA_DESC_HEADER_SPLIT		__BIT(28)
#define  RX_DMA_DESC_VLAN_STRIP			__BIT(29)
#define  RX_DMA_DESC_ENABLE			__BIT(31)

#define RX_DMA_DESC_HEAD_PTR_ADR(n)		(0x5b0c + (n) * 0x20)
#define  RX_DMA_DESC_HEAD_PTR_MSK		__BITS(12,0)
#define RX_DMA_DESC_TAIL_PTR_ADR(n)		(0x5b10 + (n) * 0x20)

#define RX_DMA_DESC_BUFSIZE_ADR(n)		(0x5b18 + (n) * 0x20)
#define  RX_DMA_DESC_BUFSIZE_DATA_MSK		__BITS(4,0)
#define  RX_DMA_DESC_BUFSIZE_HDR_MSK		__BITS(12,8)

#define RPF_RSS_KEY_ADDR_ADR			0x54d0
#define  RPF_RSS_KEY_ADDR_MSK			__BITS(4,0)
#define  RPF_RSS_KEY_WR_EN			__BIT(5)


#define RPF_RSS_KEY_WR_DATA_ADR			0x54d4


#define RDM_DCAD_ADR(n)				(0x6100 + (n) * 0x4)
#define  RDM_DCAD_CPUID_MSK			__BITS(7,0)
#define  RDM_DCAD_PAYLOAD_EN			__BIT(29)
#define  RDM_DCAD_HEADER_EN			__BIT(30)
#define  RDM_DCAD_DESC_EN			__BIT(31)

#define RX_DMA_DCA_ADR				0x6180
#define  RX_DMA_DCA_EN				__BIT(31)
#define  RX_DMA_DCA_MODE			__BITS(3,0)

//counters
#define RX_DMA_GOOD_PKT_COUNTERLSW		0x6800
#define RX_DMA_GOOD_OCTET_COUNTERLSW		0x6808
#define RX_DMA_DROP_PKT_CNT_ADR			0x6818
#define RX_DMA_COALESCED_PKT_CNT_ADR		0x6820

#define TX_SYSCONTROL_ADR			0x7000
#define  TPB_DMA_SYS_LOOPBACK			__BIT(6)
#define  TPB_TPO_PKT_SYS_LOOPBACK		__BIT(7)
#define  TX_REG_RESET_DIS			__BIT(29)

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


#define TPO_IPV4_ADR				0x7800
#define  TPO_IPV4_CHK_EN			__BIT(1)
#define  TPO_IPV4_L4_CHECK_EN			__BIT(0)	/* TCP,UDP */

#define TDM_LSO_EN_ADR				0x7810


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

#define TPB_TXBBUF_SIZE_ADR(buffer)		(0x7910 + (buffer) * 0x10)
#define  TPB_TXBBUF_SIZE_MSK			__BITS(7,0)
#define TPB_TXB_THRESH_ADR(buffer)		(0x7914 + (buffer) * 0x10)
#define  TPB_TXB_THRESH_HI			__BITS(16,28)
#define  TPB_TXB_THRESH_LO			__BITS(12,0)

#define AQ_HW_TX_DMA_TOTAL_REQ_LIMIT_ADR	0x7b20
#define TX_DMA_INT_DESC_WRWB_EN_ADR		0x7b40
#define  TX_DMA_INT_DESC_WRWB_EN		__BIT(1)
#define  TX_DMA_INT_DESC_MODERATE_EN		__BIT(4)

#define TX_DMA_DESC_BASE_ADDRLSW_ADR(n)		(0x7c00 + (n) * 0x40)
#define TX_DMA_DESC_BASE_ADDRMSW_ADR(n)		(0x7c04 + (n) * 0x40)
#define TX_DMA_DESC_LEN_ADR(n)			(0x7c08 + (n) * 0x40)
#define  TX_DMA_DESC_LEN_MSK			__BITS(12, 3)	/* TXD_NUM/8 */
#define  TX_DMA_DESC_ENABLE			__BIT(31)
#define TX_DMA_DESC_HEAD_PTR_ADR(n)		(0x7c0c + (n) * 0x40)	/* index of desc */
#define  TX_DMA_DESC_HEAD_PTR_MSK		__BITS(12,0)
#define TX_DMA_DESC_TAIL_PTR_ADR(n)		(0x7c10 + (n) * 0x40)	/* index of desc */

#define TX_DMA_DESC_WRWB_THRESH_ADR(n)		(0x7c18 + (n) * 0x40)
#define  TX_DMA_DESC_WRWB_THRESH		__BITS(14,8)

#define TDM_DCAD_ADR(n)				(0x8400 + (n) * 0x4)
#define  TDM_DCAD_CPUID_MSK			__BITS(7,0)
#define  TDM_DCAD_CPUID_EN			__BIT(31)

#define TDM_DCA_ADR				0x8480
#define  TDM_DCA_EN				__BIT(31)
#define  TDM_DCA_MODE				__BITS(3,0)

#define TX_INTR_MODERATION_CTL_ADR(n)		(0x8980 + (n) * 0x4)

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
	FW_BOOT_MODE_UNKNOWN = 0,
	FW_BOOT_MODE_FLB,
	FW_BOOT_MODE_RBL_FLASH,
	FW_BOOT_MODE_RBL_HOST_BOOTLOAD
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

#define AQ_READ_REG_BIT(sc, reg, mask)				\
	__SHIFTOUT(AQ_READ_REG(sc, reg), mask)

#define AQ_WRITE_REG_BIT(sc, reg, mask, val)			\
	do {							\
		uint32_t _v;					\
		_v = AQ_READ_REG((sc), (reg));			\
		_v &= ~(mask);					\
		if ((val) != 0)					\
			_v |= __SHIFTIN((val), (mask));		\
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
	AQ_LINK_NONE	= 0,
	AQ_LINK_100M	= (1 << 0),
	AQ_LINK_1G	= (1 << 1),
	AQ_LINK_2G5	= (1 << 2),
	AQ_LINK_5G	= (1 << 3),
	AQ_LINK_10G	= (1 << 4)
} aq_fw_link_speed_t;
#define AQ_LINK_ALL	(AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | \
			 AQ_LINK_5G | AQ_LINK_10G )
#define AQ_LINK_AUTO	AQ_LINK_ALL

typedef enum aq_fw_link_fc {
	AQ_FW_FC_NONE = 0,
	AQ_FW_FC_ENABLE_RX = __BIT(0),
	AQ_FW_FC_ENABLE_TX = __BIT(1),
	AQ_FW_FC_ENABLE_ALL = (AQ_FW_FC_ENABLE_RX | AQ_FW_FC_ENABLE_TX)
} aq_fw_link_fc_t;

typedef enum aq_fw_eee {
	AQ_FW_EEE_NONE = 0,
	AQ_FW_EEE_ENABLE = 1
} aq_fw_link_eee_t;

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
#define RXDESC_TYPE_RSS			__BITS(3,0)
#define RXDESC_TYPE_PKTTYPE		__BITS(4,11)
#define RXDESC_TYPE_RDM_ERR		__BIT(12)
#define RXDESC_TYPE_RESERVED		__BITS(13,18)
#define RXDESC_TYPE_CNTL		__BITS(19,20)
#define RXDESC_TYPE_SPH			__BIT(21)
#define RXDESC_TYPE_HDR_LEN		__BITS(22,31)
	uint32_t rss_hash;
	uint16_t status;
#define RXDESC_STATUS_DD		__BIT(0)
#define RXDESC_STATUS_EOP		__BIT(1)
#define RXDESC_STATUS_MAC_DMA_ERR	__BIT(2)
#define RXDESC_STATUS_STAT		__BITS(2,5)
#define RXDESC_STATUS_ESTAT		__BITS(6,11)
#define RXDESC_STATUS_RSC_CNT		__BITS(12,15)
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

/* hardware restrictions */
#define AQ_RINGS_MAX	32
#define AQ_RXD_MIN	32
#define AQ_TXD_MIN	32
#define AQ_RXD_MAX	8184	/* = 0x1ff8 */
#define AQ_TXD_MAX	8184	/* = 0x1ff8 */

/* configuration for this driver */
#define AQ_TXRING_NUM	1
#define AQ_RXRING_NUM	1
#define AQ_TXD_NUM	2048	/* per ring. must be 8*n */
#define AQ_RXD_NUM	2048	/* per ring. must be 8*n */

#define LINKUP_IRQ	0	/* XXX: shared with ring[0] */


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
	unsigned int ring_prodidx;
	unsigned int ring_considx;
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
	unsigned int ring_readidx;
};

struct aq_softc;
struct aq_firmware_ops {
	int (*reset)(struct aq_softc *);
	int (*set_mode)(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
	    aq_fw_link_speed_t, aq_fw_link_fc_t, aq_fw_link_eee_t);
	int (*get_mode)(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
	    aq_fw_link_speed_t *, aq_fw_link_fc_t *, aq_fw_link_eee_t *);
	int (*get_stats)(struct aq_softc *, aq_hw_stats_s_t *);
	int (*led_control)(struct aq_softc *, uint32_t);
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
	int sc_txringnum;
	int sc_rxringnum;
	int sc_ringnum;	/* = MAX(sc_txringnum, sc_rxringnum) */

	pci_chipset_tag_t sc_pc;
	pcitag_t sc_pcitag;
	uint16_t sc_product;
	uint16_t sc_revision;

	kmutex_t sc_mutex;

	struct aq_firmware_ops *sc_fw_ops;
	uint64_t sc_fw_caps;
	enum aq_media_type sc_media_type;
	aq_fw_link_speed_t sc_available_rates;

	aq_fw_link_speed_t sc_link_rate;
	aq_fw_link_fc_t sc_link_fc;
	aq_fw_link_eee_t sc_link_eee;

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

	bool sc_lro_enable;
	bool sc_rss_enable;

	int sc_media_active;

#ifdef USE_CALLOUT_TICK
	callout_t sc_tick_ch;
#endif
	struct ethercom sc_ethercom;
	struct ether_addr sc_enaddr;
	struct ifmedia sc_media;
	unsigned short sc_if_flags;	/* last if_flags */

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
static int aq_ifflags_cb(struct ethercom *);
static int aq_init(struct ifnet *);
static void aq_start(struct ifnet *);
static void aq_stop(struct ifnet *, int);
static void aq_watchdog(struct ifnet *);
static int aq_ioctl(struct ifnet *, unsigned long, void *);

static int aq_set_linkmode(struct aq_softc *);

static int aq_txring_alloc(struct aq_softc *, struct aq_txring *);
static void aq_txring_free(struct aq_softc *, struct aq_txring *);
static int aq_rxring_alloc(struct aq_softc *, struct aq_rxring *);
static void aq_rxring_free(struct aq_softc *, struct aq_rxring *);

#ifdef  XXX_INTR_DEBUG
static int aq_tx_intr_poll(struct aq_txring *);
static int aq_rx_intr_poll(struct aq_rxring *);
#endif
static int aq_tx_intr(struct aq_txring *);
static int aq_rx_intr(struct aq_rxring *);

static int fw1x_reset(struct aq_softc *);
static int fw1x_set_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
    aq_fw_link_speed_t, aq_fw_link_fc_t, aq_fw_link_eee_t);
static int fw1x_get_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
    aq_fw_link_speed_t *, aq_fw_link_fc_t *, aq_fw_link_eee_t *);
static int fw1x_get_stats(struct aq_softc *, aq_hw_stats_s_t *);
static int fw2x_reset(struct aq_softc *);
static int fw2x_set_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
    aq_fw_link_speed_t, aq_fw_link_fc_t, aq_fw_link_eee_t);
static int fw2x_get_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
    aq_fw_link_speed_t *, aq_fw_link_fc_t *, aq_fw_link_eee_t *);
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
	aq_fw_link_speed_t aq_available_rates;
} aq_products[] = {
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC107,
	  "Aquantia AQC107 10 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC108,
	  "Aquantia AQC108 5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | AQ_LINK_5G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC109,
	  "Aquantia AQC109 2.5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC111,
	  "Aquantia AQC111 5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | AQ_LINK_5G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC112,
	  "Aquantia AQC112 2.5 Gigabit Network Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC107S,
	  "Aquantia AQC107S 10 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC108S,
	  "Aquantia AQC108S 5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | AQ_LINK_5G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC109S,
	  "Aquantia AQC109S 2.5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC111S,
	  "Aquantia AQC111S 5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | AQ_LINK_5G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_AQC112S,
	  "Aquantia AQC112S 2.5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_D107,
	  "Aquantia D107 10 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_ALL
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_D108,
	  "Aquantia D108 5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | AQ_LINK_5G
	},
	{ PCI_VENDOR_AQUANTIA, PCI_PRODUCT_AQUANTIA_D109,
	  "Aquantia D109 2.5 Gigabit Ethernet Adapter",
	  AQ_MEDIA_TYPE_TP, AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5
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
		AQ_WRITE_REG_BIT(sc, HW_ATL_MIF_CMD, HW_ATL_MIF_CMD_EXECUTE, 1);
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
aq_get_linkmode(struct aq_softc *sc, aq_fw_link_speed_t *speed, aq_fw_link_fc_t *fc, aq_fw_link_eee_t *eee)
{
	aq_hw_fw_mpi_state_e_t mode;
	aq_fw_link_speed_t cur_speed;
	aq_fw_link_fc_t cur_fc;
	aq_fw_link_eee_t cur_eee;
	int error = 0;

	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->get_mode != NULL) {
		error = sc->sc_fw_ops->get_mode(sc,
		    &mode, &cur_speed, &cur_fc, &cur_eee);
	} else {
		aprint_error_dev(sc->sc_dev, "get_mode() not supported by F/W\n");
		return ENOTSUP;
	}
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "get_mode() failed, error %d\n", error);
		return error;
	}
	if (mode != MPI_INIT)
		return -1;

	if (speed != NULL)
		*speed = cur_speed;
	if (fc != NULL)
		*fc = cur_fc;
	if (eee != NULL)
		*eee = cur_eee;

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

#ifdef XXX_DUMP_MACTABLE
static void
aq_dump_mactable(struct aq_softc *sc)
{
	int i;
	uint32_t h, l;

	for (i = 0; i <= AQ_HW_MAC_MAX; i++) {
		l = AQ_READ_REG(sc, RPFL2UC_DAFLSW_ADR(i));
		h = AQ_READ_REG(sc, RPFL2UC_DAFMSW_ADR(i));
		printf("MAC TABLE[%d] %02x:%02x:%02x:%02x:%02x:%02x enable=%d, actf=%ld\n",
		    i,
		    (h >> 8) & 0xff,
		    h & 0xff,
		    (l >> 24) & 0xff,
		    (l >> 16) & 0xff,
		    (l >> 8) & 0xff,
		    l & 0xff,
		    (h & RPFL2UC_DAFMSW_EN) ? 1 : 0,
		    AQ_READ_REG_BIT(sc, RPFL2UC_DAFMSW_ADR(i), RPFL2UC_DAFMSW_ACTF));
	}
}
#endif

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
	AQ_WRITE_REG_BIT(sc, RPFL2UC_DAFMSW_ADR(index), RPFL2UC_DAFMSW_EN, 0);
	AQ_WRITE_REG(sc, RPFL2UC_DAFLSW_ADR(index), l);
	AQ_WRITE_REG_BIT(sc, RPFL2UC_DAFMSW_ADR(index), RPFL2UC_DAFMSW_MACADDR_HI, h);
	AQ_WRITE_REG_BIT(sc, RPFL2UC_DAFMSW_ADR(index), RPFL2UC_DAFMSW_ACTF, 1);
	AQ_WRITE_REG_BIT(sc, RPFL2UC_DAFMSW_ADR(index), RPFL2UC_DAFMSW_EN, 1);

	return 0;
}


static void
aq_mediastatus_update(struct aq_softc *sc)
{
	sc->sc_media_active = 0;

	if (sc->sc_link_fc & AQ_FW_FC_ENABLE_RX)
		sc->sc_media_active |= IFM_ETH_RXPAUSE;
	if (sc->sc_link_fc & AQ_FW_FC_ENABLE_TX)
		sc->sc_media_active |= IFM_ETH_TXPAUSE;

	switch (sc->sc_link_rate) {
	case AQ_LINK_100M:
		sc->sc_media_active |= IFM_100_TX | IFM_FDX;
		break;
	case AQ_LINK_1G:
		sc->sc_media_active |= IFM_1000_T | IFM_FDX;
		break;
	case AQ_LINK_2G5:
		sc->sc_media_active |= IFM_2500_T | IFM_FDX;
		break;
	case AQ_LINK_5G:
		sc->sc_media_active |= IFM_5000_T | IFM_FDX;
		break;
	case AQ_LINK_10G:
		sc->sc_media_active |= IFM_10G_T | IFM_FDX;
		break;
	default:
		sc->sc_media_active |= IFM_NONE;
		break;
	}
}

static void
aq_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct aq_softc *sc = ifp->if_softc;

	ifmr->ifm_active = IFM_ETHER;
	ifmr->ifm_status = IFM_AVALID;

	if (sc->sc_link_rate != AQ_LINK_NONE)
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
		sc->sc_link_rate = AQ_LINK_AUTO;
		break;
	case IFM_NONE:
		sc->sc_link_rate = AQ_LINK_NONE;
		break;
	case IFM_100_TX:
		sc->sc_link_rate = AQ_LINK_100M;
		break;
	case IFM_1000_T:
		sc->sc_link_rate = AQ_LINK_1G;
		break;
	case IFM_2500_T:
		sc->sc_link_rate = AQ_LINK_2G5;
		break;
	case IFM_5000_T:
		sc->sc_link_rate = AQ_LINK_5G;
		break;
	case IFM_10G_T:
		sc->sc_link_rate = AQ_LINK_10G;
		break;
	default:
		aprint_error_dev(sc->sc_dev, "unknown media: 0x%X\n", IFM_SUBTYPE(sc->sc_media.ifm_media));
		return 0;
	}

	if (sc->sc_media.ifm_media & IFM_FLOW)
		sc->sc_link_fc = AQ_FW_FC_ENABLE_ALL;
	else
		sc->sc_link_fc = AQ_FW_FC_NONE;

	/* re-initialize hardware with new parameters */
	aq_set_linkmode(sc);

	return 0;
}

static void
aq_initmedia(struct aq_softc *sc)
{
#define IFMEDIA_ETHER_ADD(sc, media)	\
	ifmedia_add(&(sc)->sc_media, IFM_ETHER | media, 0, NULL);

	IFMEDIA_ETHER_ADD(sc, IFM_NONE);
	if (sc->sc_available_rates & AQ_LINK_100M) {
		IFMEDIA_ETHER_ADD(sc, IFM_100_TX);
		IFMEDIA_ETHER_ADD(sc, IFM_100_TX | IFM_FLOW);
	}
	if (sc->sc_available_rates & AQ_LINK_1G) {
		IFMEDIA_ETHER_ADD(sc, IFM_1000_T);
		IFMEDIA_ETHER_ADD(sc, IFM_1000_T | IFM_FLOW);
	}
	if (sc->sc_available_rates & AQ_LINK_2G5) {
		IFMEDIA_ETHER_ADD(sc, IFM_2500_T);
		IFMEDIA_ETHER_ADD(sc, IFM_2500_T | IFM_FLOW);
	}
	if (sc->sc_available_rates & AQ_LINK_5G) {
		IFMEDIA_ETHER_ADD(sc, IFM_5000_T);
		IFMEDIA_ETHER_ADD(sc, IFM_5000_T | IFM_FLOW);
	}
	if (sc->sc_available_rates & AQ_LINK_10G) {
		IFMEDIA_ETHER_ADD(sc, IFM_10G_T);
		IFMEDIA_ETHER_ADD(sc, IFM_10G_T | IFM_FLOW);
	}
	IFMEDIA_ETHER_ADD(sc, IFM_AUTO);
	IFMEDIA_ETHER_ADD(sc, IFM_AUTO | IFM_FLOW);

	/* default media */
#if 0
	ifmedia_set(&sc->sc_media, IFM_ETHER | IFM_AUTO | IFM_FLOW);
	sc->sc_link_rate = AQ_LINK_AUTO;
	sc->sc_link_fc = AQ_FW_FC_ENABLE_ALL;
#else
	ifmedia_set(&sc->sc_media, IFM_ETHER | IFM_AUTO);
	sc->sc_link_rate = AQ_LINK_AUTO;
	sc->sc_link_fc = AQ_FW_FC_NONE;
#endif
}

static void
global_software_reset(struct aq_softc *sc)
{
	uint32_t v;

	AQ_WRITE_REG_BIT(sc, RX_SYSCONTROL_ADR, RX_REG_RESET_DIS, 0);
	AQ_WRITE_REG_BIT(sc, TX_SYSCONTROL_ADR, TX_REG_RESET_DIS, 0);
	AQ_WRITE_REG_BIT(sc, MPI_RESETCTRL_ADR, MPI_RESETCTRL_RESET_DIS, 0);

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
			*mode = FW_BOOT_MODE_RBL_FLASH;
		aprint_debug_dev(sc->sc_dev,
		    "RBL> reset complete! [Flash]\n");
		break;
	case RBL_STATUS_HOST_BOOT:
		if (mode != NULL)
			*mode = FW_BOOT_MODE_RBL_HOST_BOOTLOAD;
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
		*mode = FW_BOOT_MODE_FLB;
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

	aq_fw_bootloader_mode_t mode = FW_BOOT_MODE_UNKNOWN;
	error = mac_soft_reset(sc, &mode);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev, "MAC reset failed: %d\n", error);
		return error;
	}

	switch (mode) {
	case FW_BOOT_MODE_FLB:
		aprint_debug_dev(sc->sc_dev,
		    "FLB> F/W successfully loaded from flash.\n");
		sc->sc_flash_present = true;
		return wait_init_mac_firmware(sc);
	case FW_BOOT_MODE_RBL_FLASH:
		aprint_debug_dev(sc->sc_dev,
		    "RBL> F/W loaded from flash. Host Bootload disabled.\n");
		sc->sc_flash_present = true;
		return wait_init_mac_firmware(sc);
	case FW_BOOT_MODE_UNKNOWN:
		aprint_error_dev(sc->sc_dev,
		    "F/W bootload error: unknown bootloader type\n");
		return ENOTSUP;
	case FW_BOOT_MODE_RBL_HOST_BOOTLOAD:
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
	AQ_WRITE_REG_BIT(sc, AQ_INTR_CTRL, AQ_INTR_CTRL_RESET_DIS, 0);

	/* apply */
	AQ_WRITE_REG_BIT(sc, AQ_INTR_CTRL, AQ_INTR_CTRL_RESET_IRQ, 1);

	/* wait ack 10 times by 1ms */
	WAIT_FOR((AQ_READ_REG(sc, AQ_INTR_CTRL) & AQ_INTR_CTRL_RESET_IRQ) == 0,
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
		 * I don't know which is correct...
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
aq_set_linkmode(struct aq_softc *sc)
{
	int error;

	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->set_mode != NULL) {
		error = sc->sc_fw_ops->set_mode(sc, MPI_INIT,
		    sc->sc_link_rate, sc->sc_link_fc, sc->sc_link_eee);
	} else {
		aprint_error_dev(sc->sc_dev, "set_mode() not supported by F/W\n");
		error = ENOTSUP;
	}
	return error;
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
    aq_fw_link_speed_t speed, aq_fw_link_fc_t fc, aq_fw_link_eee_t eee)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	return -1;
}

static int
fw1x_get_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t *mode,
    aq_fw_link_speed_t *speed, aq_fw_link_fc_t *fc, aq_fw_link_eee_t *eee)
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

	if (speed & AQ_LINK_10G)
		rate |= FW2X_CTRL_RATE_10G;
	if (speed & AQ_LINK_5G)
		rate |= FW2X_CTRL_RATE_5G;
	if (speed & AQ_LINK_2G5)
		rate |= FW2X_CTRL_RATE_2G5;
	if (speed & AQ_LINK_1G)
		rate |= FW2X_CTRL_RATE_1G;
	if (speed & AQ_LINK_100M)
		rate |= FW2X_CTRL_RATE_100M;

	return rate;
}

static int
fw2x_set_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t mode,
    aq_fw_link_speed_t speed, aq_fw_link_fc_t fc, aq_fw_link_eee_t eee)
{
	uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_ADDR);

	switch (mode) {
	case MPI_INIT:
		mpi_ctrl &= ~FW2X_CTRL_RATE_MASK;
		mpi_ctrl |= link_speed_mask_to_fw2x(speed);
		mpi_ctrl &= ~FW2X_CTRL_LINK_DROP;

		mpi_ctrl &= ~FW2X_CTRL_EEE_MASK;
		if (sc->sc_link_eee == AQ_FW_EEE_ENABLE)
			mpi_ctrl |= FW2X_CTRL_EEE_MASK;

		mpi_ctrl &= ~(FW2X_CTRL_PAUSE | FW2X_CTRL_ASYMMETRIC_PAUSE);
		if (sc->sc_link_fc & AQ_FW_FC_ENABLE_RX)
			mpi_ctrl |= FW2X_CTRL_PAUSE;
		if (sc->sc_link_fc & AQ_FW_FC_ENABLE_TX)
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
    aq_fw_link_speed_t *speedp, aq_fw_link_fc_t *fcp, aq_fw_link_eee_t *eeep)
{
	uint64_t mpi_state = AQ_READ64_REG(sc, FW2X_MPI_STATE_ADDR);

	if (mode != NULL) {
		uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_ADDR);
		if (mpi_ctrl & FW2X_CTRL_RATE_MASK)
			*mode = MPI_INIT;
		else
			*mode = MPI_DEINIT;
	}

	aq_fw_link_speed_t speed = AQ_LINK_NONE;
	if (mpi_state & FW2X_CTRL_RATE_10G)
		speed = AQ_LINK_10G;
	else if (mpi_state & FW2X_CTRL_RATE_5G)
		speed = AQ_LINK_5G;
	else if (mpi_state & FW2X_CTRL_RATE_2G5)
		speed = AQ_LINK_2G5;
	else if (mpi_state & FW2X_CTRL_RATE_1G)
		speed = AQ_LINK_1G;
	else if (mpi_state & FW2X_CTRL_RATE_100M)
		speed = AQ_LINK_100M;

	if (speedp != NULL)
		*speedp = speed;

	aq_fw_link_fc_t fc = AQ_FW_FC_NONE;
	if (mpi_state & FW2X_CTRL_PAUSE)
		fc |= AQ_FW_FC_ENABLE_RX;
	if (mpi_state & FW2X_CTRL_ASYMMETRIC_PAUSE)
		fc |= AQ_FW_FC_ENABLE_TX;
	if (fcp != NULL)
		*fcp = fc;

	//XXX: TODO: EEE
	if (eeep != NULL)
		*eeep = 0;

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
	stats->dpc = AQ_READ_REG(sc, RX_DMA_DROP_PKT_CNT_ADR);
	stats->cprc = AQ_READ_REG(sc, RX_DMA_COALESCED_PKT_CNT_ADR);

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
	AQ_WRITE_REG_BIT(sc, TPB_TX_BUF_ADR, TPB_TX_BUF_TC_MODE_EN, 1);

	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG_FIRST_ADR, THM_LSO_TCP_FLAG_FIRST_MSK, 0x0ff6);
	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG_MID_ADR,   THM_LSO_TCP_FLAG_MID_MSK,   0x0ff6);
	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG_LAST_ADR,  THM_LSO_TCP_FLAG_LAST_MSK,  0x0f7f);

	/* Tx interrupts */
	AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_ADR, TX_DMA_INT_DESC_WRWB_EN, 1);
	AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_ADR, TX_DMA_INT_DESC_MODERATE_EN, 0);

	/* misc */
	AQ_WRITE_REG(sc, 0x7040, (sc->sc_features & FEATURES_TPO2) ? __BIT(16) : 0);
	AQ_WRITE_REG_BIT(sc, TDM_DCA_ADR, TDM_DCA_EN, 0);
	AQ_WRITE_REG_BIT(sc, TDM_DCA_ADR, TDM_DCA_MODE, 0);

	AQ_WRITE_REG_BIT(sc, TPB_TX_BUF_ADR, TPB_TX_BUF_SCP_INS_EN, 1);
}

static void
aq_hw_init_rx_path(struct aq_softc *sc)
{
	int i;

	/* Rx TC/RSS number config */
	AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_ADR, RPB_RPF_RX_TC_MODE, 1);

	/* Rx flow control */
	AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_ADR, RPB_RPF_RX_FC_MODE, 1);

	/* RSS Ring selection */
	AQ_WRITE_REG(sc, RX_FLR_RSS_CONTROL1_ADR, sc->sc_rss_enable ? 0xb3333333 : 0);

	/* Multicast filters */
	for (i = AQ_HW_MAC_MIN; i < AQ_HW_MAC_MAX; i++) {
		AQ_WRITE_REG_BIT(sc, RPFL2UC_DAFMSW_ADR(i), RPFL2UC_DAFMSW_EN, 0);
		AQ_WRITE_REG_BIT(sc, RPFL2UC_DAFMSW_ADR(i), RPFL2UC_DAFMSW_ACTF, 1);
	}

	AQ_WRITE_REG(sc, RX_FLR_MCST_FLR_MSK_ADR, 0);
	AQ_WRITE_REG(sc, RX_FLR_MCST_FLR_ADR(0), 0x00010fff);

	/* Vlan filters */
	AQ_WRITE_REG_BIT(sc, RPF_VL_TPID_ADR, RPF_VL_TPID_OUTER_MSK, ETHERTYPE_QINQ);
	AQ_WRITE_REG_BIT(sc, RPF_VL_TPID_ADR, RPF_VL_TPID_OUTER_MSK, ETHERTYPE_VLAN);
	AQ_WRITE_REG_BIT(sc, RPF_VL_ACCEPT_UNTAGGED_MODE_ADR, RPF_VL_PROMISC_MODE_MSK, 1);
	AQ_WRITE_REG_BIT(sc, RPF_VL_ACCEPT_UNTAGGED_MODE_ADR, RPF_VL_ACCEPT_UNTAGGED_MODE_MSK, 1);
	AQ_WRITE_REG_BIT(sc, RPF_VL_ACCEPT_UNTAGGED_MODE_ADR, RPF_VL_UNTAGGED_ACT_MSK, RPF_VL_UNTAGGED_ACT_HOST);

	/* Rx Interrupts */
	AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_ADR, RX_DMA_INT_DESC_WRWB_EN, 1);
	AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_ADR, RX_DMA_INT_DESC_MODERATE_EN, 0);

	/* misc */
	if (sc->sc_features & FEATURES_RPF2)
		AQ_WRITE_REG(sc, HW_ATL_RX_TCP_RSS_HASH, 0x000f0000);	/* XXX: linux:0x000f0000, freebsd:0x00f0001e */
	else
		AQ_WRITE_REG(sc, HW_ATL_RX_TCP_RSS_HASH, 0);

	AQ_WRITE_REG_BIT(sc, RPFL2BC_EN_ADR, RPFL2BC_EN, 1);
	AQ_WRITE_REG_BIT(sc, RPFL2BC_EN_ADR, RPFL2BC_ACT_MSK, 1);
	AQ_WRITE_REG_BIT(sc, RPFL2BC_EN_ADR, RPFL2BC_THRESH_MSK, 0xffff);

	AQ_WRITE_REG_BIT(sc, RX_DMA_DCA_ADR, RX_DMA_DCA_EN, 0);
	AQ_WRITE_REG_BIT(sc, RX_DMA_DCA_ADR, RX_DMA_DCA_MODE, 0);
}

static void
aq_hw_interrupt_moderation_set(struct aq_softc *sc)
{
	int i;

#if 1
	/* moderation off */
	AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_ADR, TX_DMA_INT_DESC_WRWB_EN, 1);
	AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_ADR, TX_DMA_INT_DESC_MODERATE_EN, 0);
	AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_ADR, RX_DMA_INT_DESC_WRWB_EN, 1);
	AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_ADR, RX_DMA_INT_DESC_MODERATE_EN, 0);

	for (i = 0; i < sc->sc_txringnum; i++) {
		AQ_WRITE_REG(sc, TX_INTR_MODERATION_CTL_ADR(i), 0);
		AQ_WRITE_REG(sc, RX_INTR_MODERATION_CTL_ADR(i), 0);
	}
#else
	/* moderation on */
	AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_ADR, TX_DMA_INT_DESC_WRWB_EN, 0);
	AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_ADR, TX_DMA_INT_DESC_MODERATE_EN, 1);
	AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_ADR, RX_DMA_INT_DESC_WRWB_EN, 0);
	AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_ADR, RX_DMA_INT_DESC_MODERATE_EN, 1);

	//XXX: should be configured according to link speed...
	for (i = 0; i < sc->sc_txringnum; i++) {
		AQ_WRITE_REG(sc, TX_INTR_MODERATION_CTL_ADR(i), 0x01ff4f00);
		AQ_WRITE_REG(sc, RX_INTR_MODERATION_CTL_ADR(i), 0x00380600);
	}
#endif
}

static void
aq_hw_qos_set(struct aq_softc *sc)
{
	uint32_t tc = 0;
	uint32_t buff_size;

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
	tc = 0;
	buff_size = AQ_HW_TXBUF_MAX;
	AQ_WRITE_REG_BIT(sc, TPB_TXBBUF_SIZE_ADR(tc), TPB_TXBBUF_SIZE_MSK, buff_size);
	AQ_WRITE_REG_BIT(sc, TPB_TXB_THRESH_ADR(tc), TPB_TXB_THRESH_HI, (buff_size * (1024 / 32) * 66) / 100);
	AQ_WRITE_REG_BIT(sc, TPB_TXB_THRESH_ADR(tc), TPB_TXB_THRESH_LO, (buff_size * (1024 / 32) * 50) / 100);

	/* QoS Rx buf size per TC */
	tc = 0;
	buff_size = AQ_HW_RXBUF_MAX;
	AQ_WRITE_REG_BIT(sc, RPB_RXBBUF_SIZE_ADR(tc), RPB_RXBBUF_SIZE_MSK, buff_size);
	AQ_WRITE_REG_BIT(sc, RPB_RXB_XOFF_ADR(tc), RPB_RXB_XOFF_THRESH_HI, (buff_size * (1024 / 32) * 66) / 100);
	AQ_WRITE_REG_BIT(sc, RPB_RXB_XOFF_ADR(tc), RPB_RXB_XOFF_THRESH_LO, (buff_size * (1024 / 32) * 50) / 100);

	/* QoS 802.1p priority -> TC mapping */
	int i_priority;
	for (i_priority = 0; i_priority < 8; i_priority++) {
		AQ_WRITE_REG_BIT(sc, RPF_RPB_RX_TC_UPT_ADR, RPF_RPB_RX_TC_UPT_MASK(0), i_priority);
	}
}

static void
aq_hw_offload_set(struct aq_softc *sc)
{
	uint32_t i, v;

	/* TX checksums offloads*/
	AQ_WRITE_REG_BIT(sc, TPO_IPV4_ADR, TPO_IPV4_CHK_EN, 1);
	AQ_WRITE_REG_BIT(sc, TPO_IPV4_ADR, TPO_IPV4_L4_CHECK_EN, 1);

	/* RX checksums offloads*/
	AQ_WRITE_REG_BIT(sc, RPO_IPV4_ADR, RPO_IPV4_CHK_EN, 1);
	AQ_WRITE_REG_BIT(sc, RPO_IPV4_ADR, RPO_IPV4_L4_CHECK_EN, 1);

	/* LSO offloads*/
	AQ_WRITE_REG(sc, TDM_LSO_EN_ADR, 0xffffffff);

#define HW_ATL_B0_LRO_RXD_MAX	16
#define HW_ATL_B0_RINGS_MAX	32
	v = (8 < HW_ATL_B0_LRO_RXD_MAX) ? 3 :
	    (4 < HW_ATL_B0_LRO_RXD_MAX) ? 2 :
	    (2 < HW_ATL_B0_LRO_RXD_MAX) ? 1 : 0;

	for (i = 0; i < HW_ATL_B0_RINGS_MAX; i++) {
		AQ_WRITE_REG_BIT(sc, RPO_LRO_LDES_MAX_ADR(i), RPO_LRO_LDES_MAX_MSK(i), v);
	}

	AQ_WRITE_REG_BIT(sc, RPO_LRO_TB_DIV_ADR, RPO_LRO_TB_DIV_MSK, 0x61a);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_INA_IVAL_ADR, RPO_LRO_INA_IVAL_MSK, 0);
	/*
	 * the LRO timebase divider is 5 uS (0x61a),
	 * to get a maximum coalescing interval of 250 uS,
	 * we need to multiply by 50(0x32) to get
	 * the default value 250 uS
	 */

	AQ_WRITE_REG_BIT(sc, RPO_LRO_MAX_IVAL_ADR, RPO_LRO_MAX_IVAL_MSK, 50);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_QSES_LMT_ADR, RPO_LRO_QSES_LMT_MSK, 1);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_TOT_DSC_LMT_ADR, RPO_LRO_TOT_DSC_LMT_MSK, 2);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_PTOPT_EN_ADR, RPO_LRO_PTOPT_EN_MSK, 0);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_PKT_MIN_ADR, RPO_LRO_PKT_MIN_MSK, 10);
	AQ_WRITE_REG(sc, RPO_LRO_RSC_MAX_ADR, 1);
	AQ_WRITE_REG(sc, RPO_LRO_EN_ADR, sc->sc_lro_enable ? 0xffffffff : 0);

}

static int
aq_hw_rss_hash_set(struct aq_softc *sc)
{
	/* RSS */
	uint8_t rss_key[HW_ATL_RSS_HASHKEY_SIZE];
	unsigned int i;
	int error = 0;

	//XXX XXX XXX
	cprng_fast(&rss_key, sizeof(rss_key));
	for (i = 0; i < HW_ATL_RSS_HASHKEY_SIZE; i++) {
		rss_key[i] = i & (sc->sc_rxringnum - 1);
	}

	uint32_t rss_key_dw[HW_ATL_RSS_HASHKEY_SIZE / sizeof(uint32_t)];


	memcpy(rss_key_dw, rss_key, HW_ATL_RSS_HASHKEY_SIZE);

	for (i = 0; i < __arraycount(rss_key_dw); i++) {
		uint32_t key_data = bswap32(rss_key_dw[__arraycount(rss_key_dw) - 1 - i]);

		AQ_WRITE_REG(sc, RPF_RSS_KEY_WR_DATA_ADR, key_data);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_KEY_ADDR_ADR, RPF_RSS_KEY_ADDR_MSK, i);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_KEY_ADDR_ADR, RPF_RSS_KEY_WR_EN, 1);
		WAIT_FOR(AQ_READ_REG_BIT(sc, RPF_RSS_KEY_ADDR_ADR, RPF_RSS_KEY_WR_EN) == 0,
		    1000, 10, &error);
		if (error != 0)
			break;
	}

	return error;
}

static int
aq_hw_rss_set(struct aq_softc *sc)
{
	uint8_t rss_table[HW_ATL_RSS_INDIRECTION_TABLE_MAX];
	uint16_t bitary[(HW_ATL_RSS_INDIRECTION_TABLE_MAX * 3 / sizeof(uint16_t))];
	int error = 0;
	unsigned int i;

	for (i = __arraycount(rss_table); i != 0; i--) {
		rss_table[i] = i & (sc->sc_rxringnum - 1);
	}

	memset(bitary, 0, sizeof(bitary));

	for (i = HW_ATL_RSS_INDIRECTION_TABLE_MAX; i--;) {
		(*(uint32_t *)(bitary + ((i * 3) / 16))) |=
		    ((rss_table[i]) << ((i * 3) & 15));
	}

	for (i = 0; i < __arraycount(bitary); i++) {
		AQ_WRITE_REG_BIT(sc, RPF_RSS_REDIR_WR_DATA_ADR, RPF_RSS_REDIR_WR_DATA_MSK, bitary[i]);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_REDIR_ADDR_ADR, RPF_RSS_REDIR_ADDR_MSK, i);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_REDIR_ADDR_ADR, RPF_RSS_REDIR_WR_EN, 1);
		WAIT_FOR(AQ_READ_REG_BIT(sc, RPF_RSS_REDIR_ADDR_ADR, RPF_RSS_REDIR_WR_EN) == 0,
		    1000, 10, &error);
		if (error != 0)
			break;
	}
	return error;
}

static void
aq_hw_udp_rss_enable(struct aq_softc *sc, bool enable)
{
	if (!enable) {
		/* HW bug workaround:
		 * Disable RSS for UDP using rx flow filter 0.
		 * HW does not track RSS stream for fragmenged UDP,
		 * 0x5040 control reg does not work.
		 */
		AQ_WRITE_REG_BIT(sc, HW_ATL_RPF_L3_L4_ENF_ADR(0), HW_ATL_RPF_L3_L4_ENF_MSK, 1);
		AQ_WRITE_REG_BIT(sc, HW_ATL_RPF_L3_L4_ENF_ADR(0), HW_ATL_RPF_L4_PROTF_EN_MSK, 1);
		AQ_WRITE_REG_BIT(sc, HW_ATL_RPF_L3_L4_ENF_ADR(0), HW_ATL_RPF_L3_L4_RXQF_EN_MSK, 1);
		AQ_WRITE_REG_BIT(sc, HW_ATL_RPF_L3_L4_ENF_ADR(0), HW_ATL_RPF_L3_L4_ACTF_MSK, L2_FILTER_ACTION_HOST);
		AQ_WRITE_REG_BIT(sc, HW_ATL_RPF_L3_L4_ENF_ADR(0), HW_ATL_RPF_L3_L4_RXQF_MSK, 0);
		AQ_WRITE_REG_BIT(sc, HW_ATL_RPF_L3_L4_ENF_ADR(0), HW_ATL_RPF_L4_PROTF_MSK, 0);
	} else {
		AQ_WRITE_REG_BIT(sc, HW_ATL_RPF_L3_L4_ENF_ADR(0), HW_ATL_RPF_L3_L4_ENF_MSK, 0);
	}
}

static void
aq_update_vlan_filters(struct aq_softc *sc)
{
	//XXX: notyet
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
	aq_hw_init_rx_path(sc);

	aq_hw_interrupt_moderation_set(sc);

	aq_set_mac_addr(sc, AQ_HW_MAC, sc->sc_enaddr.ether_addr_octet);
	aq_set_linkmode(sc);

	aq_hw_qos_set(sc);

	/* Enable interrupt */
	AQ_WRITE_REG(sc, AQ_INTR_CTRL, AQ_INTR_CTRL_RESET_DIS);
//	AQ_WRITE_REG(sc, AQ_INTR_CTRL, AQ_INTR_CTRL_RESET_DIS | AQ_INTR_CTRL_CLR_ON_READ);
	AQ_WRITE_REG(sc, AQ_INTR_AUTOMASK, 0xffffffff);

	int msix_mode = 0;	//XXX
	if (msix_mode) {
		AQ_WRITE_REG_BIT(sc, AQ_INTR_CTRL, AQ_INTR_CTRL_IRQMODE, AQ_INTR_CTRL_IRQMODE_MSIX);
	} else {
		AQ_WRITE_REG_BIT(sc, AQ_INTR_CTRL, AQ_INTR_CTRL_IRQMODE, AQ_INTR_CTRL_IRQMODE_MSI);
	}

	AQ_WRITE_REG(sc, AQ_GEN_INTR_MAP_ADR(0),
	    ((HW_ATL_B0_ERR_INT << 24) | (1 << 31)) |
	    ((HW_ATL_B0_ERR_INT << 16) | (1 << 23))
	);

	/* link interrupt */
	AQ_WRITE_REG(sc, AQ_GEN_INTR_MAP_ADR(3), __BIT(7) | LINKUP_IRQ);

	aq_hw_offload_set(sc);

	return 0;
}

static int
aq_if_update_admin_status(struct aq_softc *sc)
{
	aq_fw_link_speed_t rate = AQ_LINK_NONE;
	aq_fw_link_fc_t fc = AQ_FW_FC_NONE;
	aq_fw_link_eee_t eee = AQ_FW_EEE_NONE;
	unsigned int speed;
	int changed = 0;

	aq_get_linkmode(sc, &rate, &fc, &eee);

	if (sc->sc_link_rate != rate)
		changed = 1;
	if (sc->sc_link_fc != fc)
		changed = 1;
	if (sc->sc_link_eee != eee)
		changed = 1;

	if (changed) {
		switch (rate) {
		case AQ_LINK_100M:
			speed = 100;
			break;
		case AQ_LINK_1G:
			speed = 1000;
			break;
		case AQ_LINK_2G5:
			speed = 2500;
			break;
		case AQ_LINK_5G:
			speed = 5000;
			break;
		case AQ_LINK_10G:
			speed = 10000;
			break;
		case AQ_LINK_NONE:
		default:
			speed = 0;
			break;
		}

		if (sc->sc_link_rate == AQ_LINK_NONE) {
			/* link DOWN -> UP */
			aprint_debug_dev(sc->sc_dev, "link UP: speed=%u\n", speed);
		} else if (rate == AQ_LINK_NONE) {
			/* link UP -> DOWN */
			aprint_debug_dev(sc->sc_dev, "link DOWN\n");
		} else {
			aprint_debug_dev(sc->sc_dev, "link changed: speed=%u, fc=0x%x, eee=%x\n", speed, fc, eee);
		}

		sc->sc_link_rate = rate;
		sc->sc_link_fc = fc;
		sc->sc_link_eee = eee;

		/* turn on/off RX Pause in RPB */
		AQ_WRITE_REG_BIT(sc, RPB_RXB_XOFF_ADR(0), RPB_RXB_XOFF_EN, (fc != AQ_FW_FC_NONE) ? 1 : 0);

		aq_mediastatus_update(sc);

		/* update ITR settings according new link speed */
		aq_hw_interrupt_moderation_set(sc);
	}


	if (sc->sc_statistics_enable) {
		int prev = sc->sc_statistics_idx;
		int cur = prev ^ 1;

		sc->sc_fw_ops->get_stats(sc, &sc->sc_statistics[cur]);

#define ADD_DELTA(cur,prev,name,descr)	\
		do {															\
			uint64_t n = (uint32_t)(sc->sc_statistics[cur].name - sc->sc_statistics[prev].name);				\
			/* printf("# %s: %s: %u\n", descr, #name, sc->sc_statistics[cur].name); */					\
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

	return changed;
}

/* allocate and map one DMA blocks */
static int
_alloc_dma(struct aq_softc *sc, bus_size_t size, bus_size_t *sizep,
    void **addrp, bus_dmamap_t *mapp, bus_dma_segment_t *seg)
{
	int nsegs, error;

	if ((error = bus_dmamem_alloc(sc->sc_dmat, size, PAGE_SIZE, 0, seg,
	    1, &nsegs, 0)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to allocate DMA buffer, error=%d\n", error);
		goto fail_alloc;
	}

	if ((error = bus_dmamem_map(sc->sc_dmat, seg, 1, size, addrp,
	    BUS_DMA_COHERENT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to map DMA buffer, error=%d\n", error);
		goto fail_map;
	}

	if ((error = bus_dmamap_create(sc->sc_dmat, size, 1, size, 0,
	    0, mapp)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to create DMA map, error=%d\n", error);
		goto fail_create;
	}

	if ((error = bus_dmamap_load(sc->sc_dmat, *mapp, *addrp, size, NULL,
	    0)) != 0) {
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

#ifdef XXX_DEBUG_PMAP_EXTRACT
	{
		bool ok;
		char *descp = (char *)txring->ring_txdesc;
		paddr_t pa;
		vaddr_t va;


		printf("TX desc size = %lu\n", txring->ring_txdesc_size);
		printf("TX desc DM_SEGS[0] = PA=%08lx\n", txring->ring_txdesc_dmamap->dm_segs[0].ds_addr);

		for (i = 0; (bus_size_t)(PAGE_SIZE * i) < txring->ring_txdesc_size; i++) {
			va = (vaddr_t)descp + PAGE_SIZE * i;
			ok = pmap_extract(pmap_kernel(), va, &pa);
			printf("TX desc VA=%lx, PA=%08lx, ok=%d\n", va, pa, ok);
		}
	}
#endif

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

static inline void
aq_rxring_reset_desc(struct aq_softc *sc, struct aq_rxring *rxring, int idx)
{
	/* refill rxdesc, and sync */
	rxring->ring_rxdesc[idx].read.buf_addr = htole64(rxring->ring_mbufs[idx].dmamap->dm_segs[0].ds_addr);
	rxring->ring_rxdesc[idx].read.hdr_addr = 0;
	bus_dmamap_sync(sc->sc_dmat, rxring->ring_rxdesc_dmamap,
	    sizeof(aq_rx_desc_t) * idx, sizeof(aq_rx_desc_t),
	    BUS_DMASYNC_PREWRITE);
}

/* allocate mbuf and unload dmamap */
static int
aq_rxring_add(struct aq_softc *sc, struct aq_rxring *rxring, int idx)
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

	/* if mbuf already exists, unload and free */
	if (rxring->ring_mbufs[idx].m != NULL) {
		bus_dmamap_unload(sc->sc_dmat, rxring->ring_mbufs[idx].dmamap);
		m_freem(rxring->ring_mbufs[idx].m);
		rxring->ring_mbufs[idx].m = NULL;
	}

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

	aq_rxring_reset_desc(sc, rxring, idx);

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

#ifdef XXX_DEBUG_PMAP_EXTRACT
	{
		bool ok;
		char *descp = (char *)rxring->ring_rxdesc;
		paddr_t pa;
		vaddr_t va;

		printf("RX desc size = %lu\n", rxring->ring_rxdesc_size);
		printf("RX desc DM_SEGS[0] = PA=%08lx\n", rxring->ring_rxdesc_dmamap->dm_segs[0].ds_addr);

		for (i = 0; (bus_size_t)(PAGE_SIZE * i) < rxring->ring_rxdesc_size; i++) {
			va = (vaddr_t)descp + PAGE_SIZE * i;
			ok = pmap_extract(pmap_kernel(), va, &pa);
			printf("RX desc VA=%lx, PA=%08lx, ok=%d\n", va, pa, ok);
		}
	}
#endif

	memset(rxring->ring_rxdesc, 0, sizeof(aq_rx_desc_t) * AQ_RXD_NUM);

	/* fill rxring with dmamaps */
	for (i = 0; i < AQ_RXD_NUM; i++) {
		rxring->ring_mbufs[i].m = NULL;
		bus_dmamap_create(sc->sc_dmat, MCLBYTES, 1,
		    MCLBYTES, 0, 0,
		    &rxring->ring_mbufs[i].dmamap);
	}
	return 0;
}

static void
aq_rxdrain(struct aq_softc *sc, struct aq_rxring *rxring)
{
	int i;

	/* free all mbufs allocated for RX */
	for (i = 0; i < AQ_RXD_NUM; i++) {
		if (rxring->ring_mbufs[i].m != NULL) {
			bus_dmamap_unload(sc->sc_dmat, rxring->ring_mbufs[i].dmamap);
			m_freem(rxring->ring_mbufs[i].m);
			rxring->ring_mbufs[i].m = NULL;
		}
	}
}

/* free a rx ring */
static void
aq_rxring_free(struct aq_softc *sc, struct aq_rxring *rxring)
{
	int i;

	/* free all mbufs and dmamaps */
	aq_rxdrain(sc, rxring);
	for (i = 0; i < AQ_RXD_NUM; i++) {
		if (rxring->ring_mbufs[i].dmamap != NULL) {
			bus_dmamap_destroy(sc->sc_dmat, rxring->ring_mbufs[i].dmamap);
			rxring->ring_mbufs[i].dmamap = NULL;
		}
	}

	/* free RX descriptor */
	_free_dma(sc, &rxring->ring_rxdesc_size, (void **)&rxring->ring_rxdesc,
	    &rxring->ring_rxdesc_dmamap, rxring->ring_rxdesc_seg);
}


static int
aq_txrx_rings_alloc(struct aq_softc *sc)
{
	int n, error;

	for (n = 0; n < sc->sc_txringnum; n++) {
		sc->sc_txring[n].ring_sc = sc;
		sc->sc_txring[n].ring_index = n;
		error = aq_txring_alloc(sc, &sc->sc_txring[n]);
		if (error != 0)
			goto failure;
	}

	for (n = 0; n < sc->sc_rxringnum; n++) {
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

	for (n = 0; n < sc->sc_txringnum; n++) {
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
			printf("txring->ring_txdesc[%d].ctl  = %08x%s%s\n", i, txring->ring_txdesc[i].ctl,
			    (txring->ring_txdesc[i].ctl & AQ_TXDESC_CTL_EOP) ? " EOP" : "",
			    (txring->ring_txdesc[i].ctl & AQ_TXDESC_CTL_CMD_WB) ? " WB" : "");
			printf("txring->ring_txdesc[%d].ctl2 = %08x\n", i, txring->ring_txdesc[i].ctl2);
		}
	}
}

static void
dump_rxrings(struct aq_softc *sc)
{
	struct aq_rxring *rxring;
	int n, i;

	for (n = 0; n < sc->sc_rxringnum; n++) {
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

	for (n = 0; n < sc->sc_txringnum; n++)
		aq_txring_free(sc, &sc->sc_txring[n]);

	for (n = 0; n < sc->sc_rxringnum; n++)
		aq_rxring_free(sc, &sc->sc_rxring[n]);
}

#ifdef USE_CALLOUT_TICK
static void
aq_tick(void *arg)
{
	struct aq_softc *sc = arg;

	aq_if_update_admin_status(sc);

	callout_reset(&sc->sc_tick_ch, hz, aq_tick, sc);
}
#endif

static int
aq_intr(void *arg)
{
	struct aq_softc *sc __unused = arg;
	uint32_t status;
	int handled = 0;
	int i;

	status = AQ_READ_REG(sc, AQ_INTR_STATUS);
#ifdef XXX_INTR_DEBUG
	printf("#### INTERRUPT #### %s@cpu%d: INTR_MASK/INTR_STATUS = %08x/%08x=>%08x\n", __func__, cpu_index(curcpu()), AQ_READ_REG(sc, AQ_INTR_MASK), status, AQ_READ_REG(sc, AQ_INTR_STATUS));
#endif

	if (status & __BIT(LINKUP_IRQ)) {
		handled += aq_if_update_admin_status(sc);
	}

	for (i = 0; i < sc->sc_rxringnum; i++) {
		if (status & __BIT(i)) {
			handled += aq_rx_intr(&sc->sc_rxring[i]);
		}
#ifdef XXX_INTR_DEBUG
		else if (aq_rx_intr_poll(&sc->sc_rxring[i]) != 0) {
			printf("INTR: RX: rxring[%d] modified, but no INTR status\n", i);
		}
#endif
	}
	for (i = 0; i < sc->sc_txringnum; i++) {
		if (status & __BIT(i)) {
			handled += aq_tx_intr(&sc->sc_txring[i]);
		}
#ifdef XXX_INTR_DEBUG
		else if (aq_tx_intr_poll(&sc->sc_txring[i]) != 0) {
			printf("INTR: TX: txring[%d] modified, but no INTR status\n", i);
		}
#endif
	}

	AQ_WRITE_REG(sc, AQ_INTR_STATUS_CLR, 0xffffffff);	//XXX
	return handled;
}

/* Interrupt enable / disable */
static void
aq_enable_intr(struct aq_softc *sc, bool linkup, bool txrx)
{
	/* Enable interrupts */
	if (txrx) {
		/* including linkup intr */
		AQ_WRITE_REG(sc, AQ_INTR_MASK, __BITS(0, sc->sc_ringnum - 1));
	} else if (linkup) {
		/* only linkup intr */
		AQ_WRITE_REG(sc, AQ_INTR_MASK, __BIT(LINKUP_IRQ));
	} else {
		AQ_WRITE_REG(sc, AQ_INTR_MASK, 0);
	}

	AQ_WRITE_REG(sc, AQ_INTR_STATUS_CLR, 0xffffffff);	//XXX

	printf("%s:%d: INTR_MASK/INTR_STATUS=%08x/%08x\n", __func__, __LINE__, AQ_READ_REG(sc, AQ_INTR_MASK), AQ_READ_REG(sc, AQ_INTR_STATUS));
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
#ifdef USE_CALLOUT_TICK
	callout_init(&sc->sc_tick_ch, 0);	/* XXX: CALLOUT_MPSAFE */
#endif
	sc->sc_pc = pc = pa->pa_pc;
	sc->sc_pcitag = tag = pa->pa_tag;
#ifdef XXX_FORCE_32BIT_PA
	sc->sc_dmat = pa->pa_dmat;
#else
	sc->sc_dmat = pci_dma64_available(pa) ? pa->pa_dmat64 : pa->pa_dmat;
#endif

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

	sc->sc_lro_enable = CONFIG_LRO_ENABLE;
	sc->sc_rss_enable = CONFIG_RSS_ENABLE;

	sc->sc_txringnum = AQ_TXRING_NUM;
	sc->sc_rxringnum = AQ_RXRING_NUM;
	sc->sc_ringnum = MAX(sc->sc_txringnum, sc->sc_rxringnum);
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

	aq_get_mac_addr(sc);

	error = aq_hw_init(sc);	/* initialize and interrupts */
	if (error != 0)
		goto attach_failure;


	sc->sc_media_type = aqp->aq_media_type;
	sc->sc_available_rates = aqp->aq_available_rates;

	sc->sc_link_rate = AQ_LINK_AUTO;
	sc->sc_link_fc = AQ_FW_FC_ENABLE_ALL;
	sc->sc_link_eee = AQ_FW_EEE_NONE;

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
	ether_set_ifflags_cb(&sc->sc_ethercom, aq_ifflags_cb);

	aq_enable_intr(sc, true, false);

	/* media update */
	aq_set_linkmode(sc);

	/* get starting statistics values */
	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->get_stats != NULL &&
	    (sc->sc_fw_ops->get_stats(sc, &sc->sc_statistics[0]) == 0)) {
		sc->sc_statistics_enable = true;
	}

#ifdef USE_CALLOUT_TICK
	callout_reset(&sc->sc_tick_ch, hz, aq_tick, sc);
#endif

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

		aprint_debug_dev(sc->sc_dev, "%s: bus_space_unmap\n", __func__);
		bus_space_unmap(sc->sc_iot, sc->sc_ioh, sc->sc_iosize);
		sc->sc_iosize = 0;
	}

#ifdef USE_CALLOUT_TICK
	callout_stop(&sc->sc_tick_ch);
#endif
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
#ifdef XXX_DUMP_MACTABLE
	struct aq_softc *sc = ifp->if_softc;
	aq_dump_mactable(sc);
#endif

	return aq_mediachange(ifp);
}

static void
aq_ifmedia_status(struct ifnet * const ifp, struct ifmediareq *req)
{
	aq_mediastatus(ifp, req);
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
	AQ_WRITE_REG_BIT(sc, TX_DMA_DESC_LEN_ADR(ringidx), TX_DMA_DESC_ENABLE, 0);

	if (enable_dma) {
		/* TX descriptor physical address */
		paddr_t paddr = txring->ring_txdesc_dmamap->dm_segs[0].ds_addr;
		AQ_WRITE_REG(sc, TX_DMA_DESC_BASE_ADDRLSW_ADR(ringidx), paddr);
		AQ_WRITE_REG(sc, TX_DMA_DESC_BASE_ADDRMSW_ADR(ringidx), paddr >> 32);

		/* TX descriptor size */
		AQ_WRITE_REG_BIT(sc, TX_DMA_DESC_LEN_ADR(ringidx), TX_DMA_DESC_LEN_MSK,
		    AQ_TXD_NUM / 8);

		/* reset TAIL pointer */
		AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(ringidx), 0);
		AQ_WRITE_REG(sc, TX_DMA_DESC_WRWB_THRESH_ADR(ringidx), 0);

		/* irq map */
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_TX_ADR(ringidx), AQ_INTR_IRQ_MAP_TX_MSK(ringidx), ringidx /*=msix*/);
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_TX_ADR(ringidx), AQ_INTR_IRQ_MAP_TX_EN_MSK(ringidx), true);

		/* enable DMA */
		AQ_WRITE_REG_BIT(sc, TX_DMA_DESC_LEN_ADR(ringidx), TX_DMA_DESC_ENABLE, 1);

		const int cpuid = 0;	//XXX
		AQ_WRITE_REG_BIT(sc, TDM_DCAD_ADR(ringidx), TDM_DCAD_CPUID_MSK, cpuid);
		AQ_WRITE_REG_BIT(sc, TDM_DCAD_ADR(ringidx), TDM_DCAD_CPUID_EN, 0);
	}
}

static void
aq_txring_start(struct aq_softc *sc, struct aq_txring *txring)
{
//	printf("%s:%d: txring[%d] proidx = %d\n", __func__, __LINE__, txring->ring_index, txring->ring_prodidx);

	AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(txring->ring_index),
	    txring->ring_prodidx);
}

static int
aq_rxring_init(struct aq_softc *sc, struct aq_rxring *rxring, bool enable_dma)
{
	const int ringidx = rxring->ring_index;
	int i;
	int error = 0;

	rxring->ring_readidx = 0;

	/* disable DMA once */
	AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_LEN_ADR(ringidx), RX_DMA_DESC_ENABLE, 0);

	/* free all RX mbufs */
	aq_rxdrain(sc, rxring);

	if (enable_dma) {
		for (i = 0; i < AQ_RXD_NUM; i++) {
			error = aq_rxring_add(sc, rxring, i);
			if (error != 0) {
				aq_rxdrain(sc, rxring);
				return error;
			}
		}

		/* RX descriptor physical address */
		paddr_t paddr = rxring->ring_rxdesc_dmamap->dm_segs[0].ds_addr;
		AQ_WRITE_REG(sc, RX_DMA_DESC_BASE_ADDRLSW_ADR(ringidx), paddr);
		AQ_WRITE_REG(sc, RX_DMA_DESC_BASE_ADDRMSW_ADR(ringidx), paddr >> 32);

		/* RX descriptor size */
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_LEN_ADR(ringidx), RX_DMA_DESC_LEN_MSK,
		    AQ_RXD_NUM / 8);

		/* maximum receive frame size */
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_BUFSIZE_ADR(ringidx), RX_DMA_DESC_BUFSIZE_DATA_MSK, MCLBYTES / 1024);
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_BUFSIZE_ADR(ringidx), RX_DMA_DESC_BUFSIZE_HDR_MSK, 0 / 1024);

		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_LEN_ADR(ringidx), RX_DMA_DESC_HEADER_SPLIT, 0);
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_LEN_ADR(ringidx), RX_DMA_DESC_VLAN_STRIP, 0);

		/* reset HEAD and TAIL pointer */
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_ADR(ringidx), RX_DMA_DESC_HEAD_PTR_MSK, 0);
		AQ_WRITE_REG(sc, RX_DMA_DESC_TAIL_PTR_ADR(ringidx), AQ_RXD_NUM - 1);

		/* Rx ring set mode */

		/* Mapping interrupt vector */
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_RX_ADR(ringidx), AQ_INTR_IRQ_MAP_RX_MSK(ringidx), ringidx /*=msix*/);
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_RX_ADR(ringidx), AQ_INTR_IRQ_MAP_RX_EN_MSK(ringidx), 1);

		const int cpuid = 0;	//XXX
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_ADR(ringidx), RDM_DCAD_CPUID_MSK, cpuid);
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_ADR(ringidx), RDM_DCAD_DESC_EN, 0);
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_ADR(ringidx), RDM_DCAD_HEADER_EN, 0);
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_ADR(ringidx), RDM_DCAD_PAYLOAD_EN, 0);

		/* rxring_start: start receiving */
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_LEN_ADR(ringidx), RX_DMA_DESC_ENABLE, 1);
	}

	return error;
}

#define TXRING_NEXTIDX(idx)	\
	(((idx) >= (AQ_TXD_NUM - 1)) ? 0 : ((idx) + 1))
#define RXRING_NEXTIDX(idx)	\
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
			    AQ_TXDESC_CTL_EOP |
			    AQ_TXDESC_CTL_CMD_WB;
		}

#if 0
		 printf("%s:%d: write txdesc[%3d] seg:%d/%d buf_addr=%012lx, len=%-5lu ctl=%08x ctl2=%08x%s\n", __func__, __LINE__, idx,
		    i, map->dm_nsegs - 1,
		     map->dm_segs[i].ds_addr,
		     map->dm_segs[i].ds_len,
		    txring->ring_txdesc[idx].ctl,
		    txring->ring_txdesc[idx].ctl2,
		     (i == map->dm_nsegs - 1) ? " EOP/WB" : "");
#endif

		bus_dmamap_sync(sc->sc_dmat, txring->ring_txdesc_dmamap,
		    sizeof(aq_tx_desc_t) * idx, sizeof(aq_tx_desc_t),
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

		idx = TXRING_NEXTIDX(idx);
		txring->ring_nfree--;
	}

	txring->ring_prodidx = idx;

	return 0;
}

#ifdef XXX_INTR_DEBUG
static int
aq_tx_intr_poll(struct aq_txring *txring)
{
	struct aq_softc *sc = txring->ring_sc;

	if (txring->ring_considx == AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_ADR(txring->ring_index), TX_DMA_DESC_HEAD_PTR_MSK))
		return 0;
	return 1;
}
#endif

static int
aq_tx_intr(struct aq_txring *txring)
{
	struct aq_softc *sc = txring->ring_sc;
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	unsigned int idx, hw_head;

	//XXX: need lock

	hw_head = AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_ADR(txring->ring_index), TX_DMA_DESC_HEAD_PTR_MSK);
	if (hw_head == txring->ring_considx)
		return 0;

#if 0
	printf("%s:%d: ringidx=%d, TXDESC_(EN+)LEN=%08x\n", __func__, __LINE__,
	    txring->ring_index, AQ_READ_REG(sc, TX_DMA_DESC_LEN_ADR(txring->ring_index)));
	printf("%s:%d: ringidx=%d, HEAD/TAIL=%lu/%u prod/cons=%d/%d\n", __func__, __LINE__, txring->ring_index,
	    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_ADR(txring->ring_index), TX_DMA_DESC_HEAD_PTR_MSK),
	    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(txring->ring_index)),
	    txring->ring_prodidx,
	    txring->ring_considx);
#endif

	for (idx = txring->ring_considx; idx != hw_head; idx = TXRING_NEXTIDX(idx)) {

#if 0
		printf("# %s:%d: txring=%d, TX CLEANUP: HEAD/TAIL=%lu/%u, considx/prodidx=%d/%d, idx=%d\n", __func__, __LINE__,
		    txring->ring_index,
		    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_ADR(txring->ring_index), TX_DMA_DESC_HEAD_PTR_MSK),
		    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(txring->ring_index)),
		    txring->ring_considx,
		    txring->ring_prodidx,
		    idx);
#endif

#if 0
		//DEBUG. show done txdesc
		bus_dmamap_sync(sc->sc_dmat, txring->ring_txdesc_dmamap,
		    sizeof(aq_tx_desc_t) * idx, sizeof(aq_tx_desc_t),
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		printf("%s:%d: written txdesc[%3d] buf_addr=%012lx, ctl=%08x ctl2=%08x\n", __func__, __LINE__,
		    idx,
		    txring->ring_txdesc[idx].buf_addr,
		    txring->ring_txdesc[idx].ctl,
		    txring->ring_txdesc[idx].ctl2);
#endif

#if 0
		//DEBUG? clear done txdesc
		txring->ring_txdesc[idx].buf_addr = 0;
		txring->ring_txdesc[idx].ctl = AQ_TXDESC_CTL_DD;
		txring->ring_txdesc[idx].ctl2 = 0;
		bus_dmamap_sync(sc->sc_dmat, txring->ring_txdesc_dmamap,
		    sizeof(aq_tx_desc_t) * idx, sizeof(aq_tx_desc_t),
		    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
#endif

		if (txring->ring_mbufs[idx].m != NULL) {
			bus_dmamap_unload(sc->sc_dmat, txring->ring_mbufs[idx].dmamap);
			m_freem(txring->ring_mbufs[idx].m);
			txring->ring_mbufs[idx].m = NULL;
			ifp->if_opackets++;
		}

		txring->ring_nfree++;
	}
	txring->ring_considx = idx;

	if (txring->ring_nfree > 0)
		ifp->if_flags &= ~IFF_OACTIVE;

	/*
	 * no more pending TX packet? cancel watchdog.
	 * XXX: consider multi tx rings...
	 */
	if (txring->ring_nfree >= AQ_TXD_NUM) {
		ifp->if_timer = 0;

#if 0 /* force reset TXRING */
		//XXX reset txring
		txring->ring_prodidx = 0;
		txring->ring_considx = 0;
		AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(txring->ring_index), 0);
#endif
	}

	return 1;
}

#ifdef  XXX_INTR_DEBUG
static int
aq_rx_intr_poll(struct aq_rxring *rxring)
{
	struct aq_softc *sc = rxring->ring_sc;

	if (rxring->ring_readidx == AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_ADR(rxring->ring_index), RX_DMA_DESC_HEAD_PTR_MSK))
		return 0;
	return 1;
}
#endif

static int
aq_rx_intr(struct aq_rxring *rxring)
{
	struct aq_softc *sc = rxring->ring_sc;
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	const int ringidx = rxring->ring_index;
	aq_rx_desc_t *rxd;
	struct mbuf *m, *m0, *mprev;
	uint32_t rxd_type, rxd_hash __unused;
	uint16_t rxd_status, rxd_pktlen, rxd_nextdescptr __unused, rxd_vlan __unused;
	unsigned int idx, amount;

	if (rxring->ring_readidx == AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_ADR(ringidx), RX_DMA_DESC_HEAD_PTR_MSK))
		return 0;

#ifdef XXX_RXINTR_DEBUG
	//XXX: need lock
	printf("# %s:%d\n", __func__, __LINE__);
#endif

#ifdef XXX_DUMP_RX_COUNTER
	printf("RXPKT:%lu, RXBYTE:%lu, DMADROP:%u\n",
	    AQ_READ64_REG(sc, RX_DMA_GOOD_PKT_COUNTERLSW),
	    AQ_READ64_REG(sc, RX_DMA_GOOD_OCTET_COUNTERLSW),
	    AQ_READ_REG(sc, RX_DMA_DROP_PKT_CNT_ADR));
#endif

#ifdef XXX_RXINTR_DEBUG
	printf("%s:%d: begin: RX_DMA_DESC_HEAD/TAIL=%lu/%u, readidx=%u\n", __func__, __LINE__,
	    AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_ADR(ringidx), RX_DMA_DESC_HEAD_PTR_MSK),
	    AQ_READ_REG(sc, RX_DMA_DESC_TAIL_PTR_ADR(ringidx)), rxring->ring_readidx);
#endif

	m0 = mprev = NULL;
	amount = 0;
	for (idx = rxring->ring_readidx;
	    idx != AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_ADR(ringidx), RX_DMA_DESC_HEAD_PTR_MSK);
	    idx = RXRING_NEXTIDX(idx)) {

		bus_dmamap_sync(sc->sc_dmat, rxring->ring_rxdesc_dmamap,
		    sizeof(aq_rx_desc_t) * idx, sizeof(aq_rx_desc_t),
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

		rxd = &rxring->ring_rxdesc[idx];
		rxd_status = le16toh(rxd->wb.status);

		if ((rxd_status & RXDESC_STATUS_DD) == 0)
			break;	/* not yet done */

		rxd_type = le32toh(rxd->wb.type);

		if (((rxd_status & RXDESC_STATUS_MAC_DMA_ERR) != 0) &&
		    ((rxd_type & RXDESC_TYPE_RDM_ERR) != 0)) {
			aq_rxring_reset_desc(sc, rxring, idx);
			goto rx_next;
		}

		rxd_pktlen = le16toh(rxd->wb.pkt_len);
//		rxd_nextdescptr = le16toh(rxd->wb.next_desc_ptr);
//		rxd_hash = le32toh(rxd->wb.rss_hash);
//		rxd_vlan = le16toh(rxd->wb.vlan);

#ifdef XXX_RXINTR_DEBUG
		printf("desc[%d] type=0x%08x, hash=0x%08x, status=0x%08x, pktlen=%u, nextdesc=%u, vlan=0x%x\n",
		    idx, rxd_type, rxd_hash, rxd_status, rxd_pktlen, rxd_nextdescptr, rxd_vlan);
		printf(" type: rss=%ld, pkttype=%ld, rdm=%ld, cntl=%ld, sph=%ld, hdrlen=%ld\n",
		    __SHIFTOUT(rxd_type, RXDESC_TYPE_RSS),
		    __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE),
		    __SHIFTOUT(rxd_type, RXDESC_TYPE_RDM_ERR),
		    __SHIFTOUT(rxd_type, RXDESC_TYPE_CNTL),
		    __SHIFTOUT(rxd_type, RXDESC_TYPE_SPH),
		    __SHIFTOUT(rxd_type, RXDESC_TYPE_HDR_LEN));
#endif


		bus_dmamap_sync(sc->sc_dmat, rxring->ring_mbufs[idx].dmamap, 0,
		    rxring->ring_mbufs[idx].dmamap->dm_mapsize, BUS_DMASYNC_POSTREAD);

		m = rxring->ring_mbufs[idx].m;
		rxring->ring_mbufs[idx].m = NULL;

		m->m_len = rxd_pktlen;
#ifdef XXX_DUMP_RX_MBUF
		hexdump(printf, "mbuf", m->m_data, m->m_len);	// dump this mbuf
#endif

		if (m0 == NULL) {
			m0 = m;
			amount = m->m_len;
		} else {
			amount += m->m_len;
			if (m->m_flags & M_PKTHDR)
				m_remove_pkthdr(m);
			mprev->m_next = m;
		}
		mprev = m;

		if ((rxd_status & RXDESC_STATUS_EOP) != 0) {
			/* last buffer */
			m_set_rcvif(m0, ifp);
			m->m_pkthdr.len = amount;
			if_percpuq_enqueue(ifp->if_percpuq, m);

			m0 = mprev = NULL;
			amount = 0;
		}

		/* refill, and update tail */
		aq_rxring_add(sc, rxring, idx);
 rx_next:
		AQ_WRITE_REG(sc, RX_DMA_DESC_TAIL_PTR_ADR(ringidx), idx);
	}
	rxring->ring_readidx = idx;

#ifdef XXX_RXINTR_DEBUG
	printf("%s:%d: end: RX_DMA_DESC_HEAD/TAIL=%lu/%u, readidx=%u\n", __func__, __LINE__,
	    AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_ADR(ringidx), RX_DMA_DESC_HEAD_PTR_MSK),
	    AQ_READ_REG(sc, RX_DMA_DESC_TAIL_PTR_ADR(ringidx)), rxring->ring_readidx);
#endif

	return 1;
}

static int
aq_ifflags_cb(struct ethercom *ec)
{
	struct ifnet *ifp = &ec->ec_if;
	struct aq_softc *sc = ifp->if_softc;
	unsigned short iffchange;

	//XXX: need lock
	printf("%s:%d\n", __func__, __LINE__);


	iffchange = ifp->if_flags ^ sc->sc_if_flags;

	if ((iffchange & IFF_PROMISC) != 0) {
		AQ_WRITE_REG_BIT(sc, RPFL2BC_EN_ADR, RPFL2BC_PROMISC_MODE,
		    (ifp->if_flags & IFF_PROMISC) ? 1 : 0);
	}

	sc->sc_if_flags = ifp->if_flags;
	return 0;
}


static int
aq_init(struct ifnet *ifp)
{
	struct aq_softc *sc = ifp->if_softc;
	int i, error = 0;

	//XXX: need lock
	printf("%s:%d\n", __func__, __LINE__);

	aq_update_vlan_filters(sc);

	/* start TX */
	for (i = 0; i < sc->sc_txringnum; i++) {
		aq_txring_init(sc, &sc->sc_txring[i], true);
	}
	AQ_WRITE_REG_BIT(sc, TPB_TX_BUF_ADR, TPB_TX_BUF_EN, 1);

	/* start RX */
	for (i = 0; i < sc->sc_rxringnum; i++) {
		error = aq_rxring_init(sc, &sc->sc_rxring[i], true);
		if (error != 0)
			goto aq_init_failure;
	}
	AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_ADR, RPB_RPF_RX_BUF_EN, 1);

	//XXX
	(void)&dump_txrings;
	(void)&dump_rxrings;
//	dump_txrings(sc);
//	dump_rxrings(sc);


//	aq_hw_start();
	aq_enable_intr(sc, true, true);
	aq_hw_rss_hash_set(sc);
	aq_hw_rss_set(sc);
	aq_hw_udp_rss_enable(sc, true);

#ifdef USE_CALLOUT_TICK
	/* for resume */
	callout_reset(&sc->sc_tick_ch, hz, aq_tick, sc);
#endif

	/* ready */
	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

 aq_init_failure:

	sc->sc_if_flags = ifp->if_flags;
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

	txring = &sc->sc_txring[0];	// select TX ring

#if 0
	printf("%s:%d: ringidx=%d, HEAD/TAIL=%lu/%u, INTR_MASK/INTR_STATUS=%08x/%08x\n",
	    __func__, __LINE__, txring->ring_index,
	    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_ADR(txring->ring_index), TX_DMA_DESC_HEAD_PTR_MSK),
	    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(txring->ring_index)),
	    AQ_READ_REG(sc, AQ_INTR_MASK), AQ_READ_REG(sc, AQ_INTR_STATUS));
#endif

	for (npkt = 0; ; npkt++) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		if (txring->ring_nfree <= 0) {
			/* no tx descriptor now... */
			ifp->if_flags |= IFF_OACTIVE;
			device_printf(sc->sc_dev, "TX descriptor is full\n");
			break;
		}

		IFQ_DEQUEUE(&ifp->if_snd, m);

		if (aq_encap_txring(sc, txring, &m) != 0) {
			/* too many mbuf chains? */
			device_printf(sc->sc_dev,
			    "TX descriptor enqueueing failure. dropping packet\n");
			m_freem(m);
			ifp->if_oerrors++;
			break;
		}

		/* start TX DMA */
		aq_txring_start(sc, txring);

		/* Pass the packet to any BPF listeners */
		bpf_mtap(ifp, m, BPF_D_OUT);
	}

	if (npkt)
		ifp->if_timer = 5;

//	device_printf(sc->sc_dev, "ring[%d] %d/%d\n", txring->ring_index, AQ_TXD_NUM - txring->ring_nfree, AQ_TXD_NUM);
}

static void
aq_stop(struct ifnet *ifp, int disable)
{
	struct aq_softc *sc = ifp->if_softc;
	int i;

	//XXX: need lock
	printf("%s:%d: disable=%d\n", __func__, __LINE__, disable);

	/* disable interrupts */
	aq_enable_intr(sc, false, false);

	for (i = 0; i < sc->sc_txringnum; i++) {
		aq_txring_init(sc, &sc->sc_txring[i], false);
	}
	for (i = 0; i < sc->sc_rxringnum; i++) {
		aq_rxring_init(sc, &sc->sc_rxring[i], false);
	}
	//toggle cache
	AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_CACHE_INIT_ADR, RX_DMA_DESC_CACHE_INIT_MSK,
	    AQ_READ_REG_BIT(sc, RX_DMA_DESC_CACHE_INIT_ADR, RX_DMA_DESC_CACHE_INIT_MSK) ^ 1);

	ifp->if_timer = 0;

	//XXX
	if (disable) {
		/* ifconfig down, but linkup intr is enabled */
		aq_enable_intr(sc, true, false);
	} else {
#ifdef USE_CALLOUT_TICK
		/* pmf stop, disable callout */
		callout_stop(&sc->sc_tick_ch);
#endif
	}

	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
}

static void
aq_watchdog(struct ifnet *ifp)
{
	struct aq_softc *sc = ifp->if_softc;
	struct aq_txring *txring;
	int n;

	uint32_t status;
	status = AQ_READ_REG(sc, AQ_INTR_STATUS);
	printf("####!!!! %s@cpu%d: INTR_MASK/INTR_STATUS = %08x/%08x=>%08x\n", __func__, cpu_index(curcpu()), AQ_READ_REG(sc, AQ_INTR_MASK), status, AQ_READ_REG(sc, AQ_INTR_STATUS));


	for (n = 0; n < sc->sc_txringnum; n++) {
		txring = &sc->sc_txring[n];

#if 1
		//DEBUG
		printf("%s:%d: ringidx=%d, TXDESC_(EN+)LEN=%08x\n", __func__, __LINE__,
		    txring->ring_index, AQ_READ_REG(sc, TX_DMA_DESC_LEN_ADR(txring->ring_index)));
		printf("%s:%d: ringidx=%d, HEAD/TAIL=%lu/%u\n", __func__, __LINE__, txring->ring_index,
		    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_ADR(txring->ring_index), TX_DMA_DESC_HEAD_PTR_MSK),
		    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_ADR(txring->ring_index)));
#endif

		aq_tx_intr(txring);
	}

	aq_init(ifp);
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
