//
// PROBLEM
//	謎条件で、pingをとりこぼしまくる。attachしてすぐにIP addressを設定すると起きやすい? 初期化問題?
//		→どうやらaq_update_link_status()でRPB_RXB_XOFF_ENをセットしていたからっぽい? 0固定で良いっぽい。
//		→まだ起きるっぽい。detach && attach するとなおる(少くともring_num=1,8のとき起きた)
//		→RX_DMA_HEADは書き換え不可だった。でもこれを修正してもまだ落とすことがある？？？
//
//	iperfのudpが極端に遅い。落としまくってる? 他のNICでも起きるのでaqの問題ではなさそう。
//
//
// MEMO?
//	VLANIDで16ヶのringに振り分け可能?
//	ETHERTYPEで16ヶのringに振り分け可能?
//	L3 filterはで8ヶのringに振り分け可能?
//		（Linux版のAQ_RX_FIRST_LOC_FVLANIDあたりの定義より）
//	そうだとすると、RX_FLR_RSS_CONTROL1_REG の 0x33333333 の意味がなんとなくわかる(0b11が8ヶ=8ring分)
//
//	L3-L4フィルタはその名の通り、discardするかhostのどのringで受けるかを決めるテーブルであり、rssとは関係ないようだ。
//
//	RSS_ENABLEの状態だと0800と8d66が届かないけど、statistics的には受信している
//	そしてUDPはちゃんと受信している。(たぶんL3 filterでUDP throughにしてるせい)
//
//
//
// TODO
//	rss (+取りこぼし問題?)
//	lock
//	hardware offloading
//	cleanup debug printf
//	tuning
//	interrupt moderation
//	IP header offset 4n+2問題
//	msix
//	vlan
//	counters, evcnt
//	cleanup source
//	fulldup control? (100baseTX)
//	fw1x (revision A0)
//

//#define XXX_FORCE_32BIT_PA
//#define XXX_DEBUG_PMAP_EXTRACT
#define USE_CALLOUT_TICK
//#define XXX_DUMP_STAT
//#define XXX_INTR_DEBUG
//#define XXX_RXINTR_DEBUG
#define XXX_TXDESC_DEBUG
#define XXX_RXDESC_DEBUG
//#define XXX_DUMP_RX_COUNTER
#define XXX_DUMP_RX_MBUF
//#define XXX_DUMP_MACTABLE
//#ifdef XXX_DUMP_RING
#define XXX_DUMP_RSS_KEY
//#define XXX_DEBUG_RSSKEY_ZERO

//
// terminology
//
//	MPI = MAC PHY INTERFACE?
//	RPO = RX Protocol Offloading?
//	TPO = TX Protocol Offloading?
//	RPF = RX Packet Filter
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

#define CONFIG_INTR_MODERATION_ENABLE	1	/* ok */

#undef CONFIG_LRO_SUPPORT
#define CONFIG_RSS_ENABLE		0
#define CONFIG_OFFLOAD_ENABLE		0
#define CONFIG_L3_FILTER_ENABLE		0

#define AQ_RSS_HASHKEY_SIZE			40
#define AQ_RSS_INDIRECTION_TABLE_MAX		64


#define AQ_SOFTRESET_REG			0x0000
#define  AQ_SOFTRESET_RESET			__BIT(15) /* soft reset bit */
#define  AQ_SOFTRESET_DISABLE			__BIT(14) /* reset disable */

#define AQ_FW_VERSION_REG			0x0018
#define AQ_HW_REVISION_REG			0x001c
#define FW_GLB_NVR_INTERFACE1_REG		0x0100

#define AQ_FW_MBOX_CMD_REG			0x0200
#define  AQ_FW_MBOX_CMD_EXECUTE			0x00008000
#define  AQ_FW_MBOX_CMD_BUSY			0x00000100
#define AQ_FW_MBOX_ADDR_REG			0x0208
#define AQ_FW_MBOX_VAL_REG			0x020c

#define FW_GLB_CPU_SCRATCH_SCP_REG(i)		(0x0300 + (i) * 4)

#define FW2X_LED_MIN_VERSION			0x03010026	/* require 3.1.38 */
#define FW2X_LED_REG				0x031c
#define  FW2X_LED_DEFAULT			0x00000000
#define  FW2X_LED_NONE				0x0000003f
#define  FW2X_LINKLED				__BITS(0,1)
#define   FW2X_LINKLED_ACTIVE			0
#define   FW2X_LINKLED_ON			1
#define   FW2X_LINKLED_BLINK			2
#define   FW2X_LINKLED_OFF			3
#define  FW2X_STATUSLED				__BITS(2,5)
#define   FW2X_STATUSLED_ORANGE			0
#define   FW2X_STATUSLED_ORANGE_BLINK		2
#define   FW2X_STATUSLED_OFF			3
#define   FW2X_STATUSLED_GREEN			4
#define   FW2X_STATUSLED_ORANGE_GREEN_BLINK	8
#define   FW2X_STATUSLED_GREEN_BLINK		10

#define FW1X_0X370_REG				0x0370
#define FW1X_MPI_EFUSEADDR_REG			0x0374

#define FW2X_MPI_MBOX_ADDR_REG			0x0360
#define FW2X_MPI_EFUSEADDR_REG			0x0364
#define FW2X_MPI_CONTROL_REG			0x0368	/* 64bit */
#define FW2X_MPI_STATE_REG			0x0370	/* 64bit */
#define FW_BOOT_EXIT_CODE_REG			0x0388
#define  RBL_STATUS_DEAD			0x0000dead
#define  RBL_STATUS_SUCCESS			0x0000abba
#define  RBL_STATUS_FAILURE			0x00000bad
#define  RBL_STATUS_HOST_BOOT			0x0000f1a7

#define FW_GLB_CPU_SEM_REG(i)			(0x03a0 + (i) * 4)
#define FW_SEM_RAM_REG				FW_GLB_CPU_SEM_REG(2)

#define FW_GLB_CTL2_REG				0x0404
#define  FW_GLB_CTL2_MCP_UP_FORCE_INTERRUPT	__BIT(1)

#define FW_GLB_GENERAL_PROVISIONING9_REG	0x0520
#define FW_GLB_NVR_PROVISIONING2_REG		0x0534

#define FW_DAISY_CHAIN_STATUS_REG		0x0704

#define AQ_PCI_REG_CONTROL_6_REG		0x1014

// msix bitmap */
#define AQ_INTR_STATUS_REG			0x2000	/* intr status */
#define AQ_INTR_STATUS_CLR_REG			0x2050	/* intr status clear */
#define AQ_INTR_MASK_REG			0x2060	/* intr mask set */
#define AQ_INTR_MASK_CLR_REG			0x2070	/* intr mask clear */
#define AQ_INTR_AUTOMASK_REG			0x2090

#define AQ_INTR_IRQ_MAP_TXRX_REG(i)		(0x2100 + ((i) / 2) * 4)
#define AQ_INTR_IRQ_MAP_TX_REG(tx)		AQ_INTR_IRQ_MAP_TXRX_REG(tx)
#define  AQ_INTR_IRQ_MAP_TX_IRQMAP(tx)		(__BITS(28,24) >> (((tx) & 1) * 8))
#define  AQ_INTR_IRQ_MAP_TX_EN(tx)		(__BIT(31)     >> (((tx) & 1) * 8))
#define AQ_INTR_IRQ_MAP_RX_REG(rx)		AQ_INTR_IRQ_MAP_TXRX_REG(rx)
#define  AQ_INTR_IRQ_MAP_RX_IRQMAP(rx)		(__BITS(12,8)  >> (((rx) & 1) * 8))
#define  AQ_INTR_IRQ_MAP_RX_EN(rx)		(__BIT(15)     >> (((rx) & 1) * 8))

#define AQ_GEN_INTR_MAP_REG(i)			(0x2180 + (i) * 4)
#define  AQ_B0_ERR_INT				8

#define AQ_INTR_CTRL_REG			0x2300
#define  AQ_INTR_CTRL_IRQMODE			__BITS(1,0)
#define  AQ_INTR_CTRL_IRQMODE_LEGACY		0
#define  AQ_INTR_CTRL_IRQMODE_MSI		1
#define  AQ_INTR_CTRL_IRQMODE_MSIX		2
#define  AQ_INTR_CTRL_MULTIVEC			__BIT(2)	//?
#define  AQ_INTR_CTRL_AUTO_MASK			__BIT(5)
#define  AQ_INTR_CTRL_CLR_ON_READ		__BIT(7)
#define  AQ_INTR_CTRL_RESET_DIS			__BIT(29)
#define  AQ_INTR_CTRL_RESET_IRQ			__BIT(31)

#define AQ_MBOXIF_POWER_GATING_CONTROL_REG	0x32a8

#define MPI_RESETCTRL_REG			0x4000
#define  MPI_RESETCTRL_RESET_DIS		__BIT(29)

#define RX_SYSCONTROL_REG			0x5000
#define  RPB_DMA_SYS_LOOPBACK			__BIT(6)
#define  RPF_TPO_RPF_SYS_LOOPBACK		__BIT(8)
#define  RX_REG_RESET_DIS			__BIT(29)

#define RX_TCP_RSS_HASH_REG			0x5040

/* for RPF_*_REG[ACTION] */
#define RPF_ACTION_DISCARD			0
#define RPF_ACTION_HOST				1
#define RPF_ACTION_MANAGEMENT			2
#define RPF_ACTION_HOST_MANAGEMENT		3
#define RPF_ACTION_WOL				4

#define RPF_L2BC_REG				0x5100
#define  RPF_L2BC_EN				__BIT(0)
#define  RPF_L2BC_PROMISC			__BIT(3)
#define  RPF_L2BC_ACTION			__BITS(12,14)
#define  RPF_L2BC_THRESHOLD			__BITS(31,16)

#define RPF_L2UC_LSW_REG(idx)			(0x5110 + (idx) * 8)
#define RPF_L2UC_MSW_REG(idx)			(0x5114 + (idx) * 8)
#define  RPF_L2UC_MSW_MACADDR_HI		__BITS(15,0)
#define  RPF_L2UC_MSW_ACTION			__BITS(18,16)
#define  RPF_L2UC_MSW_EN			__BIT(31)
#define AQ_HW_MAC			0	/* index of own address */
#define AQ_HW_MAC_MIN			1
#define AQ_HW_MAC_MAX			33

#define RPF_MCAST_FILTER_REG(i)			(0x5250 + (i) * 4)
#define  RPF_MCAST_FILTER_EN			__BIT(31)
#define RPF_MCAST_FILTER_MASK_REG		0x5270
#define  RPF_MCAST_FILTER_MASK_ALLMULTI		__BIT(14)

#define RPF_VLAN_MODE_REG			0x5280
#define  RPF_VLAN_MODE_PROMISC			__BIT(1)
#define  RPF_VLAN_MODE_ACCEPT_UNTAGGED		__BIT(2)
#define  RPF_VLAN_MODE_UNTAGGED_ACTION		__BITS(5,3)

#define RPF_VLAN_TPID_REG			0x5284
#define  RPF_VLAN_TPID_OUTER			__BITS(31,16)
#define  RPF_VLAN_TPID_INNER			__BITS(15,0)

#define RPF_VLAN_FILTER_REG(f)			(0x5290 + (f) * 4)	/* RPF_VLAN_FILTER_REG[16] */
#define  RPF_VLAN_FILTER_EN			__BIT(31)
#define  RPF_VLAN_FILTER_RXQ_EN			__BIT(28)
#define  RPF_VLAN_FILTER_RXQ			__BITS(24,20)
#define  RPF_VLAN_FILTER_ACTION			__BITS(18,16)
#define  RPF_VLAN_FILTER_ID			__BITS(11,0)

#define RPF_ETHERTYPE_FILTER_REG(n)		(0x5300 + (n) * 4)
#define  RPF_ETHERTYPE_FILTER_EN		__BIT(31)
#define  RPF_ETHERTYPE_FILTER_UPF_EN		__BIT(30)	/* UPF: user priority filter */
#define  RPF_ETHERTYPE_FILTER_RXQF_EN		__BIT(29)
#define  RPF_ETHERTYPE_FILTER_UPF		__BITS(28,26)
#define  RPF_ETHERTYPE_FILTER_RXQF		__BITS(24,20)
#define  RPF_ETHERTYPE_FILTER_MNG_RXQF		__BIT(19)
#define  RPF_ETHERTYPE_FILTER_ACTION		__BITS(18,16)
#define  RPF_ETHERTYPE_FILTER_VAL		__BITS(15,0)

#define RPF_L3_FILTER_REG(f)			(0x5380 + (f) * 4)	/* RPF_L3_FILTER_REG[8] */
#define  RPF_L3_FILTER_L4_EN			__BIT(31)
#define  RPF_L3_FILTER_IPV6_EN			__BIT(30)
#define  RPF_L3_FILTER_SRCADDR_EN		__BIT(29)
#define  RPF_L3_FILTER_DSTADDR_EN		__BIT(28)
#define  RPF_L3_FILTER_L4_SRCPORT_EN		__BIT(27)
#define  RPF_L3_FILTER_L4_DSTPORT_EN		__BIT(26)
#define  RPF_L3_FILTER_L4_PROTO_EN		__BIT(25)
#define  RPF_L3_FILTER_ARP_EN			__BIT(24)
#define  RPF_L3_FILTER_L4_RXQUEUE_EN		__BIT(23)
#define  RPF_L3_FILTER_L4_RXQUEUE_MANAGEMENT_EN	__BIT(22)
#define  RPF_L3_FILTER_L4_ACTION		__BITS(16,18)
#define  RPF_L3_FILTER_L4_RXQUEUE		__BITS(12,8)
#define  RPF_L3_FILTER_L4_PROTO			__BITS(2,0)
#define   RPF_L3_FILTER_L4_PROTF_TCP		0
#define   RPF_L3_FILTER_L4_PROTF_UDP		1
#define   RPF_L3_FILTER_L4_PROTF_SCTP		2
#define   RPF_L3_FILTER_L4_PROTF_ICMP		3

#define RPF_L3_FILTER_SRCADDR_REG(f)		(0x53b0 + (f) * 4)
#define RPF_L3_FILTER_DSTADDR_REG(f)		(0x53d0 + (f) * 4)
#define RPF_L3_FILTER_L4_SRCPORT_REG(f)		(0x5400 + (f) * 4)
#define RPF_L3_FILTER_L4_DSTPORT_REG(f)		(0x5420 + (f) * 4)

#define RX_FLR_RSS_CONTROL1_REG			0x54c0
#define  RX_FLR_RSS_CONTROL1_EN			__BIT(31)

#define RPF_RPB_RX_TC_UPT_REG			0x54c4
#define  RPF_RPB_RX_TC_UPT_MASK(tc)		(0x00000007 << ((tc) * 4))

#define RPF_RSS_KEY_ADDR_REG			0x54d0
#define  RPF_RSS_KEY_ADDR			__BITS(4,0)
#define  RPF_RSS_KEY_WR_EN			__BIT(5)
#define RPF_RSS_KEY_WR_DATA_REG			0x54d4
#define RPF_RSS_KEY_RD_DATA_REG			0x54d8

#define RPF_RSS_REDIR_ADDR_REG			0x54e0
#define  RPF_RSS_REDIR_ADDR			__BITS(3,0)
#define  RPF_RSS_REDIR_WR_EN			__BIT(4)

#define RPF_RSS_REDIR_WR_DATA_REG		0x54e4
#define  RPF_RSS_REDIR_WR_DATA			__BITS(15,0)

#define RPO_HWCSUM_REG				0x5580
#define  RPO_HWCSUM_IP4CSUM_EN			__BIT(1)
#define  RPO_HWCSUM_L4CSUM_EN			__BIT(0)	/* TCP/UDP */

#define RPO_LRO_ENABLE_REG			0x5590

#define RPO_LRO_CONF_REG			0x5594
#define  RPO_LRO_CONF_QSESSION_LIMIT		__BITS(13,12)
#define  RPO_LRO_CONF_TOTAL_DESC_LIMIT		__BITS(6,5)
#define  RPO_LRO_CONF_PATCHOPTIMIZATION_EN	__BIT(15)
#define  RPO_LRO_CONF_MIN_PAYLOAD_OF_FIRST_PKT	__BITS(4,0)
#define RPO_LRO_RSC_MAX_REG			0x5598
#define RPO_LRO_LDES_MAX_REG(i)			(0x55a0 + (i / 8) * 4)
#define  RPO_LRO_LDES_MAX_MASK(i)		(0x00000003 << ((i & 7) * 4))
#define RPO_LRO_TB_DIV_REG			0x5620
#define  RPO_LRO_TB_DIV				__BITS(20,31)
#define RPO_LRO_INACTIVE_IVAL_REG		0x5620
#define  RPO_LRO_INACTIVE_IVAL			__BITS(10,19)
#define RPO_LRO_MAX_COALESCING_IVAL_REG		0x5620
#define  RPO_LRO_MAX_COALESCING_IVAL		__BITS(9,0)

#define RPB_RPF_RX_REG				0x5700
#define  RPB_RPF_RX_TC_MODE			__BIT(8)
#define  RPB_RPF_RX_FC_MODE			__BITS(5,4)
#define  RPB_RPF_RX_BUF_EN			__BIT(0)

#define RPB_RXBBUF_SIZE_REG(n)			(0x5710 + (n) * 0x10)
#define  RPB_RXBBUF_SIZE			__BITS(8,0)

#define RPB_RXB_XOFF_REG(n)			(0x5714 + (n) * 0x10)
#define  RPB_RXB_XOFF_EN			__BIT(31)
#define  RPB_RXB_XOFF_THRESH_HI			__BITS(29,16)
#define  RPB_RXB_XOFF_THRESH_LO			__BITS(13,0)


#define RX_DMA_DESC_CACHE_INIT_REG		0x5a00
#define  RX_DMA_DESC_CACHE_INIT			__BIT(0)

#define RX_DMA_INT_DESC_WRWB_EN_REG		0x05a30
#define  RX_DMA_INT_DESC_WRWB_EN		__BIT(2)
#define  RX_DMA_INT_DESC_MODERATE_EN		__BIT(3)

#define RX_INTR_MODERATION_CTL_REG(n)		(0x5a40 + (n) * 4)
#define  RX_INTR_MODERATION_CTL_EN		__BIT(1)
#define  RX_INTR_MODERATION_CTL_MIN		__BITS(15,8)
#define  RX_INTR_MODERATION_CTL_MAX		__BITS(16,24)

#define RX_DMA_DESC_BASE_ADDRLSW_REG(n)		(0x5b00 + (n) * 0x20)
#define RX_DMA_DESC_BASE_ADDRMSW_REG(n)		(0x5b04 + (n) * 0x20)
#define RX_DMA_DESC_REG(n)			(0x5b08 + (n) * 0x20)
#define  RX_DMA_DESC_LEN			__BITS(12,3)	/* RXD_NUM/8 */
#define  RX_DMA_DESC_RESET			__BIT(25)
#define  RX_DMA_DESC_HEADER_SPLIT		__BIT(28)
#define  RX_DMA_DESC_VLAN_STRIP			__BIT(29)
#define  RX_DMA_DESC_EN				__BIT(31)

#define RX_DMA_DESC_HEAD_PTR_REG(n)		(0x5b0c + (n) * 0x20)
#define  RX_DMA_DESC_HEAD_PTR			__BITS(12,0)
#define RX_DMA_DESC_TAIL_PTR_REG(n)		(0x5b10 + (n) * 0x20)

#define RX_DMA_DESC_BUFSIZE_REG(n)		(0x5b18 + (n) * 0x20)
#define  RX_DMA_DESC_BUFSIZE_DATA		__BITS(4,0)
#define  RX_DMA_DESC_BUFSIZE_HDR		__BITS(12,8)

#define RDM_DCAD_REG(n)				(0x6100 + (n) * 4)
#define  RDM_DCAD_CPUID				__BITS(7,0)
#define  RDM_DCAD_PAYLOAD_EN			__BIT(29)
#define  RDM_DCAD_HEADER_EN			__BIT(30)
#define  RDM_DCAD_DESC_EN			__BIT(31)

#define RX_DMA_DCA_REG				0x6180
#define  RX_DMA_DCA_EN				__BIT(31)
#define  RX_DMA_DCA_MODE			__BITS(3,0)

/* counters */
#define RX_DMA_GOOD_PKT_COUNTERLSW		0x6800
#define RX_DMA_GOOD_OCTET_COUNTERLSW		0x6808
#define RX_DMA_DROP_PKT_CNT_REG			0x6818
#define RX_DMA_COALESCED_PKT_CNT_REG		0x6820

#define TX_SYSCONTROL_REG			0x7000
#define  TPB_DMA_SYS_LOOPBACK			__BIT(6)
#define  TPB_TPO_PKT_SYS_LOOPBACK		__BIT(7)
#define  TX_REG_RESET_DIS			__BIT(29)

#define TX_TPO2_REG				0x7040
#define  TX_TPO2_EN				__BIT(16)

#define TPS_DESC_VM_ARB_MODE_REG		0x7300
#define  TPS_DESC_VM_ARB_MODE			__BIT(0)
#define TPS_DESC_RATE_REG			0x7310
#define  TPS_DESC_RATE_TA_RST			__BIT(31)
#define  TPS_DESC_RATE_LIM			__BITS(10,0)
#define TPS_DESC_TC_ARB_MODE_REG		0x7200
#define  TPS_DESC_TC_ARB_MODE			__BITS(1,0)
#define TPS_DATA_TC_ARB_MODE_REG		0x7100
#define  TPS_DATA_TC_ARB_MODE			__BIT(0)
#define TPS_DATA_TCTCREDIT_MAX_REG(tc)		(0x7110 + (tc) * 4)
#define  TPS_DATA_TCTCREDIT_MAX			__BITS(16,27)
#define TPS_DATA_TCTWEIGHT_REG(tc)		TPS_DATA_TCTCREDIT_MAX_REG(tc)
#define  TPS_DATA_TCTWEIGHT			__BITS(8,0)
#define TPS_DESC_TCTCREDIT_MAX_REG(tc)		(0x7210 + (tc) * 4)
#define  TPS_DESC_TCTCREDIT_MAX			__BITS(16,27)
#define TPS_DESC_TCTWEIGHT_REG(tc)		TPS_DESC_TCTCREDIT_MAX_REG(tc)
#define  TPS_DESC_TCTWEIGHT			__BITS(8,0)

#define AQ_HW_TXBUF_MAX		160
#define AQ_HW_RXBUF_MAX		320

#define TPO_HWCSUM_REG				0x7800
#define  TPO_HWCSUM_IP4CSUM_EN			__BIT(1)
#define  TPO_HWCSUM_L4CSUM_EN			__BIT(0)	/* TCP/UDP */

#define TDM_LSO_EN_REG				0x7810

#define THM_LSO_TCP_FLAG1_REG			0x7820
#define  THM_LSO_TCP_FLAG1_FIRST		__BITS(11,0)
#define  THM_LSO_TCP_FLAG1_MID			__BITS(27,16)
#define THM_LSO_TCP_FLAG2_REG			0x7824
#define  THM_LSO_TCP_FLAG2_LAST			__BITS(11,0)

#define TPB_TX_BUF_REG				0x7900
#define  TPB_TX_BUF_EN				__BIT(0)
#define  TPB_TX_BUF_SCP_INS_EN			__BIT(2)
#define  TPB_TX_BUF_TC_MODE_EN			__BIT(8)

#define TPB_TXBBUF_SIZE_REG(buffer)		(0x7910 + (buffer) * 0x10)
#define  TPB_TXBBUF_SIZE			__BITS(7,0)
#define TPB_TXB_THRESH_REG(buffer)		(0x7914 + (buffer) * 0x10)
#define  TPB_TXB_THRESH_HI			__BITS(16,28)
#define  TPB_TXB_THRESH_LO			__BITS(12,0)

#define AQ_HW_TX_DMA_TOTAL_REQ_LIMIT_REG	0x7b20
#define TX_DMA_INT_DESC_WRWB_EN_REG		0x7b40
#define  TX_DMA_INT_DESC_WRWB_EN		__BIT(1)
#define  TX_DMA_INT_DESC_MODERATE_EN		__BIT(4)

#define TX_DMA_DESC_BASE_ADDRLSW_REG(n)		(0x7c00 + (n) * 64)
#define TX_DMA_DESC_BASE_ADDRMSW_REG(n)		(0x7c04 + (n) * 64)
#define TX_DMA_DESC_REG(n)			(0x7c08 + (n) * 64)
#define  TX_DMA_DESC_LEN			__BITS(12, 3)	/* TXD_NUM/8 */
#define  TX_DMA_DESC_EN				__BIT(31)
#define TX_DMA_DESC_HEAD_PTR_REG(n)		(0x7c0c + (n) * 64)	/* index of desc */
#define  TX_DMA_DESC_HEAD_PTR			__BITS(12,0)
#define TX_DMA_DESC_TAIL_PTR_REG(n)		(0x7c10 + (n) * 64)	/* index of desc */

#define TX_DMA_DESC_WRWB_THRESH_REG(n)		(0x7c18 + (n) * 64)
#define  TX_DMA_DESC_WRWB_THRESH		__BITS(14,8)

#define TDM_DCAD_REG(n)				(0x8400 + (n) * 4)
#define  TDM_DCAD_CPUID				__BITS(7,0)
#define  TDM_DCAD_CPUID_EN			__BIT(31)

#define TDM_DCA_REG				0x8480
#define  TDM_DCA_EN				__BIT(31)
#define  TDM_DCA_MODE				__BITS(3,0)

#define TX_INTR_MODERATION_CTL_REG(n)		(0x8980 + (n) * 4)
#define  TX_INTR_MODERATION_CTL_EN		__BIT(1)
#define  TX_INTR_MODERATION_CTL_MIN		__BITS(15,8)
#define  TX_INTR_MODERATION_CTL_MAX		__BITS(16,24)

#define FW2X_CTRL_10BASET_HD			__BIT(0)
#define FW2X_CTRL_10BASET_FD			__BIT(1)
#define FW2X_CTRL_100BASETX_HD			__BIT(2)
#define FW2X_CTRL_100BASET4_HD			__BIT(3)
#define FW2X_CTRL_100BASET2_HD			__BIT(4)
#define FW2X_CTRL_100BASETX_FD			__BIT(5)
#define FW2X_CTRL_100BASET2_FD			__BIT(6)
#define FW2X_CTRL_1000BASET_HD			__BIT(7)
#define FW2X_CTRL_1000BASET_FD			__BIT(8)
#define FW2X_CTRL_2P5GBASET_FD			__BIT(9)
#define FW2X_CTRL_5GBASET_FD			__BIT(10)
#define FW2X_CTRL_10GBASET_FD			__BIT(11)
#define FW2X_CTRL_RESERVED1			__BIT(32)
#define FW2X_CTRL_10BASET_EEE			__BIT(33)
#define FW2X_CTRL_RESERVED2			__BIT(34)
#define FW2X_CTRL_PAUSE				__BIT(35)
#define FW2X_CTRL_ASYMMETRIC_PAUSE		__BIT(36)
#define FW2X_CTRL_100BASETX_EEE			__BIT(37)
#define FW2X_CTRL_RESERVED3			__BIT(38)
#define FW2X_CTRL_RESERVED4			__BIT(39)
#define FW2X_CTRL_1000BASET_FD_EEE		__BIT(40)
#define FW2X_CTRL_2P5GBASET_FD_EEE		__BIT(41)
#define FW2X_CTRL_5GBASET_FD_EEE		__BIT(42)
#define FW2X_CTRL_10GBASET_FD_EEE		__BIT(43)
#define FW2X_CTRL_RESERVED5			__BIT(44)
#define FW2X_CTRL_RESERVED6			__BIT(45)
#define FW2X_CTRL_RESERVED7			__BIT(46)
#define FW2X_CTRL_RESERVED8			__BIT(47)
#define FW2X_CTRL_RESERVED9			__BIT(48)
#define FW2X_CTRL_CABLE_DIAG			__BIT(49)
#define FW2X_CTRL_TEMPERATURE			__BIT(50)
#define FW2X_CTRL_DOWNSHIFT			__BIT(51)
#define FW2X_CTRL_PTP_AVB_EN			__BIT(52)
#define FW2X_CTRL_MEDIA_DETECT			__BIT(53)
#define FW2X_CTRL_LINK_DROP			__BIT(54)
#define FW2X_CTRL_SLEEP_PROXY			__BIT(55)
#define FW2X_CTRL_WOL				__BIT(56)
#define FW2X_CTRL_MAC_STOP			__BIT(57)
#define FW2X_CTRL_EXT_LOOPBACK			__BIT(58)
#define FW2X_CTRL_INT_LOOPBACK			__BIT(59)
#define FW2X_CTRL_EFUSE_AGENT			__BIT(60)
#define FW2X_CTRL_WOL_TIMER			__BIT(61)
#define FW2X_CTRL_STATISTICS			__BIT(62)
#define FW2X_CTRL_TRANSACTION_ID		__BIT(63)

#define FW2X_CTRL_RATE_100M			FW2X_CTRL_100BASETX_FD
#define FW2X_CTRL_RATE_1G			FW2X_CTRL_1000BASET_FD
#define FW2X_CTRL_RATE_2G5			FW2X_CTRL_2P5GBASET_FD
#define FW2X_CTRL_RATE_5G			FW2X_CTRL_5GBASET_FD
#define FW2X_CTRL_RATE_10G			FW2X_CTRL_10GBASET_FD
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

#define WAIT_FOR(expr, us, n, errp)				\
	do {							\
		unsigned int _n;				\
		for (_n = n; (!(expr)) && _n != 0; --_n) {	\
			delay((us));				\
		}						\
		if ((errp != NULL)) {				\
			if (_n == 0)				\
				*(errp) = ETIMEDOUT;		\
			else					\
				*(errp) = 0;			\
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

typedef enum aq_link_speed {
	AQ_LINK_NONE	= 0,
	AQ_LINK_100M	= (1 << 0),
	AQ_LINK_1G	= (1 << 1),
	AQ_LINK_2G5	= (1 << 2),
	AQ_LINK_5G	= (1 << 3),
	AQ_LINK_10G	= (1 << 4)
} aq_link_speed_t;
#define AQ_LINK_ALL	(AQ_LINK_100M | AQ_LINK_1G | AQ_LINK_2G5 | \
			 AQ_LINK_5G | AQ_LINK_10G )
#define AQ_LINK_AUTO	AQ_LINK_ALL

typedef enum aq_link_fc {
	AQ_FC_NONE = 0,
	AQ_FC_RX = __BIT(0),
	AQ_FC_TX = __BIT(1),
	AQ_FC_ALL = (AQ_FC_RX | AQ_FC_TX)
} aq_link_fc_t;

typedef enum aq_link_eee {
	AQ_EEE_DISABLE = 0,
	AQ_EEE_ENABLE = 1
} aq_link_eee_t;

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
#define RXDESC_TYPE_RSSTYPE		__BITS(3,0)
#define  RXDESC_TYPE_RSSTYPE_NONE		0
#define  RXDESC_TYPE_RSSTYPE_IPV4		2
#define  RXDESC_TYPE_RSSTYPE_IPV6		3
#define  RXDESC_TYPE_RSSTYPE_IPV4_TCP		4
#define  RXDESC_TYPE_RSSTYPE_IPV6_TCP		5
#define  RXDESC_TYPE_RSSTYPE_IPV4_UDP		6
#define  RXDESC_TYPE_RSSTYPE_IPV6_UDP		7
#define RXDESC_TYPE_PKTTYPE_ETHER	__BITS(5,4)
#define  RXDESC_TYPE_PKTTYPE_ETHER_IPV4		0
#define  RXDESC_TYPE_PKTTYPE_ETHER_IPV6		1
#define  RXDESC_TYPE_PKTTYPE_ETHER_OTHERS	2
#define  RXDESC_TYPE_PKTTYPE_ETHER_ARP		3
#define RXDESC_TYPE_PKTTYPE_PROTO	__BITS(8,6)
#define  RXDESC_TYPE_PKTTYPE_PROTO_TCP		0
#define  RXDESC_TYPE_PKTTYPE_PROTO_UDP		1
#define  RXDESC_TYPE_PKTTYPE_PROTO_SCTP		2
#define  RXDESC_TYPE_PKTTYPE_PROTO_ICMP		3
#define  RXDESC_TYPE_PKTTYPE_PROTO_OTHERS	4
#define RXDESC_TYPE_PKTTYPE_VLAN	__BIT(9)
#define RXDESC_TYPE_PKTTYPE_VLAN_DOUBLE	__BIT(10)
#define RXDESC_TYPE_RDM_ERR		__BIT(12)
#define RXDESC_TYPE_RESERVED		__BITS(18,13)
#define RXDESC_TYPE_IPV4_CSUM_CHECKED	__BIT(19)	// (PKTTYPE_L3 == 0)
#define RXDESC_TYPE_TCPUDP_CSUM_CHECKED	__BIT(20)
#define RXDESC_TYPE_SPH			__BIT(21)
#define RXDESC_TYPE_HDR_LEN		__BITS(31,22)
	uint32_t rss_hash;
	uint16_t status;
#define RXDESC_STATUS_DD		__BIT(0)
#define RXDESC_STATUS_EOP		__BIT(1)
#define RXDESC_STATUS_MAC_DMA_ERR	__BIT(2)
#define RXDESC_STATUS_L3_CSUM_NG	__BIT(3)
#define RXDESC_STATUS_L4_CSUM_ERROR	__BIT(4)
#define RXDESC_STATUS_L4_CSUM_OK	__BIT(5)

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
	uint32_t ctl1;
#define AQ_TXDESC_CTL1_TYPE_MASK	0x00000003
#define AQ_TXDESC_CTL1_TYPE_TXD		0x00000001
#define AQ_TXDESC_CTL1_TYPE_TXC		0x00000002
#define AQ_TXDESC_CTL1_BLEN		__BITS(19,4)	/* TXD */
#define AQ_TXDESC_CTL1_DD		__BIT(20)	/* TXD */
#define AQ_TXDESC_CTL1_EOP		__BIT(21)	/* TXD */
#define AQ_TXDESC_CTL1_CMD_VLAN		__BIT(22)	/* TXD */
#define AQ_TXDESC_CTL1_CMD_FCS		__BIT(23)	/* TXD */
#define AQ_TXDESC_CTL1_CMD_IP4CSUM	__BIT(24)	/* TXD */
#define AQ_TXDESC_CTL1_CMD_L4CSUM	__BIT(25)	/* TXD */
#define AQ_TXDESC_CTL1_CMD_LSO		__BIT(26)	/* TXD */
#define AQ_TXDESC_CTL1_CMD_WB		__BIT(27)	/* TXD */
#define AQ_TXDESC_CTL1_CMD_VXLAN	__BIT(28)	/* TXD */
#define AQ_TXDESC_CTL1_VID		__BITS(15,4)	/* TXC */
#define AQ_TXDESC_CTL1_LSO_IPV6		__BIT(21)	/* TXC */
#define AQ_TXDESC_CTL1_LSO_TCP		__BIT(22)	/* TXC */
	uint32_t ctl2;
#define AQ_TXDESC_CTL2_LEN		__BITS(31,14)
#define AQ_TXDESC_CTL2_CTX_EN		__BIT(13)
#define AQ_TXDESC_CTL2_CTX_IDX		__BIT(12)
} __packed aq_tx_desc_t;

/* hardware restrictions */
#define AQ_RINGS_MAX	32
#define AQ_RXD_MIN	32
#define AQ_TXD_MIN	32
#define AQ_RXD_MAX	8184	/* = 0x1ff8 = 8192 - 8 */
#define AQ_TXD_MAX	8184	/* = 0x1ff8 = 8192 - 8 */

/* configuration for this driver */
#define AQ_TXRING_NUM	8
#define AQ_RXRING_NUM	8
#define AQ_TXD_NUM	2048	/* per ring. must be 8*n */
#define AQ_RXD_NUM	2048	/* per ring. must be 8*n */

#define LINKSTAT_IRQ	31	/* shared with ring[31] */


struct aq_txring {
	struct aq_softc *ring_sc;
	kmutex_t ring_mutex;
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
	kmutex_t ring_mutex;
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
	    aq_link_speed_t, aq_link_fc_t, aq_link_eee_t);
	int (*get_mode)(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
	    aq_link_speed_t *, aq_link_fc_t *, aq_link_eee_t *);
	int (*get_stats)(struct aq_softc *, aq_hw_stats_s_t *);
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
	aq_link_speed_t sc_available_rates;

	aq_link_speed_t sc_link_rate;
	aq_link_fc_t sc_link_fc;
	aq_link_eee_t sc_link_eee;

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

	bool sc_intr_moderation_enable;
	bool sc_rss_enable;
	bool sc_offload_enable;
	bool sc_l3_filter_enable;

	uint32_t sc_rss_key[AQ_RSS_HASHKEY_SIZE / sizeof(uint32_t)];
	uint8_t sc_rss_table[AQ_RSS_INDIRECTION_TABLE_MAX];

	int sc_media_active;

#ifdef USE_CALLOUT_TICK
	callout_t sc_tick_ch;
#endif
	struct ethercom sc_ethercom;
	struct ether_addr sc_enaddr;
	struct ifmedia sc_media;
	int sc_ec_capenable;		/* last ec_capenable */
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

static int aq_txrx_rings_alloc(struct aq_softc *);
static void aq_txrx_rings_free(struct aq_softc *);

static void aq_initmedia(struct aq_softc *);
static int aq_mediachange(struct ifnet *);
static void aq_enable_intr(struct aq_softc *, bool, bool);

#ifdef USE_CALLOUT_TICK
static void aq_tick(void *);
#endif
static int aq_intr(void *);
#ifdef  XXX_INTR_DEBUG
static int aq_tx_intr_poll(struct aq_txring *);
static int aq_rx_intr_poll(struct aq_rxring *);
#endif
static int aq_tx_intr(struct aq_txring *);
static int aq_rx_intr(struct aq_rxring *);

static int aq_set_linkmode(struct aq_softc *, aq_link_speed_t, aq_link_fc_t, aq_link_eee_t);
static int aq_get_linkmode(struct aq_softc *, aq_link_speed_t *, aq_link_fc_t *, aq_link_eee_t *);

static int aq_fw_reset(struct aq_softc *);
static int aq_fw_version_init(struct aq_softc *);
static int aq_hw_init(struct aq_softc *);
static int aq_hw_init_ucp(struct aq_softc *);
static int aq_hw_reset(struct aq_softc *);
static int aq_fw_downld_dwords(struct aq_softc *, uint32_t, uint32_t *, uint32_t);
static int aq_get_mac_addr(struct aq_softc *);
static void aq_init_rsstable(struct aq_softc *);
static int aq_set_capability(struct aq_softc *);

static int fw1x_reset(struct aq_softc *);
static int fw1x_set_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
    aq_link_speed_t, aq_link_fc_t, aq_link_eee_t);
static int fw1x_get_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
    aq_link_speed_t *, aq_link_fc_t *, aq_link_eee_t *);
static int fw1x_get_stats(struct aq_softc *, aq_hw_stats_s_t *);

static int fw2x_reset(struct aq_softc *);
static int fw2x_set_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t,
    aq_link_speed_t, aq_link_fc_t, aq_link_eee_t);
static int fw2x_get_mode(struct aq_softc *, aq_hw_fw_mpi_state_e_t *,
    aq_link_speed_t *, aq_link_fc_t *, aq_link_eee_t *);
static int fw2x_get_stats(struct aq_softc *, aq_hw_stats_s_t *);

static struct aq_firmware_ops aq_fw1x_ops = {
	.reset = fw1x_reset,
	.set_mode = fw1x_set_mode,
	.get_mode = fw1x_get_mode,
	.get_stats = fw1x_get_stats
};

static struct aq_firmware_ops aq_fw2x_ops = {
	.reset = fw2x_reset,
	.set_mode = fw2x_set_mode,
	.get_mode = fw2x_get_mode,
	.get_stats = fw2x_get_stats
};

CFATTACH_DECL3_NEW(aq, sizeof(struct aq_softc),
    aq_match, aq_attach, aq_detach, NULL, NULL, NULL, DVF_DETACH_SHUTDOWN);

static const struct aq_product {
	pci_vendor_id_t aq_vendor;
	pci_product_id_t aq_product;
	const char *aq_name;
	enum aq_media_type aq_media_type;
	aq_link_speed_t aq_available_rates;
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

	sc->sc_intr_moderation_enable = CONFIG_INTR_MODERATION_ENABLE;
	sc->sc_rss_enable = CONFIG_RSS_ENABLE;
	sc->sc_offload_enable = CONFIG_OFFLOAD_ENABLE;
	sc->sc_l3_filter_enable = CONFIG_L3_FILTER_ENABLE;

	sc->sc_txringnum = AQ_TXRING_NUM;
	sc->sc_rxringnum = AQ_RXRING_NUM;
	sc->sc_ringnum = MAX(sc->sc_txringnum, sc->sc_rxringnum);
	error = aq_txrx_rings_alloc(sc);
	if (error != 0)
		goto attach_failure;

	error = aq_fw_reset(sc);
	if (error != 0)
		goto attach_failure;

	error = aq_fw_version_init(sc);
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
	aq_init_rsstable(sc);

	error = aq_hw_init(sc);	/* initialize and interrupts */
	if (error != 0)
		goto attach_failure;


	sc->sc_media_type = aqp->aq_media_type;
	sc->sc_available_rates = aqp->aq_available_rates;

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

	/* initialize capabilities */
	sc->sc_ethercom.ec_capabilities = 0;
	sc->sc_ethercom.ec_capenable = 0;
#if notyet
	// XXX: NOTYET
	sc->sc_ethercom.ec_capabilities |= ETHERCAP_JUMBO_MTU;
	sc->sc_ethercom.ec_capabilities |= ETHERCAP_EEE;
	sc->sc_ethercom.ec_capabilities |= ETHERCAP_VLAN_HWFILTER;
#endif
	sc->sc_ethercom.ec_capabilities |=
	    ETHERCAP_VLAN_MTU |
	    ETHERCAP_VLAN_HWTAGGING;
	sc->sc_ethercom.ec_capenable |=
	    ETHERCAP_VLAN_HWTAGGING;

	ifp->if_capabilities = 0;
	ifp->if_capenable = 0;
#ifdef CONFIG_LRO_SUPPORT
	ifp->if_capabilities |= IFCAP_LRO;
#endif
#if notyet
	// TSO
	ifp->if_capabilities |= IFCAP_TSOv4 | IFCAP_TSOv6;
#endif
#if notyet
	// XXX: RX L4 CSUM doesn't work for fragment packet... RX L4 CSUM is requied for LRO?
	ifp->if_capabilities |= IFCAP_CSUM_TCPv4_Rx | IFCAP_CSUM_TCPv6_Rx;
	ifp->if_capabilities |= IFCAP_CSUM_UDPv4_Rx | IFCAP_CSUM_UDPv6_Rx;
#endif
	/* TX hardware checksum offloadding */
	ifp->if_capabilities |= IFCAP_CSUM_IPv4_Tx;
	ifp->if_capabilities |= IFCAP_CSUM_TCPv4_Tx | IFCAP_CSUM_TCPv6_Tx;
	ifp->if_capabilities |= IFCAP_CSUM_UDPv4_Tx | IFCAP_CSUM_UDPv6_Tx;
	/* RX hardware checksum offloadding */
	ifp->if_capabilities |= IFCAP_CSUM_IPv4_Rx;

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr.ether_addr_octet);
	ether_set_ifflags_cb(&sc->sc_ethercom, aq_ifflags_cb);

	aq_enable_intr(sc, true, false);	/* only intr about link */

	/* media update */
	aq_mediachange(ifp);

	/* get starting statistics values */
	if (sc->sc_fw_ops != NULL && sc->sc_fw_ops->get_stats != NULL &&
	    sc->sc_fw_ops->get_stats(sc, &sc->sc_statistics[0]) == 0) {
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
global_software_reset(struct aq_softc *sc)
{
	uint32_t v;

	AQ_WRITE_REG_BIT(sc, RX_SYSCONTROL_REG, RX_REG_RESET_DIS, 0);
	AQ_WRITE_REG_BIT(sc, TX_SYSCONTROL_REG, TX_REG_RESET_DIS, 0);
	AQ_WRITE_REG_BIT(sc, MPI_RESETCTRL_REG, MPI_RESETCTRL_RESET_DIS, 0);

	v = AQ_READ_REG(sc, AQ_SOFTRESET_REG);
	v &= ~AQ_SOFTRESET_DISABLE;
	v |= AQ_SOFTRESET_RESET;
	AQ_WRITE_REG(sc, AQ_SOFTRESET_REG, v);
}

static int
mac_soft_reset_rbl(struct aq_softc *sc, aq_fw_bootloader_mode_t *mode)
{
	int timo;

	aprint_debug_dev(sc->sc_dev, "RBL> MAC reset STARTED!\n");

	AQ_WRITE_REG(sc, FW_GLB_CTL2_REG, 0x40e1);
	AQ_WRITE_REG(sc, FW_GLB_CPU_SEM_REG(0), 1);
	AQ_WRITE_REG(sc, AQ_MBOXIF_POWER_GATING_CONTROL_REG, 0);

	/* MAC FW will reload PHY FW if 1E.1000.3 was cleaned - #undone */
	AQ_WRITE_REG(sc, FW_BOOT_EXIT_CODE_REG, RBL_STATUS_DEAD);

	global_software_reset(sc);

	AQ_WRITE_REG(sc, FW_GLB_CTL2_REG, 0x40e0);

	/* Wait for RBL to finish boot process. */
#define RBL_TIMEOUT_MS	10000
	uint16_t rbl_status;
	for (timo = RBL_TIMEOUT_MS; timo > 0; timo--) {
		rbl_status =
		    AQ_READ_REG(sc, FW_BOOT_EXIT_CODE_REG) & 0xffff;
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

	AQ_WRITE_REG(sc, FW_GLB_CTL2_REG, 0x40e1);
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
	AQ_WRITE_REG(sc, FW_GLB_NVR_PROVISIONING2_REG, 0x00a0);
	AQ_WRITE_REG(sc, FW_GLB_NVR_INTERFACE1_REG, 0x009f);
	AQ_WRITE_REG(sc, FW_GLB_NVR_INTERFACE1_REG, 0x809f);
	msec_delay(50);

	v = AQ_READ_REG(sc, AQ_SOFTRESET_REG);
	v &= ~AQ_SOFTRESET_DISABLE;
	v |= AQ_SOFTRESET_RESET;
	AQ_WRITE_REG(sc, AQ_SOFTRESET_REG, v);

	/* Kickstart. */
	AQ_WRITE_REG(sc, FW_GLB_CTL2_REG, 0x80e0);
	AQ_WRITE_REG(sc, AQ_MBOXIF_POWER_GATING_CONTROL_REG, 0);
	if (!sc->sc_fast_start_enabled)
		AQ_WRITE_REG(sc, FW_GLB_GENERAL_PROVISIONING9_REG, 1);

	/*
	 * For the case SPI burst transaction was interrupted (by MCP reset
	 * above), wait until it is completed by hardware.
	 */
	msec_delay(50);

	/* MAC Kickstart */
	if (!sc->sc_fast_start_enabled) {
		AQ_WRITE_REG(sc, FW_GLB_CTL2_REG, 0x180e0);

		uint32_t flb_status;
		for (timo = 0; timo < 1000; timo++) {
			flb_status = AQ_READ_REG(sc,
			    FW_DAISY_CHAIN_STATUS_REG) & 0x10;
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
		AQ_WRITE_REG(sc, FW_GLB_CTL2_REG, 0x80e0);
		/*
		 * Let Felicity hardware complete SMBUS transaction before
		 * Global software reset.
		 */
		msec_delay(50);
	}
	AQ_WRITE_REG(sc, FW_GLB_CPU_SEM_REG(0), 1);

	/* PHY Kickstart: #undone */
	global_software_reset(sc);

	for (timo = 0; timo < 1000; timo++) {
		if (AQ_READ_REG(sc, AQ_FW_VERSION_REG) != 0)
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
aq_fw_read_version(struct aq_softc *sc)
{
	int i, error = EBUSY;
#define MAC_FW_START_TIMEOUT_MS	10000
	for (i = 0; i < MAC_FW_START_TIMEOUT_MS; i++) {
		sc->sc_fw_version = AQ_READ_REG(sc, AQ_FW_VERSION_REG);
		if (sc->sc_fw_version != 0) {
			error = 0;
			break;
		}
		delay(1000);
	}
	return error;
}

static int
aq_fw_reset(struct aq_softc *sc)
{
	uint32_t ver, v, bootExitCode;
	int i, error;

	ver = AQ_READ_REG(sc, AQ_FW_VERSION_REG);

	for (i = 1000; i > 0; i--) {
		v = AQ_READ_REG(sc, FW_DAISY_CHAIN_STATUS_REG);
		bootExitCode = AQ_READ_REG(sc, FW_BOOT_EXIT_CODE_REG);
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
		error = aq_fw_read_version(sc);
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
		return aq_fw_read_version(sc);
	case FW_BOOT_MODE_RBL_FLASH:
		aprint_debug_dev(sc->sc_dev,
		    "RBL> F/W loaded from flash. Host Bootload disabled.\n");
		sc->sc_flash_present = true;
		return aq_fw_read_version(sc);
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
	AQ_WRITE_REG_BIT(sc, AQ_INTR_CTRL_REG, AQ_INTR_CTRL_RESET_DIS, 0);

	/* apply */
	AQ_WRITE_REG_BIT(sc, AQ_INTR_CTRL_REG, AQ_INTR_CTRL_RESET_IRQ, 1);

	/* wait ack 10 times by 1ms */
	WAIT_FOR((AQ_READ_REG(sc, AQ_INTR_CTRL_REG) & AQ_INTR_CTRL_RESET_IRQ) == 0,
	    1000, 10, &error);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "atlantic: IRQ reset failed: %d\n", error);
		return error;
	}

	return sc->sc_fw_ops->reset(sc);
}

static int
aq_hw_init_ucp(struct aq_softc *sc)
{
	int timo;

	if (FW_VERSION_MAJOR(sc) == 1) {
		if (AQ_READ_REG(sc, FW1X_0X370_REG) == 0) {
			uint32_t data;
			cprng_fast(&data, sizeof(data));
			data &= 0xfefefefe;
			data |= 0x02020202;
			AQ_WRITE_REG(sc, FW1X_0X370_REG, data);
		}
		AQ_WRITE_REG(sc, FW_GLB_CPU_SCRATCH_SCP_REG(25), 0);
	}

	for (timo = 100; timo > 0; timo--) {
		sc->sc_mbox_addr =
		    AQ_READ_REG(sc, FW2X_MPI_MBOX_ADDR_REG);
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
aq_fw_version_init(struct aq_softc *sc)
{
	int error = 0;
	char fw_vers[sizeof("F/W version xxxxx.xxxxx.xxxxx")];

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
	snprintf(fw_vers, sizeof(fw_vers), "F/W version %d.%d.%d",
	    FW_VERSION_MAJOR(sc), FW_VERSION_MINOR(sc), FW_VERSION_BUILD(sc));

	/* detect revision */
	uint32_t hwrev = AQ_READ_REG(sc, AQ_HW_REVISION_REG);
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
    aq_link_speed_t speed, aq_link_fc_t fc, aq_link_eee_t eee)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	return ENOSYS;
}

static int
fw1x_get_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t *mode,
    aq_link_speed_t *speed, aq_link_fc_t *fc, aq_link_eee_t *eee)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	return ENOSYS;
}

static int
fw1x_get_stats(struct aq_softc *sc, aq_hw_stats_s_t *stats)
{
	printf("%s:%d: XXX: not implemented\n", __func__, __LINE__);
	return ENOSYS;
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

static int
fw2x_set_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t mode,
    aq_link_speed_t speed, aq_link_fc_t fc, aq_link_eee_t eee)
{
	uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_REG);

	switch (mode) {
	case MPI_INIT:
		mpi_ctrl &= ~FW2X_CTRL_RATE_MASK;
		if (speed & AQ_LINK_10G)
			mpi_ctrl |= FW2X_CTRL_RATE_10G;
		if (speed & AQ_LINK_5G)
			mpi_ctrl |= FW2X_CTRL_RATE_5G;
		if (speed & AQ_LINK_2G5)
			mpi_ctrl |= FW2X_CTRL_RATE_2G5;
		if (speed & AQ_LINK_1G)
			mpi_ctrl |= FW2X_CTRL_RATE_1G;
		if (speed & AQ_LINK_100M)
			mpi_ctrl |= FW2X_CTRL_RATE_100M;

		mpi_ctrl &= ~FW2X_CTRL_LINK_DROP;

		mpi_ctrl &= ~FW2X_CTRL_EEE_MASK;
		if (eee == AQ_EEE_ENABLE)
			mpi_ctrl |= FW2X_CTRL_EEE_MASK;

		mpi_ctrl &= ~(FW2X_CTRL_PAUSE | FW2X_CTRL_ASYMMETRIC_PAUSE);
		if (fc & AQ_FC_RX)
			mpi_ctrl |= FW2X_CTRL_PAUSE;
		if (fc & AQ_FC_TX)
			mpi_ctrl |= FW2X_CTRL_ASYMMETRIC_PAUSE;
		break;
	case MPI_DEINIT:
		mpi_ctrl &= ~(FW2X_CTRL_RATE_MASK | FW2X_CTRL_EEE_MASK);
		mpi_ctrl &= ~(FW2X_CTRL_PAUSE | FW2X_CTRL_ASYMMETRIC_PAUSE);
		break;
	default:
		device_printf(sc->sc_dev,
		    "fw2x> unknown MPI state %d\n", mode);
		return EINVAL;
	}

	AQ_WRITE64_REG(sc, FW2X_MPI_CONTROL_REG, mpi_ctrl);
	return 0;
}

static int
fw2x_get_mode(struct aq_softc *sc, aq_hw_fw_mpi_state_e_t *mode,
    aq_link_speed_t *speedp, aq_link_fc_t *fcp, aq_link_eee_t *eeep)
{
	uint64_t mpi_state = AQ_READ64_REG(sc, FW2X_MPI_STATE_REG);

	if (mode != NULL) {
		uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_REG);
		if (mpi_ctrl & FW2X_CTRL_RATE_MASK)
			*mode = MPI_INIT;
		else
			*mode = MPI_DEINIT;
	}

	aq_link_speed_t speed = AQ_LINK_NONE;
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

	aq_link_fc_t fc = AQ_FC_NONE;
	if (mpi_state & FW2X_CTRL_PAUSE)
		fc |= AQ_FC_RX;
	if (mpi_state & FW2X_CTRL_ASYMMETRIC_PAUSE)
		fc |= AQ_FC_TX;
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
	uint64_t mpi_ctrl = AQ_READ64_REG(sc, FW2X_MPI_CONTROL_REG);
	uint64_t mpi_state = AQ_READ64_REG(sc, FW2X_MPI_STATE_REG);
	int error;

	/* First, check that control and state values are consistent */
	if ((mpi_ctrl & mask) != (mpi_state & mask)) {
		device_printf(sc->sc_dev,
		    "fw2x> MPI control (%#llx) and state (%#llx)"
		    " are not consistent for mask %#llx!\n",
		    (unsigned long long)mpi_ctrl, (unsigned long long)mpi_state,
		    (unsigned long long)mask);
		return EINVAL;
	}

	/* Invert bits (toggle) in control register */
	mpi_ctrl ^= mask;
	AQ_WRITE64_REG(sc, FW2X_MPI_CONTROL_REG, mpi_ctrl);

	/* Clear all bits except masked */
	mpi_ctrl &= mask;

	/* Wait for FW reflecting change in state register */
	WAIT_FOR((AQ_READ64_REG(sc, FW2X_MPI_CONTROL_REG) & mask) == mpi_ctrl,
	    1000 * timeout_ms, try_count, &error);
	if (error != 0) {
		device_printf(sc->sc_dev,
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
		device_printf(sc->sc_dev,
		    "fw2x> statistics update error %d\n", error);
		return error;
	}

	CTASSERT(sizeof(fw2x_msm_statistics_t) <= sizeof(struct aq_hw_stats_s));
	error = aq_fw_downld_dwords(sc,
	    sc->sc_mbox_addr + offsetof(fw2x_mailbox_t, msm),
	    (uint32_t *)stats, sizeof(fw2x_msm_statistics_t) / sizeof(uint32_t));
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "fw2x> download statistics data FAILED, error %d", error);
		return error;
	}
	stats->dpc = AQ_READ_REG(sc, RX_DMA_DROP_PKT_CNT_REG);
	stats->cprc = AQ_READ_REG(sc, RX_DMA_COALESCED_PKT_CNT_REG);

	return 0;
}

static int
aq_fw_downld_dwords(struct aq_softc *sc, uint32_t addr, uint32_t *p,
    uint32_t cnt)
{
	uint32_t v;
	int error = 0;

	WAIT_FOR(AQ_READ_REG(sc, FW_SEM_RAM_REG) == 1,
	    1, 10000, &error);
	if (error != 0) {
		AQ_WRITE_REG(sc, FW_SEM_RAM_REG, 1);
		v = AQ_READ_REG(sc, FW_SEM_RAM_REG);
		if (v == 0) {
			device_printf(sc->sc_dev,
			    "%s:%d: timeout\n", __func__, __LINE__);
			return ETIMEDOUT;
		}
	}

	AQ_WRITE_REG(sc, AQ_FW_MBOX_ADDR_REG, addr);

	error = 0;
	for (; cnt > 0 && error == 0; cnt--) {
		/* execute mailbox interface */
		AQ_WRITE_REG_BIT(sc, AQ_FW_MBOX_CMD_REG, AQ_FW_MBOX_CMD_EXECUTE, 1);
		if (sc->sc_features & FEATURES_REV_B1) {
			WAIT_FOR(
			    AQ_READ_REG(sc, AQ_FW_MBOX_ADDR_REG) != addr,
			    1, 1000, &error);
		} else {
			WAIT_FOR(
			    (AQ_READ_REG(sc, AQ_FW_MBOX_CMD_REG) &
			    AQ_FW_MBOX_CMD_BUSY) == 0,
			    1, 1000, &error);
		}
		*p++ = AQ_READ_REG(sc, AQ_FW_MBOX_VAL_REG);
		addr += sizeof(uint32_t);
	}
	AQ_WRITE_REG(sc, FW_SEM_RAM_REG, 1);

	if (error != 0)
		device_printf(sc->sc_dev,
		    "%s:%d: timeout\n", __func__, __LINE__);

	return error;
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
		efuse_shadow_addr = AQ_READ_REG(sc, FW2X_MPI_EFUSEADDR_REG);
	else
		efuse_shadow_addr = AQ_READ_REG(sc, FW1X_MPI_EFUSEADDR_REG);

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

/* set multicast filter. index 0 for own address */
static int
aq_set_mac_addr(struct aq_softc *sc, int index, uint8_t *enaddr)
{
	uint32_t h, l;

	if (index > AQ_HW_MAC_MAX)
		return EINVAL;

	if (enaddr == NULL) {
		/* disable */
		AQ_WRITE_REG_BIT(sc,
		    RPF_L2UC_MSW_REG(index), RPF_L2UC_MSW_EN, 0);
		return 0;
	}

	h = (enaddr[0] << 8) |
	    (enaddr[1]);
	l = (enaddr[2] << 24) |
	    (enaddr[3] << 16) |
	    (enaddr[4] << 8) |
	    (enaddr[5]);

	/* disable, set, and enable */
	AQ_WRITE_REG_BIT(sc, RPF_L2UC_MSW_REG(index), RPF_L2UC_MSW_EN, 0);
	AQ_WRITE_REG(sc, RPF_L2UC_LSW_REG(index), l);
	AQ_WRITE_REG_BIT(sc, RPF_L2UC_MSW_REG(index), RPF_L2UC_MSW_MACADDR_HI, h);
	AQ_WRITE_REG_BIT(sc, RPF_L2UC_MSW_REG(index), RPF_L2UC_MSW_ACTION, 1);
	AQ_WRITE_REG_BIT(sc, RPF_L2UC_MSW_REG(index), RPF_L2UC_MSW_EN, 1);

	return 0;
}

static int
aq_set_capability(struct aq_softc *sc)
{
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	int ip4csum_tx = ((ifp->if_capenable & IFCAP_CSUM_IPv4_Tx) == 0) ? 0 : 1;
	int ip4csum_rx = ((ifp->if_capenable & IFCAP_CSUM_IPv4_Rx) == 0) ? 0 : 1;
	int l4csum_tx = ((ifp->if_capenable & (IFCAP_CSUM_TCPv4_Tx | IFCAP_CSUM_UDPv4_Tx | IFCAP_CSUM_TCPv6_Tx | IFCAP_CSUM_UDPv6_Tx)) == 0) ? 0 : 1;
	int l4csum_rx = ((ifp->if_capenable & (IFCAP_CSUM_TCPv4_Rx | IFCAP_CSUM_UDPv4_Rx | IFCAP_CSUM_TCPv6_Rx | IFCAP_CSUM_UDPv6_Rx)) == 0) ? 0 : 1;
	uint32_t lso = ((ifp->if_capenable & (IFCAP_TSOv4 | IFCAP_TSOv6)) == 0) ? 0 : 0xffffffff;
	uint32_t lro = ((ifp->if_capenable & IFCAP_LRO) == 0) ? 0 : 0xffffffff;
	uint32_t i, v;

	printf("%s:%d: ip4csum_tx=%d, ip4csum_rx=%d, L4csum_tx=%d, L4csum_rx=%d, lso=%x, lro=%x\n", __func__, __LINE__,
	    ip4csum_tx, ip4csum_rx, l4csum_tx, l4csum_rx, lso, lro);

	/* TX checksums offloads*/
	AQ_WRITE_REG_BIT(sc, TPO_HWCSUM_REG, TPO_HWCSUM_IP4CSUM_EN, ip4csum_tx);
	AQ_WRITE_REG_BIT(sc, TPO_HWCSUM_REG, TPO_HWCSUM_L4CSUM_EN, l4csum_tx);

	/* RX checksums offloads*/
	AQ_WRITE_REG_BIT(sc, RPO_HWCSUM_REG, RPO_HWCSUM_IP4CSUM_EN, ip4csum_rx);
	AQ_WRITE_REG_BIT(sc, RPO_HWCSUM_REG, RPO_HWCSUM_L4CSUM_EN, l4csum_rx);

	/* LSO offloads*/
	AQ_WRITE_REG(sc, TDM_LSO_EN_REG, lso);

#define AQ_B0_LRO_RXD_MAX	16
	v = (8 < AQ_B0_LRO_RXD_MAX) ? 3 :
	    (4 < AQ_B0_LRO_RXD_MAX) ? 2 :
	    (2 < AQ_B0_LRO_RXD_MAX) ? 1 : 0;
	for (i = 0; i < AQ_RINGS_MAX; i++) {
		AQ_WRITE_REG_BIT(sc, RPO_LRO_LDES_MAX_REG(i), RPO_LRO_LDES_MAX_MASK(i), v);
	}

	AQ_WRITE_REG_BIT(sc, RPO_LRO_TB_DIV_REG, RPO_LRO_TB_DIV, 0x61a);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_INACTIVE_IVAL_REG, RPO_LRO_INACTIVE_IVAL, 0);
	/*
	 * the LRO timebase divider is 5 uS (0x61a),
	 * to get a maximum coalescing interval of 250 uS,
	 * we need to multiply by 50(0x32) to get
	 * the default value 250 uS
	 */
	AQ_WRITE_REG_BIT(sc, RPO_LRO_MAX_COALESCING_IVAL_REG, RPO_LRO_MAX_COALESCING_IVAL, 50);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_CONF_REG, RPO_LRO_CONF_QSESSION_LIMIT, 1);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_CONF_REG, RPO_LRO_CONF_TOTAL_DESC_LIMIT, 2);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_CONF_REG, RPO_LRO_CONF_PATCHOPTIMIZATION_EN, 0);
	AQ_WRITE_REG_BIT(sc, RPO_LRO_CONF_REG, RPO_LRO_CONF_MIN_PAYLOAD_OF_FIRST_PKT, 10);
	AQ_WRITE_REG(sc, RPO_LRO_RSC_MAX_REG, 1);
	AQ_WRITE_REG(sc, RPO_LRO_ENABLE_REG, lro);

	return 0;
}

static int
aq_set_filter(struct aq_softc *sc)
{
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	struct ethercom *ec = &sc->sc_ethercom;
	struct ether_multi *enm;
	struct ether_multistep step;
	int idx, error = 0;

	if (ifp->if_flags & IFF_PROMISC) {
		AQ_WRITE_REG_BIT(sc, RPF_L2BC_REG, RPF_L2BC_PROMISC,
		    (ifp->if_flags & IFF_PROMISC) ? 1 : 0);
		ec->ec_flags |= ETHER_F_ALLMULTI;
		goto done;
	}

	/* clear all table */
	for (idx = AQ_HW_MAC_MIN; idx <= AQ_HW_MAC_MAX; idx++)
		aq_set_mac_addr(sc, idx, NULL);

	/* don't accept all multicast */
	AQ_WRITE_REG_BIT(sc, RPF_MCAST_FILTER_MASK_REG,
	    RPF_MCAST_FILTER_MASK_ALLMULTI, 0);
	AQ_WRITE_REG_BIT(sc, RPF_MCAST_FILTER_REG(0),
	    RPF_MCAST_FILTER_EN, 0);

	idx = AQ_HW_MAC_MIN;
	ETHER_LOCK(ec);
	ETHER_FIRST_MULTI(step, ec, enm);
	while (enm != NULL) {
		if ((idx > AQ_HW_MAC_MAX) ||
		    memcmp(enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
			/*
			 * too many filters.
			 * fallback to accept all multicast addresses.
			 */
			AQ_WRITE_REG_BIT(sc, RPF_MCAST_FILTER_MASK_REG,
			    RPF_MCAST_FILTER_MASK_ALLMULTI, 1);
			AQ_WRITE_REG_BIT(sc, RPF_MCAST_FILTER_REG(0),
			    RPF_MCAST_FILTER_EN, 1);
			ec->ec_flags |= ETHER_F_ALLMULTI;
			ETHER_UNLOCK(ec);
			goto done;
		}

		/* add a filter */
		aq_set_mac_addr(sc, idx++, enm->enm_addrlo);

		ETHER_NEXT_MULTI(step, enm);
	}
	ec->ec_flags &= ~ETHER_F_ALLMULTI;
	ETHER_UNLOCK(ec);

 done:
	return error;
}

static void
aq_mediastatus_update(struct aq_softc *sc)
{
	sc->sc_media_active = 0;

	if (sc->sc_link_fc & AQ_FC_RX)
		sc->sc_media_active |= IFM_ETH_RXPAUSE;
	if (sc->sc_link_fc & AQ_FC_TX)
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
	aq_link_speed_t rate = AQ_LINK_NONE;
	aq_link_fc_t fc = AQ_FC_NONE;
	aq_link_eee_t eee = AQ_EEE_DISABLE;

	if (IFM_TYPE(sc->sc_media.ifm_media) != IFM_ETHER)
		return EINVAL;

	switch (IFM_SUBTYPE(sc->sc_media.ifm_media)) {
	case IFM_AUTO:
		rate = AQ_LINK_AUTO;
		break;
	case IFM_NONE:
		rate = AQ_LINK_NONE;
		break;
	case IFM_100_TX:
		rate = AQ_LINK_100M;
		break;
	case IFM_1000_T:
		rate = AQ_LINK_1G;
		break;
	case IFM_2500_T:
		rate = AQ_LINK_2G5;
		break;
	case IFM_5000_T:
		rate = AQ_LINK_5G;
		break;
	case IFM_10G_T:
		rate = AQ_LINK_10G;
		break;
	default:
		device_printf(sc->sc_dev, "unknown media: 0x%X\n", IFM_SUBTYPE(sc->sc_media.ifm_media));
		return ENODEV;
	}

	if (sc->sc_media.ifm_media & IFM_FLOW)
		fc = AQ_FC_ALL;

	/* XXX: todo EEE */

	/* re-initialize hardware with new parameters */
	aq_set_linkmode(sc, rate, fc, eee);

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
	/* default: auto with flowcontrol */
	ifmedia_set(&sc->sc_media, IFM_ETHER | IFM_AUTO | IFM_FLOW);
	aq_set_linkmode(sc, AQ_LINK_AUTO, AQ_FC_ALL, AQ_EEE_DISABLE);
#else
	/* default: auto without flowcontrol */
	ifmedia_set(&sc->sc_media, IFM_ETHER | IFM_AUTO);
	aq_set_linkmode(sc, AQ_LINK_AUTO, AQ_FC_NONE, AQ_EEE_DISABLE);
#endif
}

static int
aq_set_linkmode(struct aq_softc *sc, aq_link_speed_t speed, aq_link_fc_t fc, aq_link_eee_t eee)
{
	return sc->sc_fw_ops->set_mode(sc, MPI_INIT, speed, fc, eee);
}

static int
aq_get_linkmode(struct aq_softc *sc, aq_link_speed_t *speed, aq_link_fc_t *fc, aq_link_eee_t *eee)
{
	aq_hw_fw_mpi_state_e_t mode;
	int error;

	error = sc->sc_fw_ops->get_mode(sc, &mode, speed, fc, eee);
	if (error != 0)
		return error;
	if (mode != MPI_INIT)
		return ENXIO;

	return 0;
}

static void
aq_hw_init_tx_path(struct aq_softc *sc)
{
	/* Tx TC/RSS number config */
	AQ_WRITE_REG_BIT(sc, TPB_TX_BUF_REG, TPB_TX_BUF_TC_MODE_EN, 1);

	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG1_REG, THM_LSO_TCP_FLAG1_FIRST, 0x0ff6);
	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG1_REG, THM_LSO_TCP_FLAG1_MID,   0x0ff6);
	AQ_WRITE_REG_BIT(sc, THM_LSO_TCP_FLAG2_REG, THM_LSO_TCP_FLAG2_LAST,  0x0f7f);

	/* misc */
	AQ_WRITE_REG(sc, TX_TPO2_REG, (sc->sc_features & FEATURES_TPO2) ? TX_TPO2_EN : 0);
	AQ_WRITE_REG_BIT(sc, TDM_DCA_REG, TDM_DCA_EN, 0);
	AQ_WRITE_REG_BIT(sc, TDM_DCA_REG, TDM_DCA_MODE, 0);

	AQ_WRITE_REG_BIT(sc, TPB_TX_BUF_REG, TPB_TX_BUF_SCP_INS_EN, 1);
}

static void
aq_hw_init_rx_path(struct aq_softc *sc)
{
	int i;

	/* clear setting */
	AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_REG, RPB_RPF_RX_TC_MODE, 0);
	AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_REG, RPB_RPF_RX_FC_MODE, 0);
	AQ_WRITE_REG(sc, RX_FLR_RSS_CONTROL1_REG, 0);
	for (i = 0; i < 32; i++) {
		AQ_WRITE_REG_BIT(sc, RPF_ETHERTYPE_FILTER_REG(i), RPF_ETHERTYPE_FILTER_EN, 0);
	}

	if (sc->sc_rss_enable) {
		/* Rx TC/RSS number config */
		AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_REG, RPB_RPF_RX_TC_MODE, 1);

		/* Rx flow control */
		AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_REG, RPB_RPF_RX_FC_MODE, 1);

		/* RSS Ring selection */
		AQ_WRITE_REG(sc, RX_FLR_RSS_CONTROL1_REG, RX_FLR_RSS_CONTROL1_EN | 0x33333333);
	}

	/* L2 and Multicast filters */
	for (i = AQ_HW_MAC_MIN; i < AQ_HW_MAC_MAX; i++) {
		AQ_WRITE_REG_BIT(sc, RPF_L2UC_MSW_REG(i), RPF_L2UC_MSW_EN, 0);
		AQ_WRITE_REG_BIT(sc, RPF_L2UC_MSW_REG(i), RPF_L2UC_MSW_ACTION, 1);
	}
	AQ_WRITE_REG(sc, RPF_MCAST_FILTER_MASK_REG, 0);
	AQ_WRITE_REG(sc, RPF_MCAST_FILTER_REG(0), 0x00010fff);

	/* Vlan filters */
	AQ_WRITE_REG_BIT(sc, RPF_VLAN_TPID_REG, RPF_VLAN_TPID_OUTER, ETHERTYPE_QINQ);
	AQ_WRITE_REG_BIT(sc, RPF_VLAN_TPID_REG, RPF_VLAN_TPID_OUTER, ETHERTYPE_VLAN);
	AQ_WRITE_REG_BIT(sc, RPF_VLAN_MODE_REG, RPF_VLAN_MODE_PROMISC, 1);
	AQ_WRITE_REG_BIT(sc, RPF_VLAN_MODE_REG, RPF_VLAN_MODE_ACCEPT_UNTAGGED, 1);
	AQ_WRITE_REG_BIT(sc, RPF_VLAN_MODE_REG, RPF_VLAN_MODE_UNTAGGED_ACTION, RPF_ACTION_HOST);

	/* misc */
	if (sc->sc_features & FEATURES_RPF2)
		AQ_WRITE_REG(sc, RX_TCP_RSS_HASH_REG, 0x000f001e);	/* XXX: linux:0x000f0000, freebsd:0x00f0001e */
	else
		AQ_WRITE_REG(sc, RX_TCP_RSS_HASH_REG, 0);

	AQ_WRITE_REG_BIT(sc, RPF_L2BC_REG, RPF_L2BC_EN, 1);
	AQ_WRITE_REG_BIT(sc, RPF_L2BC_REG, RPF_L2BC_ACTION, RPF_ACTION_HOST);
	AQ_WRITE_REG_BIT(sc, RPF_L2BC_REG, RPF_L2BC_THRESHOLD, 0xffff);

	AQ_WRITE_REG_BIT(sc, RX_DMA_DCA_REG, RX_DMA_DCA_EN, 0);
	AQ_WRITE_REG_BIT(sc, RX_DMA_DCA_REG, RX_DMA_DCA_MODE, 0);
}

static void
aq_hw_interrupt_moderation_set(struct aq_softc *sc)
{
	int i;

	if (sc->sc_intr_moderation_enable) {
		AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_REG, TX_DMA_INT_DESC_WRWB_EN, 0);
		AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_REG, TX_DMA_INT_DESC_MODERATE_EN, 1);
		AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_REG, RX_DMA_INT_DESC_WRWB_EN, 0);
		AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_REG, RX_DMA_INT_DESC_MODERATE_EN, 1);

		//XXX: should be configured according to link speed...
		for (i = 0; i < sc->sc_txringnum; i++) {
			AQ_WRITE_REG(sc, TX_INTR_MODERATION_CTL_REG(i),
			    __SHIFTIN(0x0f, TX_INTR_MODERATION_CTL_MIN) |
			    __SHIFTIN(0x1ff, TX_INTR_MODERATION_CTL_MAX) |
			    TX_INTR_MODERATION_CTL_EN);
		}
		for (i = 0; i < sc->sc_rxringnum; i++) {
			AQ_WRITE_REG(sc, RX_INTR_MODERATION_CTL_REG(i),
			    __SHIFTIN(0x30, RX_INTR_MODERATION_CTL_MIN) |
			    __SHIFTIN(0x80, RX_INTR_MODERATION_CTL_MAX) |
			    RX_INTR_MODERATION_CTL_EN);
		}

	} else {
		AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_REG, TX_DMA_INT_DESC_WRWB_EN, 1);
		AQ_WRITE_REG_BIT(sc, TX_DMA_INT_DESC_WRWB_EN_REG, TX_DMA_INT_DESC_MODERATE_EN, 0);
		AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_REG, RX_DMA_INT_DESC_WRWB_EN, 1);
		AQ_WRITE_REG_BIT(sc, RX_DMA_INT_DESC_WRWB_EN_REG, RX_DMA_INT_DESC_MODERATE_EN, 0);

		for (i = 0; i < sc->sc_txringnum; i++) {
			AQ_WRITE_REG(sc, TX_INTR_MODERATION_CTL_REG(i), 0);
		}
		for (i = 0; i < sc->sc_rxringnum; i++) {
			AQ_WRITE_REG(sc, RX_INTR_MODERATION_CTL_REG(i), 0);
		}
	}
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
	AQ_WRITE_REG_BIT(sc, TPS_DESC_VM_ARB_MODE_REG, TPS_DESC_VM_ARB_MODE, 0);

	/* TPS TC credits init */
	AQ_WRITE_REG_BIT(sc, TPS_DESC_TC_ARB_MODE_REG, TPS_DESC_TC_ARB_MODE, 0);
	AQ_WRITE_REG_BIT(sc, TPS_DATA_TC_ARB_MODE_REG, TPS_DATA_TC_ARB_MODE, 0);

	AQ_WRITE_REG_BIT(sc, TPS_DATA_TCTCREDIT_MAX_REG(tc), TPS_DATA_TCTCREDIT_MAX, 0xfff);
	AQ_WRITE_REG_BIT(sc, TPS_DATA_TCTWEIGHT_REG(tc), TPS_DATA_TCTWEIGHT, 0x64);
	AQ_WRITE_REG_BIT(sc, TPS_DESC_TCTCREDIT_MAX_REG(tc), TPS_DESC_TCTCREDIT_MAX, 0x50);
	AQ_WRITE_REG_BIT(sc, TPS_DESC_TCTWEIGHT_REG(tc), TPS_DESC_TCTWEIGHT, 0x1e);

	/* Tx buf size */
	tc = 0;
	buff_size = AQ_HW_TXBUF_MAX;
	AQ_WRITE_REG_BIT(sc, TPB_TXBBUF_SIZE_REG(tc), TPB_TXBBUF_SIZE, buff_size);
	AQ_WRITE_REG_BIT(sc, TPB_TXB_THRESH_REG(tc), TPB_TXB_THRESH_HI, (buff_size * (1024 / 32) * 66) / 100);
	AQ_WRITE_REG_BIT(sc, TPB_TXB_THRESH_REG(tc), TPB_TXB_THRESH_LO, (buff_size * (1024 / 32) * 50) / 100);

	/* QoS Rx buf size per TC */
	tc = 0;
	buff_size = AQ_HW_RXBUF_MAX;
	AQ_WRITE_REG_BIT(sc, RPB_RXBBUF_SIZE_REG(tc), RPB_RXBBUF_SIZE, buff_size);
	AQ_WRITE_REG_BIT(sc, RPB_RXB_XOFF_REG(tc), RPB_RXB_XOFF_EN, 0);
	AQ_WRITE_REG_BIT(sc, RPB_RXB_XOFF_REG(tc), RPB_RXB_XOFF_THRESH_HI, (buff_size * (1024 / 32) * 66) / 100);
	AQ_WRITE_REG_BIT(sc, RPB_RXB_XOFF_REG(tc), RPB_RXB_XOFF_THRESH_LO, (buff_size * (1024 / 32) * 50) / 100);

	/* QoS 802.1p priority -> TC mapping */
	int i_priority;
	for (i_priority = 0; i_priority < 8; i_priority++) {
		AQ_WRITE_REG_BIT(sc, RPF_RPB_RX_TC_UPT_REG, RPF_RPB_RX_TC_UPT_MASK(0), i_priority);
	}
}

/* called once from aq_attach */
static void
aq_init_rsstable(struct aq_softc *sc)
{
	unsigned int i;

	/* initialize RSS key and redirect table */

	cprng_fast(&sc->sc_rss_key, sizeof(sc->sc_rss_key));
#if XXX_DEBUG_RSSKEY_ZERO
	memset(&sc->sc_rss_key, 0, sizeof(sc->sc_rss_key));
#endif


	for (i = 0; i < AQ_RSS_INDIRECTION_TABLE_MAX; i++) {
		sc->sc_rss_table[i] = i % sc->sc_rxringnum;
	}

#ifdef XXX_DUMP_RSS_KEY
	printf("rss_key:");
	for (i = 0; i < __arraycount(sc->sc_rss_key); i++) {
		printf(" %08x", sc->sc_rss_key[i]);
	}
	printf("\n");

	printf("rss_table:");
	for (i = 0; i < AQ_RSS_INDIRECTION_TABLE_MAX; i++) {
		printf(" %d", sc->sc_rss_table[i]);
	}
	printf("\n");
#endif

}

static int
aq_hw_rss_hash_set(struct aq_softc *sc)
{
	unsigned int i;
	int error = 0;

	for (i = 0; i < __arraycount(sc->sc_rss_key); i++) {
		uint32_t key_data = sc->sc_rss_enable ? sc->sc_rss_key[i] : 0;

		AQ_WRITE_REG(sc, RPF_RSS_KEY_WR_DATA_REG, key_data);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_KEY_ADDR_REG, RPF_RSS_KEY_ADDR, i);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_KEY_ADDR_REG, RPF_RSS_KEY_WR_EN, 1);
		WAIT_FOR(AQ_READ_REG_BIT(sc, RPF_RSS_KEY_ADDR_REG, RPF_RSS_KEY_WR_EN) == 0,
		    1000, 10, &error);
		if (error != 0) {
			printf("%s:%d: XXX: timeout\n", __func__, __LINE__);
			break;
		}
	}

	return error;
}

static int
aq_hw_rss_set(struct aq_softc *sc)
{
	uint16_t bitary[1 + (AQ_RSS_INDIRECTION_TABLE_MAX * 3 / 16)];
	int error = 0;
	unsigned int i;

	memset(bitary, 0, sizeof(bitary));
	for (i = AQ_RSS_INDIRECTION_TABLE_MAX; i--;) {
		(*(uint32_t *)(bitary + ((i * 3) / 16))) |=
		    ((sc->sc_rss_table[i]) << ((i * 3) & 15));
	}

	for (i = __arraycount(bitary); i--;) {
		AQ_WRITE_REG_BIT(sc, RPF_RSS_REDIR_WR_DATA_REG, RPF_RSS_REDIR_WR_DATA, bitary[i]);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_REDIR_ADDR_REG, RPF_RSS_REDIR_ADDR, i);
		AQ_WRITE_REG_BIT(sc, RPF_RSS_REDIR_ADDR_REG, RPF_RSS_REDIR_WR_EN, 1);
		WAIT_FOR(AQ_READ_REG_BIT(sc, RPF_RSS_REDIR_ADDR_REG, RPF_RSS_REDIR_WR_EN) == 0,
		    1000, 10, &error);
		if (error != 0)
			break;
	}
	return error;
}

static void
aq_hw_l3_filter_set(struct aq_softc *sc, bool enable)
{
	int i;

	/* clear all filter */
	for (i = 0; i < 8; i++) {
		AQ_WRITE_REG_BIT(sc, RPF_L3_FILTER_REG(i), RPF_L3_FILTER_L4_EN, 0);
	}

	if (!enable) {
		/*
		 * HW bug workaround:
		 * Disable RSS for UDP using rx flow filter 0.
		 * HW does not track RSS stream for fragmenged UDP,
		 * 0x5040 control reg does not work.
		 */
		AQ_WRITE_REG_BIT(sc, RPF_L3_FILTER_REG(0), RPF_L3_FILTER_L4_EN, 1);
		AQ_WRITE_REG_BIT(sc, RPF_L3_FILTER_REG(0), RPF_L3_FILTER_L4_PROTO_EN, 1);
		AQ_WRITE_REG_BIT(sc, RPF_L3_FILTER_REG(0), RPF_L3_FILTER_L4_PROTO, RPF_L3_FILTER_L4_PROTF_UDP);
		AQ_WRITE_REG_BIT(sc, RPF_L3_FILTER_REG(0), RPF_L3_FILTER_L4_RXQUEUE_EN, 1);
		AQ_WRITE_REG_BIT(sc, RPF_L3_FILTER_REG(0), RPF_L3_FILTER_L4_RXQUEUE, 0);	/* -> rxring[0] */
		AQ_WRITE_REG_BIT(sc, RPF_L3_FILTER_REG(0), RPF_L3_FILTER_L4_ACTION, RPF_ACTION_HOST);
	}
}

struct aq_rx_filter_vlan {
	uint8_t enable;
	uint8_t location;
	uint16_t vlan_id;
	uint8_t queue;
};

static void
aq_update_vlan_filters(struct aq_softc *sc)
{
#define VLAN_MAX_FILTERS		16
//	struct aq_rx_filter_vlan aq_vlans[AQ_HW_VLAN_MAX_FILTERS];
//	int bit_pos = 0;
//	int vlan_tag = -1;
	int i;

	AQ_WRITE_REG_BIT(sc, RPF_VLAN_MODE_REG, RPF_VLAN_MODE_PROMISC, 1);
#if 0
XXX: notyet
	for (i = 0; i < AQ_HW_VLAN_MAX_FILTERS; i++) {
		bit_ffs_at(softc->vlan_tags, bit_pos, 4096, &vlan_tag);
		if (vlan_tag != -1) {
			aq_vlans[i].enable = true;
			aq_vlans[i].location = i;
			aq_vlans[i].queue = 0xFF;
			aq_vlans[i].vlan_id = vlan_tag;
			bit_pos = vlan_tag;
		} else {
			aq_vlans[i].enable = false;
		}
	}
#endif

	for (i = 0; i < VLAN_MAX_FILTERS; i++) {
		AQ_WRITE_REG_BIT(sc, RPF_VLAN_FILTER_REG(i), RPF_VLAN_FILTER_EN, 0);
		AQ_WRITE_REG_BIT(sc, RPF_VLAN_FILTER_REG(i), RPF_VLAN_FILTER_RXQ_EN, 0);

#if 0
XXX: notyet
		if (aq_vlans[i].enable) {
			hw_atl_rpf_vlan_id_flr_set(self,
						   aq_vlans[i].vlan_id,
						   i);
			hw_atl_rpf_vlan_flr_act_set(self, 1U, i);
			hw_atl_rpf_vlan_flr_en_set(self, 1U, i);
			if (aq_vlans[i].queue != 0xFF) {
				hw_atl_rpf_vlan_rxq_flr_set(self,
							    aq_vlans[i].queue,
							    i);
				hw_atl_rpf_vlan_rxq_en_flr_set(self, 1U, i);
			}
		}
#endif
	}

//	hw_atl_b0_hw_vlan_promisc_set(hw, aq_is_vlan_promisc_required(softc));

}

static int
aq_hw_init(struct aq_softc *sc)
{
	uint32_t v;

	/* Force limit MRRS on RDM/TDM to 2K */
	v = AQ_READ_REG(sc, AQ_PCI_REG_CONTROL_6_REG);
	AQ_WRITE_REG(sc, AQ_PCI_REG_CONTROL_6_REG,
	    (v & ~0x0707) | 0x0404);

	/*
	 * TX DMA total request limit. B0 hardware is not capable to
	 * handle more than (8K-MRRS) incoming DMA data.
	 * Value 24 in 256byte units
	 */
	AQ_WRITE_REG(sc, AQ_HW_TX_DMA_TOTAL_REQ_LIMIT_REG, 24);

	aq_hw_init_tx_path(sc);
	aq_hw_init_rx_path(sc);

	aq_hw_interrupt_moderation_set(sc);

	aq_set_mac_addr(sc, AQ_HW_MAC, sc->sc_enaddr.ether_addr_octet);
	aq_set_linkmode(sc, AQ_LINK_NONE, AQ_FC_NONE, AQ_EEE_DISABLE);

	aq_hw_qos_set(sc);

	/* Enable interrupt */
//	int irqmode =  AQ_INTR_CTRL_IRQMODE_LEGACY;	// spurious interrupt occurs??? (intr-status=0)
	int irqmode =  AQ_INTR_CTRL_IRQMODE_MSI;
//	int irqmode =  AQ_INTR_CTRL_IRQMODE_MSIX;

#if 0
	AQ_WRITE_REG(sc, AQ_INTR_CTRL_REG, AQ_INTR_CTRL_RESET_DIS);
//	AQ_WRITE_REG(sc, AQ_INTR_CTRL_REG, AQ_INTR_CTRL_RESET_DIS | AQ_INTR_CTRL_CLR_ON_READ);
	AQ_WRITE_REG_BIT(sc, AQ_INTR_CTRL_REG, AQ_INTR_CTRL_IRQMODE, irqmode);
#else
	AQ_WRITE_REG(sc, AQ_INTR_CTRL_REG, AQ_INTR_CTRL_RESET_DIS | __SHIFTIN(irqmode, AQ_INTR_CTRL_IRQMODE));
#endif

	AQ_WRITE_REG(sc, AQ_INTR_AUTOMASK_REG, 0xffffffff);

	AQ_WRITE_REG(sc, AQ_GEN_INTR_MAP_REG(0),
	    ((AQ_B0_ERR_INT << 24) | (1 << 31)) |
	    ((AQ_B0_ERR_INT << 16) | (1 << 23))
	);

	/* link interrupt */
	AQ_WRITE_REG(sc, AQ_GEN_INTR_MAP_REG(3), __BIT(7) | LINKSTAT_IRQ);

	return 0;
}

static int
aq_update_link_status(struct aq_softc *sc)
{
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	aq_link_speed_t rate = AQ_LINK_NONE;
	aq_link_fc_t fc = AQ_FC_NONE;
	aq_link_eee_t eee = AQ_EEE_DISABLE;
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
			device_printf(sc->sc_dev, "link is UP: speed=%u\n", speed);
			if_link_state_change(ifp, LINK_STATE_UP);
		} else if (rate == AQ_LINK_NONE) {
			/* link UP -> DOWN */
			device_printf(sc->sc_dev, "link is DOWN\n");
			if_link_state_change(ifp, LINK_STATE_DOWN);
		} else {
			device_printf(sc->sc_dev, "link mode changed: speed=%u, fc=0x%x, eee=%x\n", speed, fc, eee);
		}

		sc->sc_link_rate = rate;
		sc->sc_link_fc = fc;
		sc->sc_link_eee = eee;

		aq_mediastatus_update(sc);

		/* update interrupt timing according to new link speed */
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
				device_printf(sc->sc_dev, "STAT: %s: %s: %lu -> %lu (+%lu)\n", descr, #name, sc->sc_statistics_ ## name, sc->sc_statistics_ ## name + n, n);	\
			}														\
			sc->sc_statistics_ ## name += n;										\
		} while (/*CONSTCOND*/0);

#ifdef XXX_DUMP_STAT
		ADD_DELTA(cur, prev, uprc, "RX ucast");
		ADD_DELTA(cur, prev, mprc, "RX mcast");
		ADD_DELTA(cur, prev, bprc, "RX bcast");
		ADD_DELTA(cur, prev, prc,  "RX good");
		ADD_DELTA(cur, prev, erpr, "RX error");
		ADD_DELTA(cur, prev, uptc, "TX ucast");
		ADD_DELTA(cur, prev, mptc, "TX mcast");
		ADD_DELTA(cur, prev, bptc, "TX bcast");
		ADD_DELTA(cur, prev, ptc,  "TX good");
		ADD_DELTA(cur, prev, erpt, "TX error");
		ADD_DELTA(cur, prev, mbtc, "TX mcast bytes");
		ADD_DELTA(cur, prev, bbtc, "TX bcast bytes");
		ADD_DELTA(cur, prev, mbrc, "RX mcast bytes");
		ADD_DELTA(cur, prev, bbrc, "RX bcast bytes");
		ADD_DELTA(cur, prev, ubrc, "RX ucast bytes");
		ADD_DELTA(cur, prev, ubtc, "TX ucast bytes");
		ADD_DELTA(cur, prev, dpc,  "DMA drop");
		ADD_DELTA(cur, prev, cprc, "RX coalesced");
#endif

		sc->sc_statistics_idx = cur;
	}

	return changed;
}

/* allocate and map one DMA block */
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
		device_printf(sc->sc_dev,
		    "unable to load rx DMA map %d, error = %d\n", idx, error);
		panic("%s: unable to load rx DMA map. error=%d", __func__, error);
	}
	bus_dmamap_sync(sc->sc_dmat, rxring->ring_mbufs[idx].dmamap, 0,
	    rxring->ring_mbufs[idx].dmamap->dm_mapsize, BUS_DMASYNC_PREREAD);

	aq_rxring_reset_desc(sc, rxring, idx);

	return 0;
}

static int
aq_txrx_rings_alloc(struct aq_softc *sc)
{
	int n, error;

	for (n = 0; n < sc->sc_txringnum; n++) {
		sc->sc_txring[n].ring_sc = sc;
		sc->sc_txring[n].ring_index = n;
		mutex_init(&sc->sc_txring[n].ring_mutex, MUTEX_DEFAULT, IPL_NET);
		error = aq_txring_alloc(sc, &sc->sc_txring[n]);
		if (error != 0)
			goto failure;
	}

	for (n = 0; n < sc->sc_rxringnum; n++) {
		sc->sc_rxring[n].ring_sc = sc;
		sc->sc_rxring[n].ring_index = n;
		mutex_init(&sc->sc_rxring[n].ring_mutex, MUTEX_DEFAULT, IPL_NET);
		error = aq_rxring_alloc(sc, &sc->sc_rxring[n]);
		if (error != 0)
			break;
	}

 failure:
	return error;
}

#ifdef XXX_DUMP_RING
static void
dump_txrings(struct aq_softc *sc)
{
	struct aq_txring *txring;
	int n, i;

	for (n = 0; n < sc->sc_txringnum; n++) {
		txring = &sc->sc_txring[n];
		mutex_enter(&txring->ring_mutex);


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
			    (txring->ring_txdesc[i].ctl & AQ_TXDESC_CTL1_EOP) ? " EOP" : "",
			    (txring->ring_txdesc[i].ctl & AQ_TXDESC_CTL1_CMD_WB) ? " WB" : "");
			printf("txring->ring_txdesc[%d].ctl2 = %08x\n", i, txring->ring_txdesc[i].ctl2);
		}

		mutex_exit(&txring->ring_mutex);
	}
}

static void
dump_rxrings(struct aq_softc *sc)
{
	struct aq_rxring *rxring;
	int n, i;

	for (n = 0; n < sc->sc_rxringnum; n++) {
		rxring = &sc->sc_rxring[n];
		mutex_enter(&rxring->ring_mutex);

		printf("# rxring=%p (index=%d)\n", rxring, rxring->ring_index);
		printf("rxring->ring_rxdesc        = %p\n", rxring->ring_rxdesc);
		printf("rxring->ring_rxdesc_dmamap = %p\n", rxring->ring_rxdesc_dmamap);
		printf("rxring->ring_rxdesc_size   = %lu\n", rxring->ring_rxdesc_size);
		for (i = 0; i < AQ_RXD_NUM; i++) {
			printf("rxring->ring_mbufs[%d].m      = %p\n", i, rxring->ring_mbufs[i].m);
			printf("rxring->ring_mbufs[%d].dmamap = %p\n", i, rxring->ring_mbufs[i].dmamap);
		}

		mutex_exit(&rxring->ring_mutex);
	}
}
#endif /* XXX_DUMP_RING */

static void
aq_txrx_rings_free(struct aq_softc *sc)
{
	int n;

	for (n = 0; n < sc->sc_txringnum; n++) {
		aq_txring_free(sc, &sc->sc_txring[n]);
		mutex_destroy(&sc->sc_txring[n].ring_mutex);
	}

	for (n = 0; n < sc->sc_rxringnum; n++) {
		aq_rxring_free(sc, &sc->sc_rxring[n]);
		mutex_destroy(&sc->sc_rxring[n].ring_mutex);
	}
}

#ifdef USE_CALLOUT_TICK
static void
aq_tick(void *arg)
{
	struct aq_softc *sc = arg;

	aq_update_link_status(sc);

	callout_reset(&sc->sc_tick_ch, hz, aq_tick, sc);
}
#endif

/* interrupt enable/disable */
static void
aq_enable_intr(struct aq_softc *sc, bool link, bool txrx)
{
	uint32_t imask = 0;

	if (txrx)
		imask |= __BITS(0, sc->sc_ringnum - 1);
	if (link)
		imask |= __BIT(LINKSTAT_IRQ);

	AQ_WRITE_REG(sc, AQ_INTR_MASK_REG, imask);
	AQ_WRITE_REG(sc, AQ_INTR_STATUS_CLR_REG, 0xffffffff);	//XXX

	printf("%s:%d: INTR_MASK/INTR_STATUS=%08x/%08x\n", __func__, __LINE__, AQ_READ_REG(sc, AQ_INTR_MASK_REG), AQ_READ_REG(sc, AQ_INTR_STATUS_REG));
}

static int
aq_intr(void *arg)
{
	struct aq_softc *sc __unused = arg;
	uint32_t status;
	int nintr = 0;
	int i, n;
#ifdef XXX_INTR_DEBUG
	int rxcount[32];
	int txcount[32];
#endif


	status = AQ_READ_REG(sc, AQ_INTR_STATUS_REG);
#ifdef XXX_INTR_DEBUG
	memset(rxcount, 0, sizeof(rxcount));
	memset(txcount, 0, sizeof(txcount));
	printf("#### INTERRUPT #### %s@cpu%d: INTR_MASK/INTR_STATUS = %08x/%08x=>%08x\n", __func__, cpu_index(curcpu()), AQ_READ_REG(sc, AQ_INTR_MASK_REG), status, AQ_READ_REG(sc, AQ_INTR_STATUS_REG));
#endif

	if (status & __BIT(LINKSTAT_IRQ)) {
		nintr += aq_update_link_status(sc);
	}

	for (i = 0; i < sc->sc_rxringnum; i++) {
		if (status & __BIT(i)) {
			mutex_enter(&sc->sc_rxring[i].ring_mutex);
			n = aq_rx_intr(&sc->sc_rxring[i]);
			mutex_exit(&sc->sc_rxring[i].ring_mutex);
			if (n != 0)
				nintr++;
#ifdef XXX_INTR_DEBUG
			rxcount[i] = n;
#endif
		}
	}
	for (i = 0; i < sc->sc_txringnum; i++) {
		if (status & __BIT(i)) {
			mutex_enter(&sc->sc_txring[i].ring_mutex);
			n = aq_tx_intr(&sc->sc_txring[i]);
			mutex_exit(&sc->sc_txring[i].ring_mutex);
			if (n != 0)
				nintr++;
#ifdef XXX_INTR_DEBUG
			txcount[i] = n;
#endif
		}
	}

#ifdef XXX_INTR_DEBUG
	printf("RX:");
	for (i = 0; i < sc->sc_rxringnum; i++) {
		printf("%d%s", rxcount[i], aq_rx_intr_poll(&sc->sc_rxring[i]) ? "!" : " ");
	}

	printf(" / TX:");
	for (i = 0; i < sc->sc_txringnum; i++) {
		printf("%d%s", txcount[i], aq_tx_intr_poll(&sc->sc_txring[i]) ? "!" : " ");
	}
	printf("\n");
#endif

	AQ_WRITE_REG(sc, AQ_INTR_STATUS_CLR_REG, 0xffffffff);	//XXX
	return nintr;
}

static void
aq_txring_reset(struct aq_softc *sc, struct aq_txring *txring, bool start)
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

	/* disable DMA */
	AQ_WRITE_REG_BIT(sc, TX_DMA_DESC_REG(ringidx), TX_DMA_DESC_EN, 0);

	if (start) {
		/* TX descriptor physical address */
		paddr_t paddr = txring->ring_txdesc_dmamap->dm_segs[0].ds_addr;
		AQ_WRITE_REG(sc, TX_DMA_DESC_BASE_ADDRLSW_REG(ringidx), paddr);
		AQ_WRITE_REG(sc, TX_DMA_DESC_BASE_ADDRMSW_REG(ringidx), paddr >> 32);

		/* TX descriptor size */
		AQ_WRITE_REG_BIT(sc, TX_DMA_DESC_REG(ringidx), TX_DMA_DESC_LEN,
		    AQ_TXD_NUM / 8);

		/* reset TAIL pointer */
		AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_REG(ringidx), 0);
		AQ_WRITE_REG(sc, TX_DMA_DESC_WRWB_THRESH_REG(ringidx), 0);

		/* irq map */
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_TX_REG(ringidx), AQ_INTR_IRQ_MAP_TX_IRQMAP(ringidx), ringidx /*=msix*/);
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_TX_REG(ringidx), AQ_INTR_IRQ_MAP_TX_EN(ringidx), true);

		/* enable DMA */
		AQ_WRITE_REG_BIT(sc, TX_DMA_DESC_REG(ringidx), TX_DMA_DESC_EN, 1);

		const int cpuid = 0;	//XXX
		AQ_WRITE_REG_BIT(sc, TDM_DCAD_REG(ringidx), TDM_DCAD_CPUID, cpuid);
		AQ_WRITE_REG_BIT(sc, TDM_DCAD_REG(ringidx), TDM_DCAD_CPUID_EN, 0);
	}
}

static void
aq_txring_start(struct aq_softc *sc, struct aq_txring *txring)
{
//	printf("%s:%d: txring[%d] proidx = %d\n", __func__, __LINE__, txring->ring_index, txring->ring_prodidx);

	AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_REG(txring->ring_index),
	    txring->ring_prodidx);
}

static int
aq_rxring_reset(struct aq_softc *sc, struct aq_rxring *rxring, bool start)
{
	const int ringidx = rxring->ring_index;
	int i;
	int error = 0;

	/* disable DMA */
	AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_REG(ringidx), RX_DMA_DESC_EN, 0);

	/* free all RX mbufs */
	aq_rxdrain(sc, rxring);

	if (start) {
		for (i = 0; i < AQ_RXD_NUM; i++) {
			error = aq_rxring_add(sc, rxring, i);
			if (error != 0) {
				aq_rxdrain(sc, rxring);
				return error;
			}
		}

		/* RX descriptor physical address */
		paddr_t paddr = rxring->ring_rxdesc_dmamap->dm_segs[0].ds_addr;
		AQ_WRITE_REG(sc, RX_DMA_DESC_BASE_ADDRLSW_REG(ringidx), paddr);
		AQ_WRITE_REG(sc, RX_DMA_DESC_BASE_ADDRMSW_REG(ringidx), paddr >> 32);

		/* RX descriptor size */
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_REG(ringidx), RX_DMA_DESC_LEN,
		    AQ_RXD_NUM / 8);

		/* maximum receive frame size */
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_BUFSIZE_REG(ringidx), RX_DMA_DESC_BUFSIZE_DATA, MCLBYTES / 1024);
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_BUFSIZE_REG(ringidx), RX_DMA_DESC_BUFSIZE_HDR, 0 / 1024);

		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_REG(ringidx), RX_DMA_DESC_HEADER_SPLIT, 0);
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_REG(ringidx), RX_DMA_DESC_VLAN_STRIP,
		    (sc->sc_ethercom.ec_capenable & ETHERCAP_VLAN_HWTAGGING) ? 1 : 0);

		/* reset TAIL pointer, and update readidx (HEAD pointer cannot write) */
		AQ_WRITE_REG(sc, RX_DMA_DESC_TAIL_PTR_REG(ringidx), AQ_RXD_NUM - 1);
		rxring->ring_readidx = AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_REG(ringidx), RX_DMA_DESC_HEAD_PTR);

		/* Rx ring set mode */

		/* Mapping interrupt vector */
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_RX_REG(ringidx), AQ_INTR_IRQ_MAP_RX_IRQMAP(ringidx), ringidx /*=msix*/);
		AQ_WRITE_REG_BIT(sc, AQ_INTR_IRQ_MAP_RX_REG(ringidx), AQ_INTR_IRQ_MAP_RX_EN(ringidx), 1);

		const int cpuid = 0;	//XXX
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_REG(ringidx), RDM_DCAD_CPUID, cpuid);
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_REG(ringidx), RDM_DCAD_DESC_EN, 0);
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_REG(ringidx), RDM_DCAD_HEADER_EN, 0);
		AQ_WRITE_REG_BIT(sc, RDM_DCAD_REG(ringidx), RDM_DCAD_PAYLOAD_EN, 0);

		/* enable DMA. start receiving */
		AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_REG(ringidx), RX_DMA_DESC_EN, 1);
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
	struct mbuf * const m = *mp;
	uint32_t ctl1, ctl1_ctx, ctl2;
	int idx, i, error;

	idx = txring->ring_prodidx;
	map = txring->ring_mbufs[idx].dmamap;

	error = bus_dmamap_load_mbuf(sc->sc_dmat, map, m,
	    BUS_DMA_NOWAIT);
	if (error != 0) {
		/* XXX: TODO: try to m_defrag */
		device_printf(sc->sc_dev,
		    "Error mapping mbuf into TX chain: error=%d\n", error);
		m_freem(m);
		return error;
	}

	/*
	 * check spaces of free descriptors.
	 * +1 is reserved for context descriptor for vlan, etc,.
	 */
	if ((map->dm_nsegs + 1)  > txring->ring_nfree) {
		bus_dmamap_unload(sc->sc_dmat, map);
		device_printf(sc->sc_dev,
		    "too many mbuf chain %d\n", map->dm_nsegs);
		m_freem(m);
		return ENOBUFS;
	}

	bus_dmamap_sync(sc->sc_dmat, map, 0, map->dm_mapsize,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	ctl1_ctx = 0;
	ctl2 = __SHIFTIN(m->m_pkthdr.len, AQ_TXDESC_CTL2_LEN);

	if (vlan_has_tag(m)) {
		ctl1 = AQ_TXDESC_CTL1_TYPE_TXC;
#ifdef XXX_TXDESC_DEBUG
		printf("TXdesc[%d] set VLANID %u\n", idx, vlan_get_tag(m));
#endif
		ctl1 |= __SHIFTIN(vlan_get_tag(m), AQ_TXDESC_CTL1_VID);

		ctl1_ctx |= AQ_TXDESC_CTL1_CMD_VLAN;
		ctl2 |= AQ_TXDESC_CTL2_CTX_EN;


		/* fill context descriptor and forward index */
		txring->ring_txdesc[idx].buf_addr = 0;
		txring->ring_txdesc[idx].ctl1 = htole32(ctl1);
		txring->ring_txdesc[idx].ctl2 = 0;

		idx = TXRING_NEXTIDX(idx);
		txring->ring_nfree--;
	}

	if (m->m_pkthdr.csum_flags & M_CSUM_IPv4)
		ctl1_ctx |= AQ_TXDESC_CTL1_CMD_IP4CSUM;
	if (m->m_pkthdr.csum_flags & (M_CSUM_TCPv4 | M_CSUM_UDPv4 | M_CSUM_TCPv6 | M_CSUM_UDPv6)) {
		ctl1_ctx |= AQ_TXDESC_CTL1_CMD_L4CSUM;
	}

	/* fill descriptor(s) */
	for (i = 0; i < map->dm_nsegs; i++) {
		ctl1 = ctl1_ctx | AQ_TXDESC_CTL1_TYPE_TXD |
		    __SHIFTIN(map->dm_segs[i].ds_len, AQ_TXDESC_CTL1_BLEN);
		ctl1 |= AQ_TXDESC_CTL1_CMD_FCS;

		if (i == 0) {
			/* remember mbuf of these descriptors */
			txring->ring_mbufs[idx].m = m;
		} else {
			txring->ring_mbufs[idx].m = NULL;
		}

		if (i == map->dm_nsegs - 1) {
			/* last segment, mark as EndOfPacket, and cause to intr */
			ctl1 |= AQ_TXDESC_CTL1_EOP | AQ_TXDESC_CTL1_CMD_WB;
		}

#ifdef XXX_TXDESC_DEBUG
		printf("TXdesc[%d] seg:%d/%d buf_addr=%012lx, len=%-5lu ctl1=%08x ctl2=%08x%s\n",
		    idx,
		    i, map->dm_nsegs - 1,
		    map->dm_segs[i].ds_addr,
		    map->dm_segs[i].ds_len,
		    ctl1, ctl2,
		    (i == map->dm_nsegs - 1) ? " EOP/WB" : "");
#endif
		txring->ring_txdesc[idx].buf_addr = htole64(map->dm_segs[i].ds_addr);
		txring->ring_txdesc[idx].ctl1 = htole32(ctl1);
		txring->ring_txdesc[idx].ctl2 = htole32(ctl2);

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

	if (txring->ring_considx == AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_REG(txring->ring_index), TX_DMA_DESC_HEAD_PTR))
		return 0;
	return 1;
}
#endif

static int
aq_tx_intr(struct aq_txring *txring)
{
	struct aq_softc *sc = txring->ring_sc;
	struct ifnet *ifp = &sc->sc_ethercom.ec_if;
	unsigned int idx, hw_head, n;

	//XXX: need lock

	hw_head = AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_REG(txring->ring_index), TX_DMA_DESC_HEAD_PTR);
	if (hw_head == txring->ring_considx)
		return 0;

#if 0
	printf("%s:%d: ringidx=%d, HEAD/TAIL=%lu/%u prod/cons=%d/%d\n", __func__, __LINE__, txring->ring_index,
	    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_REG(txring->ring_index), TX_DMA_DESC_HEAD_PTR),
	    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_REG(txring->ring_index)),
	    txring->ring_prodidx,
	    txring->ring_considx);
#endif

	for (idx = txring->ring_considx, n = 0; idx != hw_head;
	    idx = TXRING_NEXTIDX(idx), n++) {

#if 0
		printf("# %s:%d: txring=%d, TX CLEANUP: HEAD/TAIL=%lu/%u, considx/prodidx=%d/%d, idx=%d\n", __func__, __LINE__,
		    txring->ring_index,
		    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_REG(txring->ring_index), TX_DMA_DESC_HEAD_PTR),
		    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_REG(txring->ring_index)),
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
		txring->ring_txdesc[idx].ctl = AQ_TXDESC_CTL1_DD;
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
		AQ_WRITE_REG(sc, TX_DMA_DESC_TAIL_PTR_REG(txring->ring_index), 0);
#endif
	}

	return n;
}

#ifdef  XXX_INTR_DEBUG
static int
aq_rx_intr_poll(struct aq_rxring *rxring)
{
	struct aq_softc *sc = rxring->ring_sc;

	if (rxring->ring_readidx == AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_REG(rxring->ring_index), RX_DMA_DESC_HEAD_PTR))
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
	unsigned int idx, amount, n;

	if (rxring->ring_readidx == AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_REG(ringidx), RX_DMA_DESC_HEAD_PTR))
		return 0;

#ifdef XXX_RXINTR_DEBUG
	//XXX: need lock
	printf("# %s:%d\n", __func__, __LINE__);
#endif

#ifdef XXX_DUMP_RX_COUNTER
	printf("RXPKT:%lu, RXBYTE:%lu, DMADROP:%u\n",
	    AQ_READ64_REG(sc, RX_DMA_GOOD_PKT_COUNTERLSW),
	    AQ_READ64_REG(sc, RX_DMA_GOOD_OCTET_COUNTERLSW),
	    AQ_READ_REG(sc, RX_DMA_DROP_PKT_CNT_REG));
#endif

#ifdef XXX_RXINTR_DEBUG
	printf("%s:%d: begin: RX_DMA_DESC_HEAD/TAIL=%lu/%u, readidx=%u\n", __func__, __LINE__,
	    AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_REG(ringidx), RX_DMA_DESC_HEAD_PTR),
	    AQ_READ_REG(sc, RX_DMA_DESC_TAIL_PTR_REG(ringidx)), rxring->ring_readidx);
#endif

	m0 = mprev = NULL;
	amount = 0;
	for (idx = rxring->ring_readidx, n = 0;
	    idx != AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_REG(ringidx), RX_DMA_DESC_HEAD_PTR);
	    idx = RXRING_NEXTIDX(idx), n++) {

		bus_dmamap_sync(sc->sc_dmat, rxring->ring_rxdesc_dmamap,
		    sizeof(aq_rx_desc_t) * idx, sizeof(aq_rx_desc_t),
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

		rxd = &rxring->ring_rxdesc[idx];
		rxd_status = le16toh(rxd->wb.status);

		if ((rxd_status & RXDESC_STATUS_DD) == 0)
			break;	/* not yet done */

		rxd_type = le32toh(rxd->wb.type);
		rxd_pktlen = le16toh(rxd->wb.pkt_len);
		rxd_nextdescptr = le16toh(rxd->wb.next_desc_ptr);
		rxd_hash = le32toh(rxd->wb.rss_hash);
		rxd_vlan = le16toh(rxd->wb.vlan);

		if (((rxd_status & RXDESC_STATUS_MAC_DMA_ERR) != 0) &&
		    ((rxd_type & RXDESC_TYPE_RDM_ERR) != 0)) {
			//XXX
			printf("RDM_ERR: desc[%d] type=0x%08x, hash=0x%08x, status=0x%08x, pktlen=%u, nextdesc=%u, vlan=0x%x\n",
			    idx, rxd_type, rxd_hash, rxd_status, rxd_pktlen, rxd_nextdescptr, rxd_vlan);
			printf("RDM_ERR: type: rsstype=0x%lx, rdm=%ld, ipv4checked=%ld, tcpudpchecked=%ld, sph=%ld, hdrlen=%ld\n",
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_RSSTYPE),
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_RDM_ERR),
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_IPV4_CSUM_CHECKED),
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_TCPUDP_CSUM_CHECKED),
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_SPH),
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_HDR_LEN));
			aq_rxring_reset_desc(sc, rxring, idx);
			goto rx_next;
		}

#ifdef XXX_RXDESC_DEBUG
		{
			const char * const rsstype_tbl[15] = {
				[RXDESC_TYPE_RSSTYPE_NONE] = "none",
				[RXDESC_TYPE_RSSTYPE_IPV4] = "ipv4",
				[RXDESC_TYPE_RSSTYPE_IPV6] = "ipv6",
				[RXDESC_TYPE_RSSTYPE_IPV4_TCP] = "ipv4-tcp",
				[RXDESC_TYPE_RSSTYPE_IPV6_TCP] = "ipv6-tcp",
				[RXDESC_TYPE_RSSTYPE_IPV4_UDP] = "ipv4-udp",
				[RXDESC_TYPE_RSSTYPE_IPV6_UDP] = "ipv6-udp",
			};
			const char * const pkttype_eth_table[4] = {
				"IPV4",
				"IPV6",
				"OTHERS",
				"ARP"
			};
			const char * const pkttype_proto_table[8] = {
				"TCP",
				"UDP",
				"SCTP",
				"ICMP",
				"PKTTYPE_PROTO4",
				"PKTTYPE_PROTO5", 
				"PKTTYPE_PROTO6", 
				"PKTTYPE_PROTO7"
			};

			const char *rsstype = rsstype_tbl[__SHIFTOUT(rxd_type, RXDESC_TYPE_RSSTYPE) & 15];
			if (rsstype == NULL)
				rsstype = "???";

			unsigned int pkttype_proto = __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE_PROTO);
			unsigned int pkttype_eth = __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE_ETHER);

			const char *csumstatus = "?";
			if (pkttype_eth == RXDESC_TYPE_PKTTYPE_ETHER_IPV4) {
				if (__SHIFTOUT(rxd_type, RXDESC_TYPE_IPV4_CSUM_CHECKED)) {
					csumstatus = "ipv4 checked";
					if (__SHIFTOUT(rxd_status, RXDESC_STATUS_L3_CSUM_NG) == 0) {
						csumstatus = "ipv4 csum OK";
					} else {
						csumstatus = "ipv4 csum NG";
					}
				} else {
					csumstatus = "ipv4 not checked";
				}
			}
			const char *l4csumstatus = "?";
			if (__SHIFTOUT(rxd_type, RXDESC_TYPE_TCPUDP_CSUM_CHECKED)) {
				l4csumstatus = "TCP/UDP checked";
				if (__SHIFTOUT(rxd_status, RXDESC_STATUS_L4_CSUM_ERROR)) {
					l4csumstatus = "TCP/UDP csum ERR";
				} else {
					if (__SHIFTOUT(rxd_status, RXDESC_STATUS_L4_CSUM_OK)) {
						l4csumstatus = "TCP/UDP csum OK";
					} else {
						l4csumstatus = "TCP/UDP csum NG";
					}
				}
			} else {
				l4csumstatus = "TCP/UDP not checked";
			}

			printf("RXdesc[%d]\n    type=0x%x, hash=0x%x, status=0x%x, DD=%lu, EOP=%lu, ERR=%lu, pktlen=%u, nextdsc=%u, vlan=%u, sph=%ld, hdrlen=%ld\n",
			    idx, rxd_type, rxd_hash, rxd_status,
			    __SHIFTOUT(rxd_status, RXDESC_STATUS_DD),
			    __SHIFTOUT(rxd_status, RXDESC_STATUS_EOP),
			    __SHIFTOUT(rxd_status, RXDESC_STATUS_MAC_DMA_ERR),
			    rxd_pktlen, rxd_nextdescptr, rxd_vlan,
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_SPH),
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_HDR_LEN));

			printf("    rsstype=0x%lx(%s), pkttype_vlan=%lu/%lu, pkttype_eth=%u(%s), pkttype_proto=%u(%s)\n",
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_RSSTYPE),
			    rsstype,
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE_VLAN),
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE_VLAN_DOUBLE),
			    pkttype_eth,
			    pkttype_eth_table[pkttype_eth],
			    pkttype_proto,
			    pkttype_proto_table[pkttype_proto]);

			printf("    v4chked=%ld,%s, l4chked=%ld,%s,%s\n",
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_IPV4_CSUM_CHECKED),
			    __SHIFTOUT(rxd_status, RXDESC_STATUS_L3_CSUM_NG) ? "NG" : "OK",
			    __SHIFTOUT(rxd_type, RXDESC_TYPE_TCPUDP_CSUM_CHECKED),
			    __SHIFTOUT(rxd_status, RXDESC_STATUS_L4_CSUM_ERROR) ? "ERR" : "NoERR",
			    __SHIFTOUT(rxd_status, RXDESC_STATUS_L4_CSUM_OK) ? "OK" : "NG");
			printf("    csumstatus=%s, l4csumstatus=%s\n", csumstatus, l4csumstatus);
		}
#endif /* XXX_RXDESC_DEBUG */

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

			if (m0 == m) {	/* XXX: do csum test for multi descriptors/JUMBO frame */
				if ((sc->sc_ethercom.ec_capenable & ETHERCAP_VLAN_HWTAGGING) &&
				    __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE_VLAN)) {
					vlan_set_tag(m0, rxd_vlan);
				}

				unsigned int pkttype_eth = __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE_ETHER);
				if ((ifp->if_capabilities & IFCAP_CSUM_IPv4_Rx) &&
				    (pkttype_eth == RXDESC_TYPE_PKTTYPE_ETHER_IPV4) &&
				    __SHIFTOUT(rxd_type, RXDESC_TYPE_IPV4_CSUM_CHECKED)) {
					m0->m_pkthdr.csum_flags |= M_CSUM_IPv4;
					if (__SHIFTOUT(rxd_status, RXDESC_STATUS_L3_CSUM_NG))
						m0->m_pkthdr.csum_flags |= M_CSUM_IPv4_BAD;
				}

#if notyet
				//XXX: NIC always marks BAD for fragmented packet? need to care.
				if (__SHIFTOUT(rxd_type, RXDESC_TYPE_TCPUDP_CSUM_CHECKED)) {
					bool need_result = false;
					unsigned int pkttype_proto = __SHIFTOUT(rxd_type, RXDESC_TYPE_PKTTYPE_PROTO);

					if (pkttype_proto == RXDESC_TYPE_PKTTYPE_PROTO_TCP) {
						if ((pkttype_eth == RXDESC_TYPE_PKTTYPE_ETHER_IPV4) && (ifp->if_capabilities & IFCAP_CSUM_TCPv4_Rx)) {
							m0->m_pkthdr.csum_flags |= M_CSUM_TCPv4;
							need_result = true;
						} else if ((pkttype_eth == RXDESC_TYPE_PKTTYPE_ETHER_IPV6) && (ifp->if_capabilities & IFCAP_CSUM_TCPv6_Rx)) {
							m0->m_pkthdr.csum_flags |= M_CSUM_TCPv6;
							need_result = true;
						}
					} else if (pkttype_proto == RXDESC_TYPE_PKTTYPE_PROTO_UDP) {
						if ((pkttype_eth == RXDESC_TYPE_PKTTYPE_ETHER_IPV4) && (ifp->if_capabilities & IFCAP_CSUM_UDPv4_Rx)) {
							m0->m_pkthdr.csum_flags |= M_CSUM_UDPv4;
							need_result = true;
						} else if ((pkttype_eth == RXDESC_TYPE_PKTTYPE_ETHER_IPV6) && (ifp->if_capabilities & IFCAP_CSUM_UDPv6_Rx)) {
							m0->m_pkthdr.csum_flags |= M_CSUM_UDPv6;
							need_result = true;
						}
					}
					if (need_result &&
					    (__SHIFTOUT(rxd_status, RXDESC_STATUS_L4_CSUM_ERROR) ||
					    !__SHIFTOUT(rxd_status, RXDESC_STATUS_L4_CSUM_OK))) {
						m0->m_pkthdr.csum_flags |= M_CSUM_TCP_UDP_BAD;
					}
				}
#endif
			}

			m_set_rcvif(m0, ifp);
			m0->m_pkthdr.len = amount;
			if_percpuq_enqueue(ifp->if_percpuq, m);

			m0 = mprev = NULL;
			amount = 0;
		}

		/* refill, and update tail */
		aq_rxring_add(sc, rxring, idx);
 rx_next:
		AQ_WRITE_REG(sc, RX_DMA_DESC_TAIL_PTR_REG(ringidx), idx);
	}
	rxring->ring_readidx = idx;

#ifdef XXX_RXINTR_DEBUG
	printf("%s:%d: end: RX_DMA_DESC_HEAD/TAIL=%lu/%u, readidx=%u\n", __func__, __LINE__,
	    AQ_READ_REG_BIT(sc, RX_DMA_DESC_HEAD_PTR_REG(ringidx), RX_DMA_DESC_HEAD_PTR),
	    AQ_READ_REG(sc, RX_DMA_DESC_TAIL_PTR_REG(ringidx)), rxring->ring_readidx);
#endif

	return n;
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

static int
aq_ifflags_cb(struct ethercom *ec)
{
	struct ifnet *ifp = &ec->ec_if;
	struct aq_softc *sc = ifp->if_softc;
	int i, ecchange, error = 0;
	unsigned short iffchange;

	//XXX: need lock
	printf("%s:%d\n", __func__, __LINE__);

	iffchange = ifp->if_flags ^ sc->sc_if_flags;
	if ((iffchange & IFF_PROMISC) != 0)
		error = aq_set_filter(sc);


	ecchange = ec->ec_capenable ^ sc->sc_ec_capenable;
	printf("old EC=%08x\n", sc->sc_ec_capenable);
	printf("new EC=%08x\n", ec->ec_capenable);
	printf("CHG EC=%08x\n", ecchange);
	if (ecchange & ETHERCAP_VLAN_HWTAGGING) {
		for (i = 0; i < AQ_RXRING_NUM; i++) {
			AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_REG(i), RX_DMA_DESC_VLAN_STRIP,
			    (ec->ec_capenable & ETHERCAP_VLAN_HWTAGGING) ? 1 : 0);
			printf("HWTAGGING[%d] -> %d\n", i, (ec->ec_capenable & ETHERCAP_VLAN_HWTAGGING) ? 1 : 0);
		}
	}

	sc->sc_ec_capenable = ec->ec_capenable;
	sc->sc_if_flags = ifp->if_flags;
	return error;
}

static int
aq_init(struct ifnet *ifp)
{
	struct aq_softc *sc = ifp->if_softc;
	int i, error = 0;

	//XXX: need lock
	printf("%s:%d\n", __func__, __LINE__);

	aq_update_vlan_filters(sc);
	aq_set_capability(sc);

	/* start TX */
	for (i = 0; i < sc->sc_txringnum; i++) {
		aq_txring_reset(sc, &sc->sc_txring[i], true);
	}
	AQ_WRITE_REG_BIT(sc, TPB_TX_BUF_REG, TPB_TX_BUF_EN, 1);

	/* start RX */
	for (i = 0; i < sc->sc_rxringnum; i++) {
		error = aq_rxring_reset(sc, &sc->sc_rxring[i], true);
		if (error != 0)
			goto aq_init_failure;
	}
	AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_REG, RPB_RPF_RX_BUF_EN, 1);

#ifdef XXX_DUMP_RING
	//XXX
	(void)&dump_txrings;
	(void)&dump_rxrings;
//	dump_txrings(sc);
//	dump_rxrings(sc);
#endif


//	aq_hw_start();
	aq_hw_rss_hash_set(sc);
	aq_hw_rss_set(sc);
	aq_hw_l3_filter_set(sc, sc->sc_l3_filter_enable);

	aq_enable_intr(sc, true, true);

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

	txring = &sc->sc_txring[0];	// XXX: select TX ring

#if 0
	printf("%s:%d: ringidx=%d, HEAD/TAIL=%lu/%u, INTR_MASK/INTR_STATUS=%08x/%08x\n",
	    __func__, __LINE__, txring->ring_index,
	    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_REG(txring->ring_index), TX_DMA_DESC_HEAD_PTR),
	    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_REG(txring->ring_index)),
	    AQ_READ_REG(sc, AQ_INTR_MASK_REG), AQ_READ_REG(sc, AQ_INTR_STATUS_REG));
#endif

	mutex_enter(&txring->ring_mutex);

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

	mutex_exit(&txring->ring_mutex);

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

	/* disable tx/rx interrupts */
	aq_enable_intr(sc, true, false);

	AQ_WRITE_REG_BIT(sc, TPB_TX_BUF_REG, TPB_TX_BUF_EN, 0);
	for (i = 0; i < sc->sc_txringnum; i++)
		aq_txring_reset(sc, &sc->sc_txring[i], false);

	AQ_WRITE_REG_BIT(sc, RPB_RPF_RX_REG, RPB_RPF_RX_BUF_EN, 0);
	for (i = 0; i < sc->sc_rxringnum; i++)
		aq_rxring_reset(sc, &sc->sc_rxring[i], false);

	/* invalidate RX descriptor cache */
	AQ_WRITE_REG_BIT(sc, RX_DMA_DESC_CACHE_INIT_REG, RX_DMA_DESC_CACHE_INIT,
	    AQ_READ_REG_BIT(sc, RX_DMA_DESC_CACHE_INIT_REG, RX_DMA_DESC_CACHE_INIT) ^ 1);

	ifp->if_timer = 0;

	if (!disable) {
		/* when pmf stop, disable link status intr, and callout */
		aq_enable_intr(sc, false, false);
#ifdef USE_CALLOUT_TICK
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
	status = AQ_READ_REG(sc, AQ_INTR_STATUS_REG);

	//XXX
	printf("####!!!! %s@cpu%d: INTR_MASK/INTR_STATUS = %08x/%08x=>%08x\n", __func__, cpu_index(curcpu()), AQ_READ_REG(sc, AQ_INTR_MASK_REG), status, AQ_READ_REG(sc, AQ_INTR_STATUS_REG));

	for (n = 0; n < sc->sc_txringnum; n++) {
		txring = &sc->sc_txring[n];

#if 1
		//DEBUG
		printf("%s:%d: ringidx=%d, HEAD/TAIL=%lu/%u\n", __func__, __LINE__, txring->ring_index,
		    AQ_READ_REG_BIT(sc, TX_DMA_DESC_HEAD_PTR_REG(txring->ring_index), TX_DMA_DESC_HEAD_PTR),
		    AQ_READ_REG(sc, TX_DMA_DESC_TAIL_PTR_REG(txring->ring_index)));
#endif

		aq_tx_intr(txring);
	}

	aq_init(ifp);
	//XXX
}

#ifdef XXX_DUMP_MACTABLE
static void
aq_dump_mactable(struct aq_softc *sc)
{
	int i;
	uint32_t h, l;

	for (i = 0; i <= AQ_HW_MAC_MAX; i++) {
		l = AQ_READ_REG(sc, RPF_L2UC_LSW_REG(i));
		h = AQ_READ_REG(sc, RPF_L2UC_MSW_REG(i));
		printf("MAC TABLE[%d] %02x:%02x:%02x:%02x:%02x:%02x enable=%d, actf=%ld\n",
		    i,
		    (h >> 8) & 0xff,
		    h & 0xff,
		    (l >> 24) & 0xff,
		    (l >> 16) & 0xff,
		    (l >> 8) & 0xff,
		    l & 0xff,
		    (h & RPF_L2UC_MSW_EN) ? 1 : 0,
		    AQ_READ_REG_BIT(sc, RPF_L2UC_MSW_REG(i), RPF_L2UC_MSW_ACTION));
	}
}
#endif

static int
aq_ioctl(struct ifnet *ifp, unsigned long cmd, void *data)
{
	struct aq_softc *sc __unused;
	struct ifreq *ifr __unused;
	int error, s;

	sc = (struct aq_softc *)ifp->if_softc;
	ifr = (struct ifreq *)data;
	error = 0;

	//XXX: need mutex lock
	s = splnet();
	error = ether_ioctl(ifp, cmd, data);
	splx(s);

	if (error != ENETRESET)
		return error;

	switch (cmd) {
	case SIOCSIFCAP:
		error = aq_set_capability(sc);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if ((ifp->if_flags & IFF_RUNNING) == 0)
			break;

		/*
		 * Multicast list has changed; set the hardware filter
		 * accordingly.
		 */
		error = aq_set_filter(sc);
#ifdef XXX_DUMP_MACTABLE
		aq_dump_mactable(sc);
#endif
		break;
	}

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
