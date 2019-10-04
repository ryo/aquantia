#if 0
static int __unused
aq_mpi_read_mbox(struct aq_softc *sc, struct aq_fw_mbox *mbox)
{
	return aq_fw_downld_dwords(sc, sc->sc_mbox_addr,
	    (uint32_t *)mbox, sizeof(*mbox) / sizeof(uint32_t));
}


static int
aq_mpi_set_state(struct aq_softc *sc, int state)
{
	struct aq_fw_mbox mbox;
	uint32_t v, id;
	int i;

	v = AQ_READ_REG(sc, HW_ATL_MPI_CONTROL_ADR);

	if (state == MPI_RESET) {
		memset(&mbox, 0, sizeof(mbox));
		aq_mpi_read_mbox(sc, &mbox);
		id = mbox.id;

		for (i = 10000; i >= 0; i--) {
			memset(&mbox, 0, sizeof(mbox));
			aq_mpi_read_mbox(sc, &mbox);
			if (id != mbox.id)
				break;
			delay(1000);
		}
		if (i <= 0)
			return ETIMEDOUT;
	}

	switch (state) {
	case MPI_DEINIT:
	case MPI_POWER:
		v |= HW_ATL_MPI_DIRTY_WAKE_MSK;
		break;
	default:
		v &= ~HW_ATL_MPI_DIRTY_WAKE_MSK;
		break;
	}

	v &= ~HW_ATL_MPI_STATE_MSK;
	v |= state & HW_ATL_MPI_STATE_MSK;

	AQ_WRITE_REG(sc, HW_ATL_MPI_CONTROL_ADR, v);
	return 0;
}
#endif

#if 0
static int
hw_atl_utils_soft_reset_flb(struct aq_softc *sc)
{
	uint32_t gsr, v;
	int k = 0;

	aprint_normal_dev(sc->sc_dev, "%s:%d\n", __func__, __LINE__);

	AQ_WRITE_REG(sc, HW_ATL_MCP_UP_FORCE_INTERRUPT_ADR, 0x40e1);
	delay(50000);

	/* Cleanup SPI */
	v = AQ_READ_REG(sc, 0x053c);
	AQ_WRITE_REG(sc, 0x053c, v | 0x10);

	gsr = AQ_READ_REG(sc, HW_ATL_GLB_SOFT_RES_ADR);
	AQ_WRITE_REG(sc, HW_ATL_GLB_SOFT_RES_ADR, (gsr & 0xbfff) | 0x8000);

	/* Kickstart MAC */
	AQ_WRITE_REG(sc, 0x0404, 0x80e0);
	AQ_WRITE_REG(sc, 0x32a8, 0x0000);
	AQ_WRITE_REG(sc, 0x0520, 0x0001);

	/* Reset SPI again because of possible interrupted SPI burst */
	v = AQ_READ_REG(sc, 0x053c);
	AQ_WRITE_REG(sc, 0x053c, v | 0x10);
	delay(10000);
	/* Clear SPI reset state */
	AQ_WRITE_REG(sc, 0x053c, v & ~0x10);

	AQ_WRITE_REG(sc, 0x0404, 0x000180e0);

	for (k = 0; k < 1000; k++) {
		uint32_t flb_status = AQ_READ_REG(sc, HW_ATL_MPI_DAISY_CHAIN_STATUS);

		flb_status = flb_status & 0x10;
		if (flb_status)
			break;
		delay(10000);
	}
	if (k == 1000) {
		aprint_error_dev(sc->sc_dev, "%s:%d: MAC kickstart failed\n", __func__, __LINE__);
		return -EIO;
	}

	/* FW reset */
	AQ_WRITE_REG(sc, 0x0404, 0x80e0);
	delay(50000);
	AQ_WRITE_REG(sc, 0x03a0, 0x0001);

	/* Kickstart PHY - skipped */

	/* Global software reset*/
	v = AQ_READ_REG(sc, HW_ATL_RX_REG_RES_DSBL_ADR);	// RX disable
	v &= ~__BIT(29);
	AQ_WRITE_REG(sc, HW_ATL_RX_REG_RES_DSBL_ADR, v);

	v = AQ_READ_REG(sc, HW_ATL_TX_REG_RES_DSBL_ADR);	// TX disable
	v &= ~__BIT(29);
	AQ_WRITE_REG(sc, HW_ATL_TX_REG_RES_DSBL_ADR, v);

	v = AQ_READ_REG(sc, HW_ATL_MAC_PHY_CONTROL);
	v &= ~__BIT(29);
	AQ_WRITE_REG(sc, HW_ATL_MAC_PHY_CONTROL, v);

	gsr = AQ_READ_REG(sc, HW_ATL_GLB_SOFT_RES_ADR);
	AQ_WRITE_REG(sc, HW_ATL_GLB_SOFT_RES_ADR, (gsr & 0xbfff) | 0x8000);

	for (k = 0; k < 1000; k++) {
		uint32_t fw_state = AQ_READ_REG(sc, HW_ATL_MPI_FW_VERSION);

		if (fw_state)
			break;
		delay(10000);
	}
	if (k == 1000) {
		aprint_error_dev(sc->sc_dev, "%s:%d: FW kickstart failed\n", __func__, __LINE__);
		return -EIO;
	}
	/* Old FW requires fixed delay after init */
	msec_delay(15);

	return 0;
}
#endif

#if 0
static int __unused
aq_init_ucp(struct aq_softc *sc)
{
	int n;

	if (AQ_READ_REG(sc, HW_ATL_UCP_0X370_REG) == 0) {
		uint32_t data;
		cprng_fast(&data, sizeof(data));
		data &= 0xfefefefe;
		data |= 0x02020202;
		AQ_WRITE_REG(sc, HW_ATL_UCP_0X370_REG, data);
	}

	AQ_WRITE_REG(sc, HW_ATL_GLB_CPU_SCRATCH_SCP_ADR(25), 0);
	for (n = 10000; n >= 0; n--) {
		sc->sc_mbox_addr = AQ_READ_REG(sc, HW_ATL_GLB_CPU_SCRATCH_SCP_ADR(24));
		if (sc->sc_mbox_addr != 0)
			break;
		delay(1000);
	}
	if (n <= 0)
		return ETIMEDOUT;

	aprint_debug_dev(sc->sc_dev, "MBOX_ADDR=%08x\n", sc->sc_mbox_addr);

	return 0;
}
#endif

#if 0
static void __unused
aq_init(struct aq_softc *sc)
{
	uint32_t v;
	int i;

	AQ_WRITE_REG(sc, 0x0404, 0x40e1);
	delay(100);

	/* reset spi */
	v = AQ_READ_REG(sc, 0x053c);
	AQ_WRITE_REG(sc, 0x053c, v | 0x10);

	v = AQ_READ_REG(sc, HW_ATL_GLB_SOFT_RES_ADR);
	AQ_WRITE_REG(sc, HW_ATL_GLB_SOFT_RES_ADR, (v & 0xbfff) | 0x8000);

	/* kickstart mac */
	AQ_WRITE_REG(sc, 0x0404, 0x80e0);
	AQ_WRITE_REG(sc, 0x32a8, 0x0000);
	AQ_WRITE_REG(sc, 0x0520, 0x0001);

	/* reset spi again */
	v = AQ_READ_REG(sc, 0x053c);
	AQ_WRITE_REG(sc, 0x053c, v | 0x10);
	delay(100);
	AQ_WRITE_REG(sc, 0x053c, v & ~0x10);

	AQ_WRITE_REG(sc, 0x0404, 0x180e0);

	for (i = 1000; i > 0; i--) {
		if (AQ_READ_REG(sc, HW_ATL_MPI_DAISY_CHAIN_STATUS) & 0x10)
			break;
		delay(10);
	}
	if (i <= 0) {
		aprint_error_dev(sc->sc_dev, "%s:%d: kickstart failed\n", __func__, __LINE__);
		goto failure;
	}

	/* FW reset */
	AQ_WRITE_REG(sc, 0x0404, 0x80e0);
	delay(100);
	AQ_WRITE_REG(sc, 0x03a0, 0x0001);

	/* Global software reset*/
	v = AQ_READ_REG(sc, HW_ATL_RX_REG_RES_DSBL_ADR);
	v &= ~__BIT(29);
	AQ_WRITE_REG(sc, HW_ATL_RX_REG_RES_DSBL_ADR, v);

	v = AQ_READ_REG(sc, HW_ATL_TX_REG_RES_DSBL_ADR);
	v &= ~__BIT(29);
	AQ_WRITE_REG(sc, HW_ATL_TX_REG_RES_DSBL_ADR, v);

	v = AQ_READ_REG(sc, HW_ATL_MAC_PHY_CONTROL);
	v &= ~HW_ATL_MAC_PHY_MPI_RESET_BIT;
	AQ_WRITE_REG(sc, HW_ATL_MAC_PHY_CONTROL, v);

	v = AQ_READ_REG(sc, HW_ATL_GLB_SOFT_RES_ADR);
	AQ_WRITE_REG(sc, HW_ATL_GLB_SOFT_RES_ADR, (v & 0xbfff) | 0x8000);

	for (i = 1000; i > 0; i--) {
		if (AQ_READ_REG(sc, HW_ATL_MPI_FW_VERSION) != 0)
			break;
		delay(10);
	}
	if (i <= 0) {
		aprint_error_dev(sc->sc_dev, "%s:%d: kickstart failed\n", __func__, __LINE__);
		goto failure;
	}

	delay(100);

 failure:
	aprint_debug_dev(sc->sc_dev, "%s:%d\n", __func__, __LINE__);
}
#endif


