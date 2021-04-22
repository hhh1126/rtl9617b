/*
 * Copyright (c) 2016 Quantenna Communications, Inc.
 * All rights reserved.
 */
#ifndef __QTN_MONITOR_H__
#define __QTN_MONITOR_H__

#if defined(MUC_BUILD)

struct qtn_softc;
struct qtn_hal_rxstatus;

struct qtn_monitor_regs {
	uint32_t rx_csr_reg;
	uint32_t rx_cntl_allow_reg;
};

void qtn_monitor_mode_attach(struct qtn_softc *sc);
void qtn_monitor_mode_detach(struct qtn_softc *sc);
void qtn_monitor_mode_setup(struct qtn_softc *sc, uint32_t value);

#define MONITOR_STATE_ON 1
#define MONITOR_STATE_OFF 0

#endif	/* defined(MUC_BUILD) */

/**
 * @addtogroup NonAssociatedClientsAPIs
 * @{
 */

/** Default monitor period as a percentage. */
#define MONITOR_DEFAULT_ON_PERIOD	10

/** Minimum monitor period as a percentage. */
#define MONITOR_MIN_ON_PERIOD		1

/** Maximum monitor period as a percentage. */
#define MONITOR_MAX_ON_PERIOD		99

/** Default cycle period in milliseconds. */
#define MONITOR_DEFAULT_CYCLE_PERIOD	1000

/** Minimum cycle period in milliseconds. */
#define MONITOR_MIN_CYCLE_PERIOD	100

/** Maximum cycle period in milliseconds. */
#define MONITOR_MAX_CYCLE_PERIOD	5000

/** @} */

#define MONITOR_ENABLE_MASK		0x000000FF
#define MONITOR_ENABLE_SHIFT		0
#define MONITOR_MAKE_ENABLE(val)	(((val) << MONITOR_ENABLE_SHIFT) & MONITOR_ENABLE_MASK)
#define MONITOR_GET_ENABLE(val)		(((val) & MONITOR_ENABLE_MASK) >> MONITOR_ENABLE_SHIFT)

#define MONITOR_ON_PERIOD_MASK		0x0000FF00
#define MONITOR_ON_PERIOD_SHIFT		8
#define MONITOR_MAKE_ON_PERIOD(val)	(((val) << MONITOR_ON_PERIOD_SHIFT) & MONITOR_ON_PERIOD_MASK)
#define MONITOR_GET_ON_PERIOD(val)	(((val) & MONITOR_ON_PERIOD_MASK) >> MONITOR_ON_PERIOD_SHIFT)

#define MONITOR_CYCLE_PERIOD_MASK	0xFFFF0000
#define MONITOR_CYCLE_PERIOD_SHIFT	16
#define MONITOR_MAKE_CYCLE_PERIOD(val)	(((val) << MONITOR_CYCLE_PERIOD_SHIFT) & MONITOR_CYCLE_PERIOD_MASK)
#define MONITOR_GET_CYCLE_PERIOD(val)	(((val) & MONITOR_CYCLE_PERIOD_MASK) >> MONITOR_CYCLE_PERIOD_SHIFT)

#define MONITOR_DEFAULT_CONFIG		(\
			MONITOR_MAKE_CYCLE_PERIOD(MONITOR_DEFAULT_CYCLE_PERIOD) | \
			MONITOR_MAKE_ON_PERIOD(MONITOR_DEFAULT_ON_PERIOD))

#define IEEE80211_NAC_PERCENT_TO_INT	100
#endif /* __QTN_MONITOR_H__ */

