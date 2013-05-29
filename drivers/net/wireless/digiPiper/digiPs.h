/*
 * Header file for digiPs.c.  Declares functions used for power save mode.
 */

#ifndef digiPs_h

#define digiPS_h

#include <net/mac80211.h>
#include "pipermain.h"

enum piper_ps_event {
	PS_EVENT_BEACON_RECEIVED,
	PS_EVENT_STOP_TRANSMIT_TIMER_EXPIRED,
	PS_EVENT_TRANSMITTER_DONE,
    PS_EVENT_TRANSMITTER_DONE_TIMER_TICK,
	PS_EVENT_WAKEUP,
	PS_EVENT_MISSED_BEACON
};

enum piper_ps_tx_completion_result {
	PS_RETURN_SKB_TO_MAC80211,
	PS_DONT_RETURN_SKB_TO_MAC80211
};

enum piper_ps_active_result {
	PS_CONTINUE_TRANSMIT,
	PS_STOP_TRANSMIT
};


/*
 * Current version of mac80211 doesn't set power management bit in frame headers,
 * so I guess we have to for now.
 *
 * TODO:  See if we still have to do this in the next drop.
 */
#define piper_ps_set_header_flag(piperp, header) 	\
			header->fc.pwrMgt = (piperp->ps.power_management)

int piper_ps_active(struct piper_priv *piperp);
void piper_ps_process_receive_frame(struct piper_priv *piperp,
				    struct sk_buff *skb);
void piper_ps_init(struct piper_priv *piperp);
void piper_ps_deinit(struct piper_priv *piperp);
void piper_ps_set(struct piper_priv *piperp, bool powerSaveOn);
struct ieee80211_rate *piper_ps_check_rate(struct piper_priv *piperp,
					   struct ieee80211_rate *rate);

#endif
