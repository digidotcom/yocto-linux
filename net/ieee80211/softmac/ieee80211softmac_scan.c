/*
 * Scanning routines.
 *
 * These are not exported because they're assigned to the function pointers.
 *
 * Copyright (c) 2005, 2006 Johannes Berg <johannes@sipsolutions.net>
 *                          Joseph Jezak <josejx@gentoo.org>
 *                          Larry Finger <Larry.Finger@lwfinger.net>
 *                          Danny van Dyk <kugelfang@gentoo.org>
 *                          Michael Buesch <mbuesch@freenet.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * The full GNU General Public License is included in this distribution in the
 * file called COPYING.
 */

#include <linux/completion.h>
#include "ieee80211softmac_priv.h"

/* internal, use to trigger scanning if needed.
 * Returns -EBUSY if already scanning,
 * result of start_scan otherwise */
int
ieee80211softmac_start_scan(struct ieee80211softmac_device *sm)
{
        struct ieee80211_network *net = NULL;
        struct ieee80211_network *nettmp = NULL;
        struct ieee80211softmac_network *netptr, *nettmp_sm;
	unsigned long flags;
        unsigned long flags2;
	int ret;

	spin_lock_irqsave(&sm->lock, flags);
	if (sm->scanning)
	{
		spin_unlock_irqrestore(&sm->lock, flags);
		return -EINPROGRESS;
	}
	sm->scanning = 1;
        sm->scan.had_probe_resp   = 0;
        sm->scan.wait_cycles_left = 0;

        if( !sm->associnfo.associated ) {
                /* Delete all existing networks. This is necessary to overcome
                 * stale entries that are not active any longer but are still in
                 * list with good values. The association logic based on RSSI
                 * might decide to switch to that AP when a new but worse
                 * active entry is present.  all ieee802.11 networks */
                spin_lock_irqsave(&sm->ieee->lock, flags2);
                list_for_each_entry_safe(net, nettmp, &sm->ieee->network_list, list) {
                        list_move(&net->list, &sm->ieee->network_free_list);
                }
                spin_unlock_irqrestore(&sm->ieee->lock, flags2);

                /* all softmac networks */
                list_for_each_entry_safe(netptr, nettmp_sm, &sm->network_list, list) {
                        ieee80211softmac_del_network_locked(sm, netptr);
                        if(netptr->challenge != NULL)
                                kfree(netptr->challenge);
                        kfree(netptr);
                }
        }
	spin_unlock_irqrestore(&sm->lock, flags);

	ret = sm->start_scan(sm->dev);
	if (ret) {
		spin_lock_irqsave(&sm->lock, flags);
		sm->scanning = 0;
		spin_unlock_irqrestore(&sm->lock, flags);
	}
	return ret;
}

void
ieee80211softmac_stop_scan(struct ieee80211softmac_device *sm)
{
	unsigned long flags;

	spin_lock_irqsave(&sm->lock, flags);

	if (!sm->scanning) {
		spin_unlock_irqrestore(&sm->lock, flags);
		return;
	}

	spin_unlock_irqrestore(&sm->lock, flags);
	sm->stop_scan(sm->dev);
}

void
ieee80211softmac_wait_for_scan(struct ieee80211softmac_device *sm)
{
	unsigned long flags;

	spin_lock_irqsave(&sm->lock, flags);

	if (!sm->scanning) {
		spin_unlock_irqrestore(&sm->lock, flags);
		return;
	}

	spin_unlock_irqrestore(&sm->lock, flags);
	sm->wait_for_scan(sm->dev);
}


/* internal scanning implementation follows */
void ieee80211softmac_scan(struct work_struct *work)
{
	int invalid_channel;
	u8 current_channel_idx;
	struct ieee80211softmac_scaninfo *si =
		container_of(work, struct ieee80211softmac_scaninfo,
	                     softmac_scan.work);

	struct ieee80211softmac_device *sm = si->mac;
	unsigned long flags;

        /* waiting a fixed time for a probe response is maybe not the best
         * solution. We probably won't catch all if there are many. Therefore,
         * if we had a probe response, we wait a tick longer. Maybe another AP
         * responds as well, but was too slow for that time slice. */
        spin_lock_irqsave(&sm->lock, flags);
        if( sm->scan.had_probe_resp && sm->scan.wait_cycles_left ) {
                sm->scan.had_probe_resp = 0;
                sm->scan.wait_cycles_left--;
		queue_delayed_work(si->mac->wq, &si->softmac_scan, IEEE80211SOFTMAC_PROBE_DELAY);
                spin_unlock_irqrestore(&sm->lock, flags);
                return;
        }
        spin_unlock_irqrestore(&sm->lock, flags);

	while (!(si->stop) && (si->current_channel_idx < si->number_channels)) {
		current_channel_idx = si->current_channel_idx;
		si->current_channel_idx++; /* go to the next channel */

		invalid_channel = (si->skip_flags & si->channels[current_channel_idx].flags);

		if (!invalid_channel) {
			spin_lock_irqsave(&sm->lock, flags);
                        /* limit cycles we wait for probe responses so that
                           we never hang. */
                        sm->scan.wait_cycles_left = 3;
                        sm->scan.had_probe_resp   = 0;
                        spin_unlock_irqrestore(&sm->lock, flags);

			sm->set_channel(sm->dev, si->channels[current_channel_idx].channel);
			// FIXME make this user configurable (active/passive)
			if(ieee80211softmac_send_mgt_frame(sm, NULL, IEEE80211_STYPE_PROBE_REQ, 0))
				printkl(KERN_DEBUG PFX "Sending Probe Request Failed\n");

			/* also send directed management frame for the network we're looking for */
			// TODO: is this if correct, or should we do this only if scanning from assoc request?
			if (sm->associnfo.req_essid.len)
				ieee80211softmac_send_mgt_frame(sm, &sm->associnfo.req_essid, IEEE80211_STYPE_PROBE_REQ, 0);

			spin_lock_irqsave(&sm->lock, flags);
			if (unlikely(!sm->running)) {
				/* Prevent reschedule on workqueue flush */
				spin_unlock_irqrestore(&sm->lock, flags);
				break;
			}
			queue_delayed_work(si->mac->wq, &si->softmac_scan, IEEE80211SOFTMAC_PROBE_DELAY);
			spin_unlock_irqrestore(&sm->lock, flags);
			return;
		} else {
			dprintk(PFX "Not probing Channel %d (not allowed here)\n", si->channels[current_channel_idx].channel);
		}
	}

	spin_lock_irqsave(&sm->lock, flags);
	cancel_delayed_work(&si->softmac_scan);
	si->started = 0;
	spin_unlock_irqrestore(&sm->lock, flags);

	dprintk(PFX "Scanning finished: scanned %d channels starting with channel %d\n",
		     sm->scaninfo->number_channels, sm->scaninfo->channels[0].channel);
	ieee80211softmac_scan_finished(sm);
	complete_all(&sm->scaninfo->finished);
}

static inline struct ieee80211softmac_scaninfo *allocate_scaninfo(struct ieee80211softmac_device *mac)
{
	/* ugh. can we call this without having the spinlock held? */
	struct ieee80211softmac_scaninfo *info = kmalloc(sizeof(struct ieee80211softmac_scaninfo), GFP_ATOMIC);
	if (unlikely(!info))
		return NULL;

	INIT_DELAYED_WORK(&info->softmac_scan, ieee80211softmac_scan);

	info->mac = mac;
	init_completion(&info->finished);
	return info;
}

int ieee80211softmac_start_scan_implementation(struct net_device *dev)
{
	struct ieee80211softmac_device *sm = ieee80211_priv(dev);
	unsigned long flags;

	if (!(dev->flags & IFF_UP))
		return -ENODEV;

	assert(ieee80211softmac_scan_handlers_check_self(sm));
	if (!ieee80211softmac_scan_handlers_check_self(sm))
		return -EINVAL;

	spin_lock_irqsave(&sm->lock, flags);
	/* it looks like we need to hold the lock here
	 * to make sure we don't allocate two of these... */
	if (unlikely(!sm->scaninfo))
		sm->scaninfo = allocate_scaninfo(sm);
	if (unlikely(!sm->scaninfo)) {
		spin_unlock_irqrestore(&sm->lock, flags);
		return -ENOMEM;
	}

	sm->scaninfo->skip_flags = IEEE80211_CH_INVALID;
	if (0 /* not scanning in IEEE802.11b */)//TODO
		sm->scaninfo->skip_flags |= IEEE80211_CH_B_ONLY;
	if (0 /* IEEE802.11a */) {//TODO
		sm->scaninfo->channels = sm->ieee->geo.a;
		sm->scaninfo->number_channels = sm->ieee->geo.a_channels;
	} else {
		sm->scaninfo->channels = sm->ieee->geo.bg;
		sm->scaninfo->number_channels = sm->ieee->geo.bg_channels;
	}
	sm->scaninfo->current_channel_idx = 0;
	sm->scaninfo->started = 1;
	sm->scaninfo->stop = 0;
	INIT_COMPLETION(sm->scaninfo->finished);
	if (sm->set_bssid_filter) {
		u8 bc_addr[ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
		sm->set_bssid_filter(sm->dev, bc_addr);
	}
	queue_delayed_work(sm->wq, &sm->scaninfo->softmac_scan, 0);
	spin_unlock_irqrestore(&sm->lock, flags);
	return 0;
}

void ieee80211softmac_stop_scan_implementation(struct net_device *dev)
{
	struct ieee80211softmac_device *sm = ieee80211_priv(dev);
	unsigned long flags;

	assert(ieee80211softmac_scan_handlers_check_self(sm));
	if (!ieee80211softmac_scan_handlers_check_self(sm))
		return;

	spin_lock_irqsave(&sm->lock, flags);
	assert(sm->scaninfo != NULL);
	if (sm->scaninfo) {
		if (sm->scaninfo->started)
			sm->scaninfo->stop = 1;
		else
			complete_all(&sm->scaninfo->finished);
	}
	spin_unlock_irqrestore(&sm->lock, flags);
}

void ieee80211softmac_wait_for_scan_implementation(struct net_device *dev)
{
	struct ieee80211softmac_device *sm = ieee80211_priv(dev);
	unsigned long flags;

	assert(ieee80211softmac_scan_handlers_check_self(sm));
	if (!ieee80211softmac_scan_handlers_check_self(sm))
		return;

	spin_lock_irqsave(&sm->lock, flags);
	if (!sm->scaninfo->started) {
		spin_unlock_irqrestore(&sm->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&sm->lock, flags);
	wait_for_completion(&sm->scaninfo->finished);
}

/* this is what drivers (that do scanning) call when they're done */
void ieee80211softmac_scan_finished(struct ieee80211softmac_device *sm)
{
	unsigned long flags;

	spin_lock_irqsave(&sm->lock, flags);
	sm->scanning = 0;
	spin_unlock_irqrestore(&sm->lock, flags);

	if (sm->associnfo.bssvalid) {
		struct ieee80211softmac_network *net;

		net = ieee80211softmac_get_network_by_bssid_with_channel(sm, sm->associnfo.bssid, sm->associnfo.channel);
		if (net) {
			sm->set_channel(sm->dev, net->channel);
			if (sm->set_bssid_filter)
				sm->set_bssid_filter(sm->dev, net->bssid);
		}
	}
	ieee80211softmac_call_events(sm, IEEE80211SOFTMAC_EVENT_SCAN_FINISHED, NULL);
}
EXPORT_SYMBOL_GPL(ieee80211softmac_scan_finished);

/* Probe Response handling */
int ieee80211softmac_handle_probe_resp(
        struct net_device *dev,
	struct ieee80211_beacon *beacon,
	struct ieee80211_network *network )
{
	struct ieee80211softmac_device *sm = ieee80211_priv(dev);

        unsigned long flags;

        spin_lock_irqsave(&sm->lock, flags);
        /* it's unlikely that we overrun it while scanning */
        sm->scan.had_probe_resp++;
        spin_unlock_irqrestore(&sm->lock, flags);

        return 0;
}

