/*
 *  net/wireless/digi_wi_g_priv_handler.c
 *
 *  Copyright (C) 2007 by Digi International Inc.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version2  as published by
 *  the Free Software Foundation.
*/
/*
 *  !Revision:   $Revision: 1.1 $
 *  !Author:     Markus Pietrek
 *  !Descr:      Contains private user handler stuff.
*/

#warning fix the getter functions, they compare 0 instead of '0'

/***********************************************************************
 * @Function: digi_wi_g_wx_set_swencryption
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_set_swencryption(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	char on;
	
        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	on = *extra;

	digi_wi_g_lock(priv, flags);
	if (on)
		priv->hw_aes = 0;
	else 
		priv->hw_aes = 1;
	digi_wi_g_unlock(priv, flags);

	return 0;
}


/***********************************************************************
 * @Function: digi_wi_g_wx_get_swencryption
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_get_swencryption(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	int on;

        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	digi_wi_g_lock(priv, flags);
	if (priv->hw_aes)
		on = 0;
	else
		on = 1;
	digi_wi_g_unlock(priv, flags);

	if (on)
		strncpy(extra, "1 (SW encryption enabled) ", MAX_WX_STRING);
	else
		strncpy(extra, "0 (SW encryption disabled) ", MAX_WX_STRING);
	data->data.length = strlen(extra + 1);

	return 0;
}


/***********************************************************************
 * @Function: digi_wi_g_wx_set_ant_div
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_set_ant_div(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	char on;
	
        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	on = *extra;

	digi_wi_g_lock(priv, flags);
	if (on) {
		priv->ant_div = 1;
		HW_GEN_CONTROL |= GEN_ANTDIV;
	} else {
		priv->ant_div = 0;
		HW_GEN_CONTROL &= ~GEN_ANTDIV;
	}
	digi_wi_g_unlock(priv, flags);

	return 0;
}


/***********************************************************************
 * @Function: digi_wi_g_wx_get_ant_div
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_get_ant_div(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	int on;

        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	digi_wi_g_lock(priv, flags);
	if (priv->ant_div)
		on = 1;
	else
		on = 0;
	digi_wi_g_unlock(priv, flags);

	if (on)
		strncpy(extra, "1 (Antenna Diversity is enabled) ", MAX_WX_STRING);
	else
		strncpy(extra, "0 (Antenna Diversity is disabled) ", MAX_WX_STRING);
	data->data.length = strlen(extra + 1);

	return 0;
}


/***********************************************************************
 * @Function: digi_wi_g_wx_set_antenna
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_set_antenna(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	char ant;
	
        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	ant = *extra;

	if ((ant == 0) || (ant == 1)) {
		digi_wi_g_lock(priv, flags);
		if (ant == 0) {
			priv->antenna = 0;
			HW_GEN_CONTROL &= ~GEN_ANTSEL;
		} else {
			priv->antenna = 1;
			HW_GEN_CONTROL |= GEN_ANTSEL;
		}
		digi_wi_g_unlock(priv, flags);
	} else {
		printk(PFX KERN_ERR "Value should be 0 or 1.\n");
		return -EOPNOTSUPP;
	}
	return 0;
}


/***********************************************************************
 * @Function: digi_wi_g_wx_get_antenna
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_get_antenna(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	int ant;

        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	digi_wi_g_lock(priv, flags);
	ant = priv->antenna;
	digi_wi_g_unlock(priv, flags);

	if (ant == 0)
		strncpy(extra, "0 (Antenna 1 selected) ", MAX_WX_STRING);
	else
		strncpy(extra, "1 (Antenna 2 selected) ", MAX_WX_STRING);
	data->data.length = strlen(extra + 1);

	return 0;
}


/***********************************************************************
 * @Function: digi_wi_g_wx_set_rx_off
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_set_rx_off(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	char rx_off;
	
        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	rx_off = *extra;

	if ((rx_off == 0) || (rx_off == 1)) {
		digi_wi_g_lock(priv, flags);
		if (rx_off == 0) {
			priv->rx_off = 0;
		} else {
			priv->rx_off = 1;
		}
		digi_wi_g_unlock(priv, flags);
	} else {
		printk(PFX KERN_ERR "Value should be 0 or 1.\n");
		return -EOPNOTSUPP;
	}
	return 0;
}


/***********************************************************************
 * @Function: digi_wi_g_wx_get_rx_off
 * @Return:
 * @Descr:
 ***********************************************************************/
static int digi_wi_g_wx_get_rx_off(struct net_device *dev,
				       struct iw_request_info *info,
				       union iwreq_data *data,
				       char *extra)
{
	struct digi_wi_g_private *priv = ieee80211softmac_priv(dev);
	unsigned long flags;
	int rx_off;

        DBG_FN( DBG_INTERFACE | DBG_MINOR );

	digi_wi_g_lock(priv, flags);
	rx_off = priv->rx_off;
	digi_wi_g_unlock(priv, flags);

	if (rx_off == 0)
		strncpy(extra, "0 (rx is on) ", MAX_WX_STRING);
	else
		strncpy(extra, "1 (rx is off) ", MAX_WX_STRING);
	data->data.length = strlen(extra + 1);

	return 0;
}

static const iw_handler digi_wi_g_priv_wx_handlers[] = {
	/* Enable/Disable Software Encryption mode */
	digi_wi_g_wx_set_swencryption,
	/* Get Software Encryption mode */
	digi_wi_g_wx_get_swencryption,
	/* Enable/Disable Antenna Diversity mode */
	digi_wi_g_wx_set_ant_div,
	/* Get Antenna Diversity mode */
	digi_wi_g_wx_get_ant_div,
	/* Select Antenna */
	digi_wi_g_wx_set_antenna,
	/* Get Antenna */
	digi_wi_g_wx_get_antenna,
	/* Set rx off */
	digi_wi_g_wx_set_rx_off,
	/* Get rx off */
	digi_wi_g_wx_get_rx_off,
};

#define PRIV_WX_SET_SWENCRYPTION	(SIOCIWFIRSTPRIV + 0)
#define PRIV_WX_GET_SWENCRYPTION	(SIOCIWFIRSTPRIV + 1)
#define PRIV_WX_SET_ANTDIV		(SIOCIWFIRSTPRIV + 2)
#define PRIV_WX_GET_ANTDIV		(SIOCIWFIRSTPRIV + 3)
#define PRIV_WX_SET_ANTSEL		(SIOCIWFIRSTPRIV + 4)
#define PRIV_WX_GET_ANTSEL		(SIOCIWFIRSTPRIV + 5)
#define PRIV_WX_SET_RX_OFF		(SIOCIWFIRSTPRIV + 6)
#define PRIV_WX_GET_RX_OFF		(SIOCIWFIRSTPRIV + 7)

static const struct iw_priv_args digi_wi_g_priv_wx_args[] = {
	{
		.cmd		= PRIV_WX_SET_SWENCRYPTION,
		.set_args	= IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
		.name		= "set_swencrypt",
	},
	{
		.cmd		= PRIV_WX_GET_SWENCRYPTION,
		.get_args	= IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		.name		= "get_swencrypt",
	},
	{
		.cmd		= PRIV_WX_SET_ANTDIV,
		.set_args	= IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
		.name		= "set_ant_div",
	},
	{
		.cmd		= PRIV_WX_GET_ANTDIV,
		.get_args	= IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		.name		= "get_ant_div",
	},
	{
		.cmd		= PRIV_WX_SET_ANTSEL,
		.set_args	= IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
		.name		= "set_ant_sel",
	},
	{
		.cmd		= PRIV_WX_GET_ANTSEL,
		.get_args	= IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		.name		= "get_ant_sel",
	},
	{
		.cmd		= PRIV_WX_SET_RX_OFF,
		.set_args	= IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
		.name		= "set_rx_off",
	},
	{
		.cmd		= PRIV_WX_GET_RX_OFF,
		.get_args	= IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		.name		= "get_rx_off",
	},
};

