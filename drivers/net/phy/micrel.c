/*
 * drivers/net/phy/micrel.c
 *
 * Driver for Micrel PHYs
 *
 * Author: David J. Choi
 *
 * Copyright (c) 2010 Micrel, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support : ksz9021 , vsc8201, ks8001, KSZ8031RNL
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/micrel_phy.h>

/* Operation Mode Strap Override register. */
#define MII_KSZPHY_OMSO			0x16
#define KSZ8031_OMSO_ADDR_FIX		0x0200	/* undocumented */

/* PHY Control 2 register. */
#define MII_KSZPHY_CTRL2		0x1f
#define KSZ8031_CTRL2_RMII_50MHZ_CLK	0x0080

static int kszphy_config_init(struct phy_device *phydev)
{
	return 0;
}

static int ksz8031phy_config_init(struct phy_device *phydev)
{
	int val;

	if (phydev->dev_flags & MICREL_PHY_50MHZ_CLK) {
		/* Select 50 MHz RMII clock input mode */
		val = phy_read(phydev, MII_KSZPHY_CTRL2);
		if (val < 0)
			return val;
		phy_write(phydev, MII_KSZPHY_CTRL2, (val | KSZ8031_CTRL2_RMII_50MHZ_CLK));
	}

	/*
	 * Undocumented bit 9 of register 0x16 in PHY Micrel KSZ8031RNL
	 * (documented as 'reserved') needs to be set to work around a PHY bug
	 * that causes the second PHY, with address=3, to also respond to
	 * reads/writes addressed to the first PHY, which has address=0.
	 * The setting of this bit for platforms having only one PHY at
	 * address 0 is harmless.
	 */
	val = phy_read(phydev, MII_KSZPHY_OMSO);
	if (val < 0)
		return val;
	phy_write(phydev, MII_KSZPHY_OMSO, (val | KSZ8031_OMSO_ADDR_FIX));

	return 0;
}

static int ksz8031phy_suspend(struct phy_device *phydev)
{
	int val;

	mutex_lock(&phydev->lock);

	val = phy_read(phydev, MII_BMCR);
	if (val < 0) {
		mutex_unlock(&phydev->lock);
		return 0;
	}
	phy_write(phydev, MII_BMCR, (val | BMCR_PDOWN));

	mutex_unlock(&phydev->lock);

	return 0;
}

static int ksz8031phy_resume(struct phy_device *phydev)
{
	int val;

	mutex_lock(&phydev->lock);

	val = phy_read(phydev, MII_BMCR);
	if (val < 0) {
		mutex_unlock(&phydev->lock);
		return 0;
	}
	phy_write(phydev, MII_BMCR, (val & ~BMCR_PDOWN));

	/*
	 * The 50 MHz RMII clock input mode setting is lost when the PHY is
	 * powered down and then powered back up again, so reapply it here if
	 * required.
	 */
	if (phydev->dev_flags & MICREL_PHY_50MHZ_CLK) {
		val = phy_read(phydev, MII_KSZPHY_CTRL2);
		if (val < 0) {
			mutex_unlock(&phydev->lock);
			return 0;
		}
		phy_write(phydev, MII_KSZPHY_CTRL2, (val | KSZ8031_CTRL2_RMII_50MHZ_CLK));
	}

	mutex_unlock(&phydev->lock);

	return 0;
}

static struct phy_driver ks8001_driver = {
	.phy_id		= PHY_ID_KS8001,
	.name		= "Micrel KS8001",
	.phy_id_mask	= 0x00fffff0,
	.features	= PHY_BASIC_FEATURES,
	.flags		= PHY_POLL,
	.config_init	= kszphy_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.driver		= { .owner = THIS_MODULE,},
};

static struct phy_driver vsc8201_driver = {
	.phy_id		= PHY_ID_VSC8201,
	.name		= "Micrel VSC8201",
	.phy_id_mask	= 0x00fffff0,
	.features	= PHY_BASIC_FEATURES,
	.flags		= PHY_POLL,
	.config_init	= kszphy_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.driver		= { .owner = THIS_MODULE,},
};

static struct phy_driver ksz9021_driver = {
	.phy_id		= PHY_ID_KSZ9021,
	.phy_id_mask	= 0x000fff10,
	.name		= "Micrel KSZ9021 Gigabit PHY",
	.features	= PHY_GBIT_FEATURES | SUPPORTED_Pause,
	.flags		= PHY_POLL,
	.config_init	= kszphy_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.driver		= { .owner = THIS_MODULE, },
};

static struct phy_driver ksz8031_driver = {
	.phy_id		= PHY_ID_KSZ8031,
	.phy_id_mask	= 0xfffffff0,
	.name		= "Micrel KSZ8031RNL",
	.features	= PHY_BASIC_FEATURES,
	.flags		= PHY_POLL,
	.config_init	= ksz8031phy_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.suspend	= ksz8031phy_suspend,
	.resume		= ksz8031phy_resume,
	.driver		= { .owner = THIS_MODULE, },
};

static int __init ksphy_init(void)
{
	int ret;

	ret = phy_driver_register(&ks8001_driver);
	if (ret)
		goto err1;
	ret = phy_driver_register(&vsc8201_driver);
	if (ret)
		goto err2;
	ret = phy_driver_register(&ksz9021_driver);
	if (ret)
		goto err3;
	ret = phy_driver_register(&ksz8031_driver);
	if (ret)
		goto err4;
	return 0;

err4:
	phy_driver_unregister(&ksz9021_driver);
err3:
	phy_driver_unregister(&vsc8201_driver);
err2:
	phy_driver_unregister(&ks8001_driver);
err1:
	return ret;
}

static void __exit ksphy_exit(void)
{
	phy_driver_unregister(&ks8001_driver);
	phy_driver_unregister(&vsc8201_driver);
	phy_driver_unregister(&ksz9021_driver);
	phy_driver_unregister(&ksz8031_driver);
}

module_init(ksphy_init);
module_exit(ksphy_exit);

MODULE_DESCRIPTION("Micrel PHY driver");
MODULE_AUTHOR("David J. Choi");
MODULE_LICENSE("GPL");

static struct mdio_device_id micrel_tbl[] __maybe_unused = {
	{ PHY_ID_KSZ9021, 0x000fff10 },
	{ PHY_ID_VSC8201, 0x00fffff0 },
	{ PHY_ID_KS8001, 0x00fffff0 },
	{ PHY_ID_KSZ8031, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, micrel_tbl);
