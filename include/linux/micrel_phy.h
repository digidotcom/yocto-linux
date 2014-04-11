#ifndef _MICREL_PHY_H
#define _MICREL_PHY_H

#define MICREL_PHY_ID_MASK	0xfffffff0

#define PHY_ID_KSZ9021		0x00221611
#define PHY_ID_VSC8201		0x000FC413
#define PHY_ID_KS8001		0x0022161A
#define PHY_ID_KSZ8021		0x00221555
#define PHY_ID_KSZ8031		0x00221556
#define PHY_ID_KSZ8051		0x00221550
#define PHY_ID_KSZ8081		0x00221560

/* struct phy_device dev_flags definitions */
#define MICREL_PHY_50MHZ_CLK	0x00000001

#endif /* _MICREL_PHY_H */
