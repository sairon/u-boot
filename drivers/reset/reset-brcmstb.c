// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom STB generic reset controller for SW_INIT style reset controller
 *
 * Based on upstream Linux driver:
 * drivers/reset/reset-brcmstb.c
 * Author: Florian Fainelli <f.fainelli@gmail.com>
 * Copyright (C) 2018 Broadcom
 */

#include <dm.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <reset-uclass.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#define SW_INIT_SET		0x00
#define SW_INIT_CLEAR		0x04
#define SW_INIT_STATUS		0x08

#define SW_INIT_BIT(id)		BIT((id) & 0x1f)
#define SW_INIT_BANK(id)	((id) >> 5)

/* A full bank contains extra registers that we are not utilizing but still
 * qualify as a single bank.
 */
#define SW_INIT_BANK_SIZE	0x18

struct brcmstb_reset_priv {
	void __iomem *base;
};

static int brcmstb_reset_assert(struct reset_ctl *rst)
{
	unsigned int off = SW_INIT_BANK(rst->id) * SW_INIT_BANK_SIZE;
	struct brcmstb_reset_priv *priv = dev_get_priv(rst->dev);

	writel_relaxed(SW_INIT_BIT(rst->id), priv->base + off + SW_INIT_SET);

	return 0;
}

static int brcmstb_reset_deassert(struct reset_ctl *rst)
{
  unsigned int off = SW_INIT_BANK(rst->id) * SW_INIT_BANK_SIZE;
	struct brcmstb_reset_priv *priv = dev_get_priv(rst->dev);

	writel_relaxed(SW_INIT_BIT(rst->id), priv->base + off + SW_INIT_CLEAR);
	/* Maximum reset delay after de-asserting a line and seeing block
	* operation is typically 14us for the worst case, build some slack
	* here.
	*/
	udelay(200);

  return 0;
}

struct reset_ops brcmstb_reset_reset_ops = {
	.rst_assert = brcmstb_reset_assert,
	.rst_deassert = brcmstb_reset_deassert,
};

static const struct udevice_id brcmstb_reset_ids[] = {
	{ .compatible = "brcm,brcmstb-reset" },
	{ /* sentinel */ }
};

static int brcmstb_reset_probe(struct udevice *dev)
{
	struct brcmstb_reset_priv *priv = dev_get_priv(dev);

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;

	return 0;
}

U_BOOT_DRIVER(brcmstb_reset) = {
	.name = "brcmstb-reset",
	.id = UCLASS_RESET,
	.of_match = brcmstb_reset_ids,
	.ops = &brcmstb_reset_reset_ops,
	.probe = brcmstb_reset_probe,
	.priv_auto	= sizeof(struct brcmstb_reset_priv),
};
