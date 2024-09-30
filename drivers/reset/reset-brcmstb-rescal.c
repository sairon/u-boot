// SPDX-License-Identifier: GPL-2.0
/*
 * Broadcom RESCAL reset controller, based on upstream Linux driver:
 * drivers/reset/reset-brcmstb-rescal.c
 *
 * Copyright (C) 2018-2020 Broadcom
 */

#include <dm.h>
#include <errno.h>
#include <malloc.h>
#include <reset-uclass.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#define BRCM_RESCAL_START	0x0
#define  BRCM_RESCAL_START_BIT	BIT(0)
#define BRCM_RESCAL_CTRL	0x4
#define BRCM_RESCAL_STATUS	0x8
#define  BRCM_RESCAL_STATUS_BIT	BIT(0)

struct brcm_rescal_reset_priv {
	void __iomem *base;
};

static int brcm_rescal_reset_assert(struct reset_ctl *rst)
{
	return 0;
}

static int brcm_rescal_reset_deassert(struct reset_ctl *rst)
{
	struct brcm_rescal_reset_priv *priv = dev_get_priv(rst->dev);
	u32 reg;
	int ret;

	reg = readl(priv->base + BRCM_RESCAL_START);
	writel(reg | BRCM_RESCAL_START_BIT, priv->base + BRCM_RESCAL_START);
	reg = readl(priv->base + BRCM_RESCAL_START);
	if (!(reg & BRCM_RESCAL_START_BIT)) {
		printf("failed to start SATA/PCIe rescal\n");
		return -EIO;
	}

	ret = read_poll_timeout(readl, reg, (reg & BRCM_RESCAL_STATUS_BIT),
				100, 1000, priv->base + BRCM_RESCAL_STATUS);
	if (ret) {
		printf("time out on SATA/PCIe rescal\n");
		return ret;
	}

	reg = readl(priv->base + BRCM_RESCAL_START);
	writel(reg & ~BRCM_RESCAL_START_BIT, priv->base + BRCM_RESCAL_START);

	return ret;
}

static int brcm_rescal_reset_of_xlate(struct reset_ctl *reset_ctl,
				      struct ofnode_phandle_args *args)
{
	/* Rescal takes no parameters. */
	if (args->args_count != 0) {
		printf("Invalid args_count: %d\n", args->args_count);
		return -EINVAL;
	}

	return 0;
}

struct reset_ops brcm_rescal_reset_reset_ops = {
	.rst_assert = brcm_rescal_reset_assert,
	.rst_deassert = brcm_rescal_reset_deassert,
	.of_xlate = brcm_rescal_reset_of_xlate,
};

static const struct udevice_id brcm_rescal_reset_ids[] = {
	{ .compatible = "brcm,bcm7216-pcie-sata-rescal" },
	{ /* sentinel */ }
};

static int brcm_rescal_reset_probe(struct udevice *dev)
{
	struct brcm_rescal_reset_priv *priv = dev_get_priv(dev);

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;

	return 0;
}

U_BOOT_DRIVER(brcmstb_rescal_reset) = {
	.name = "brcm-rescal-reset",
	.id = UCLASS_RESET,
	.of_match = brcm_rescal_reset_ids,
	.ops = &brcm_rescal_reset_reset_ops,
	.probe = brcm_rescal_reset_probe,
	.priv_auto	= sizeof(struct brcm_rescal_reset_priv),
};
