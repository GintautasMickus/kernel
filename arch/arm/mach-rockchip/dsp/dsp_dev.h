/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 * author: ZhiChao Yu zhichao.yu@rock-chips.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _ARCH_ARM_MACH_RK_DSP_DEV_H_
#define _ARCH_ARM_MACH_RK_DSP_DEV_H_

#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/rockchip/dvfs.h>
#if defined(CONFIG_ION_ROCKCHIP)
#include <linux/rockchip_ion.h>
#endif

#include "dsp_loader.h"
#include "dsp_dma.h"
#include "dsp_mbox.h"
#include "dsp_work.h"

enum dsp_clk_status {
	DSP_CLK_AUTO   = 0,
	DSP_CLK_USER   = 1,
};

enum dsp_status {
	DSP_OFF      = 0,
	DSP_ON       = 1,
	DSP_SLEEP    = 2,
};

struct dsp_heap {
	u32 size;
	u32 phys;
	void *virt;
	struct ion_handle *ion_hdl;
};

struct dsp_dev_client {
	void *data;

	int (*device_ready)(struct dsp_dev_client *);
	int (*device_pause)(struct dsp_dev_client *);
	int (*work_done)(struct dsp_dev_client *, struct dsp_work *);
};

struct dsp_dev {
	struct device *device;
	enum dsp_status status;

	int (*on)(struct dsp_dev *);
	int (*off)(struct dsp_dev *);
	int (*suspend)(struct dsp_dev *);
	int (*resume)(struct dsp_dev *);
	int (*work)(struct dsp_dev *, struct dsp_work *);

	struct dsp_dma *dma;
	struct dsp_mbox *mbox;
	struct dsp_loader *loader;
	struct dma_pool *dma_pool;
	struct dsp_dev_client *client;
	struct dsp_mbox_client mbox_client;
	struct ion_client *ion_client;
	struct dsp_work *running_work;
	struct dsp_heap heap;

	struct delayed_work guard_work;
	struct delayed_work idle_work;
	u32 dsp_timeout;
	u32 idle_timeout;

	char *trace_buffer;
	u32 trace_dma;
	u32 trace_index;

	struct dvfs_node *dsp_dvfs_node;
	enum dsp_clk_status clk_status;

	struct clk *clk_dsp;
	struct clk *clk_dsp_free;
	struct clk *clk_iop;
	struct clk *clk_epp;
	struct clk *clk_edp;
	struct clk *clk_edap;

	struct reset_control *core_rst;
	struct reset_control *sys_rst;
	struct reset_control *global_rst;
	struct reset_control *oecm_rst;

	void __iomem *dsp_grf;
	void __iomem *dsp_axi;
	void __iomem *mbox_base;

	/* Lock DSP device */
	struct mutex lock;
};

unsigned long dsp_dev_get_freq(struct dsp_dev *dev);
int dsp_dev_set_freq(struct dsp_dev *dev, unsigned long dsp_rate);

int dsp_dev_register_client(struct dsp_dev *dev,
			    struct dsp_dev_client *client);

int dsp_dev_create(struct platform_device *pdev, struct dma_pool *dma_pool,
		   struct dsp_dev **dev);

int dsp_dev_destroy(struct dsp_dev *dev);

#endif

