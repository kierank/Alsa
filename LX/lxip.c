/*
 *
 * ALSA driver for the digigram lx audio interface
 *
 * Copyright (c) 2016 Jubier Sylvain <alsa@digigram.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

#include <sound/initval.h>
#include <sound/control.h>
#include <sound/info.h>

#include "lxcommon.h"

MODULE_AUTHOR("Sylvain Jubier <alsa@digigram.com> ");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("digigram lxip");
MODULE_SUPPORTED_DEVICE("{digigram lxip{}}");

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for Digigram LX IP interface.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for  Digigram LX IP interface.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable/disable specific Digigram LX IP soundcards.");

#define PCI_DEVICE_ID_PLX_LXIP                PCI_DEVICE_ID_PLX_9056

static DEFINE_PCI_DEVICE_TABLE(snd_lxip_ids) = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_PLX,	PCI_DEVICE_ID_PLX_LXIP),
		.subvendor = PCI_VENDOR_ID_DIGIGRAM,
		.subdevice = PCI_SUBDEVICE_ID_DIGIGRAM_LXIP_SUBSYSTEM
	}, /* LX-IP */
	{
		PCI_DEVICE(PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_LXIP),
		.subvendor = PCI_VENDOR_ID_DIGIGRAM,
		.subdevice =
			PCI_SUBDEVICE_ID_DIGIGRAM_LXIP_MADI_SUBSYSTEM
	}, /* LX-IP-MADI */
	{0, },
};

MODULE_DEVICE_TABLE(pci, snd_lxip_ids);

/* defaults */
#define LXIP_USE_RATE			(SNDRV_PCM_RATE_44100 | \
					SNDRV_PCM_RATE_48000 | \
					SNDRV_PCM_RATE_88200 | \
					SNDRV_PCM_RATE_96000)
#define USE_RATE_MIN                    44100
#define USE_RATE_MAX                    96000
#define LXIP_USE_CHANNELS_MIN           2
#define LXIP_USE_CHANNELS_MAX           64
#define LXIP_USE_PERIODS_MIN            2
#define LXIP_USE_PERIODS_MAX            8	/* theoretical max : infinity
						*                   (your RAM),
						* set to 8 to reduce
						* buffer_bytes_max
*/
#define LXIP_GRANULARITY_MIN            8
#define LXIP_GRANULARITY_MAX            64
#define LXIP_PERIOD_MULTIPLE_GRAN_MIN   1
#define LXIP_PERIOD_MULTIPLE_GRAN_MAX   32	/* theoretical max : 255,
						* set to 32 to reduce
						* buffer_bytes_max
						*/
#define LXIP_SAMPLE_SIZE_MIN            2 /*16bits/sample*/
#define LXIP_SAMPLE_SIZE_MAX            3 /*24bits/sample*/
/* alsa callbacks */
static struct snd_pcm_hardware lx_ip_caps = {
		.info = (SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_SYNC_START),
		.formats =	(SNDRV_PCM_FMTBIT_S24_3LE |
				SNDRV_PCM_FMTBIT_S24_3BE),
		.rates = LXIP_USE_RATE,
		.rate_min = 44100,
		.rate_max = 96000,
		.channels_min = LXIP_USE_CHANNELS_MIN,
		.channels_max = LXIP_USE_CHANNELS_MAX, .buffer_bytes_max =
				LXIP_USE_CHANNELS_MAX *
				LXIP_GRANULARITY_MAX *
				LXIP_PERIOD_MULTIPLE_GRAN_MAX *
				LXIP_SAMPLE_SIZE_MAX *
				LXIP_USE_PERIODS_MAX, .period_bytes_min =
				LXIP_USE_CHANNELS_MIN *
				LXIP_GRANULARITY_MIN *
				LXIP_PERIOD_MULTIPLE_GRAN_MIN *
				LXIP_SAMPLE_SIZE_MIN, .period_bytes_max =
				LXIP_USE_CHANNELS_MAX *
				LXIP_GRANULARITY_MAX *
				LXIP_PERIOD_MULTIPLE_GRAN_MAX *
				LXIP_SAMPLE_SIZE_MAX, .periods_min =
				LXIP_USE_PERIODS_MIN, .periods_max =
				LXIP_USE_PERIODS_MAX, };

struct ravenna_clocks_info {
	unsigned char cm; /* freq event on ravennas clock */
	unsigned int ravenna_freq; /* internal clock frequency */

};

static unsigned int ravenna_freq_conversion[] = {
		8000, 11025, 12000, 160000,
		22050, 24000, 32000, 44100,
		48000, 64000, 88200, 96000,
		128000,	176400, 192000, 0
};
#define IP_RAVENNA_FREQ_MASK         0x00000F0
#define IP_GET_RAVENNA_FREQ(val)     ((val & IP_RAVENNA_FREQ_MASK) >> 4)
#define IP_CM_MASK                    0x0000001
#define IP_GET_CM(val)                ((val & IP_CM_MASK) >> 0)

int lx_ip_get_clocks_status(struct lx_chip *chip,
		struct ravenna_clocks_info *clocks_information)
{
	int err = 0;
	unsigned long clocks_status;

	clocks_status = lx_dsp_reg_read(chip, eReg_MADI_RAVENNA_CLOCK_CFG);
/*	printk(KERN_DEBUG "%s  lxip %lx\n", __func__, clocks_status);*/
	if (clocks_information != NULL) {
		clocks_information->cm = IP_GET_CM(clocks_status);
		clocks_information->ravenna_freq =
				ravenna_freq_conversion[IP_GET_RAVENNA_FREQ(
						clocks_status)];
	}
	return err;
}

static int snd_clock_rate_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *info)
{
	info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	info->count = 1;
	info->value.integer.min = 0; /* clock not present */
	info->value.integer.max = 192000; /* max sample rate 192 kHz */
	return 0;
}

static int snd_clock_rate_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *value)
{
	struct lx_chip *chip = snd_kcontrol_chip(kcontrol);
	struct ravenna_clocks_info clocks_information;

	lx_ip_get_clocks_status(chip, &clocks_information);
	value->value.integer.value[0] = clocks_information.ravenna_freq;
	return 0;
}

/*Clock Mode "Uggly hack" for Digigram Audio Engine.*/
const char * const sync_names[] = {"Internal"};
static int snd_clock_iobox_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *info)
{
	info->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	return snd_ctl_enum_info(info, 1, 1, sync_names);
}

static int snd_clock_iobox_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *value)
{
	struct lx_chip *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = chip->use_clock_sync;
	return 0;
}

static int snd_clock_iobox_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *value)
{
	struct lx_chip *chip = snd_kcontrol_chip(kcontrol);
	int changed;

	if (value->value.enumerated.item[0] > 2)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != chip->use_clock_sync;
	return changed;
}

const char * const lxip_granularity_names[] = {"8", "16", "32", "64", "128",
		"256", "512"};
const int const lxip_granularity_value[] = {8, 16, 32, 64, 128, 256, 512};

static int snd_granularity_iobox_info(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_info *info)
{
	return snd_ctl_enum_info(info, 1, 4, lxip_granularity_names);
}

static int snd_granularity_iobox_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *value)
{
	struct lx_chip *chip = snd_kcontrol_chip(kcontrol);
	int i = 0;

	for (i = 0; i < 7; i++) {
		if (lxip_granularity_value[i] == chip->pcm_granularity) {
			value->value.enumerated.item[0] = i;
			break;
		}
	}
	return 0;
}

static int snd_granularity_iobox_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *value)
{
	struct lx_chip *chip = snd_kcontrol_chip(kcontrol);
	int changed;
	int err;

	if (value->value.enumerated.item[0] > 4)
		return -EINVAL;

	changed = (lxip_granularity_value[value->value.enumerated.item[0]]
			!= chip->pcm_granularity);
	if (changed) {
		/*set_gran if allow*/
/*
*                printk(KERN_DEBUG
*                       "%s set gran to %d\n",
*                       __func__,
*                       chip->pcm_granularity);
*/
		err = lx_set_granularity(chip,
		lxip_granularity_value[value->value.enumerated.item[0]]);
		if (err < 0) {
			dev_err(chip->card->dev,
		"setting granularity to %d failed\n",
		lxip_granularity_value[value->value.enumerated.item[0]]);
			return err;
		}
	}
	return changed;
}

static struct snd_kcontrol_new snd_lxip_controls[] = {
		{
			.access	= SNDRV_CTL_ELEM_ACCESS_READ,
			.iface	= SNDRV_CTL_ELEM_IFACE_CARD,
			.name	= "Clock Rates",
			.info	= snd_clock_rate_info,
			.get	= snd_clock_rate_get,
		},
		{
			.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
			.name	= "Clock Mode",
			.info	= snd_clock_iobox_info,
			.get	= snd_clock_iobox_get,
			.put	= snd_clock_iobox_put,
		},
		{
			.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
			.name	= "DMA Granularity",
			.info	= snd_granularity_iobox_info,
			.get	= snd_granularity_iobox_get,
			.put	= snd_granularity_iobox_put,
			.private_value = 1,
			.index = 0,
		},
};

void lx_ip_proc_get_clocks_status(struct snd_info_entry *entry,
		struct snd_info_buffer *buffer)
{
	struct ravenna_clocks_info clocks_information;
	struct lx_chip *chip = entry->private_data;

	lx_ip_get_clocks_status(chip, &clocks_information);

	snd_iprintf(buffer, "Ravenna freq change :        %s\n"
			"Internal frequency :      %d Hz\n",
			clocks_information.cm ? "True" : "False",
			clocks_information.ravenna_freq);
}

int lx_ip_proc_create(struct snd_card *card, struct lx_chip *chip)
{
	struct snd_info_entry *entry;
	/*clocks*/
	int err = snd_card_proc_new(card, "Clocks", &entry);

	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, snd_card_proc_new clocks\n", __func__);
		return err;
	}

	snd_info_set_text_ops(entry, chip, lx_ip_proc_get_clocks_status);

	return 0;
}

static struct snd_pcm_ops lx_ops_playback = {
		.open = lx_pcm_open,
		.close = lx_pcm_close,
		.ioctl = snd_pcm_lib_ioctl,
		.prepare = lx_pcm_prepare,
		.hw_params = lx_pcm_hw_params,
		.hw_free = lx_pcm_hw_free,
		.trigger = lx_pcm_trigger,
		.pointer = lx_pcm_stream_pointer,
};

static struct snd_pcm_ops lx_ops_capture = {
		.open = lx_pcm_open,
		.close = lx_pcm_close,
		.ioctl = snd_pcm_lib_ioctl,
		.prepare = lx_pcm_prepare,
		.hw_params = lx_pcm_hw_params,
		.hw_free = lx_pcm_hw_free,
		.trigger = lx_pcm_trigger,
		.pointer = lx_pcm_stream_pointer,
};

int lx_ip_pcm_create(struct lx_chip *chip)
{
	int err;
	struct snd_pcm *pcm;
	u32 size;

	size =	LXIP_USE_CHANNELS_MAX * /* channels */
		LXIP_SAMPLE_SIZE_MAX * /* 24 bit samples */
		LXIP_USE_PERIODS_MAX * /* periods */
		LXIP_GRANULARITY_MAX * /* frames per period */
		LXIP_PERIOD_MULTIPLE_GRAN_MAX;/* max period size */

/*        printk(KERN_DEBUG  "%s\n", __func__);*/

	size = PAGE_ALIGN(size);

	/* hardcoded device name & channel count */
	if (chip->lx_type == LX_IP) {
		err = snd_pcm_new(chip->card, (char *)"LX_IP", 0, 1, 1, &pcm);
	} else {
		err = snd_pcm_new(chip->card, (char *)"LX_IP_MADI", 0, 1, 1,
				&pcm);
	}
	if (err < 0)
		return err;

	pcm->private_data = chip;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &lx_ops_playback);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &lx_ops_capture);

	pcm->info_flags = 0;
	if (chip->lx_type == LX_IP)
		strcpy(pcm->name, "LX_IP");
	else
		strcpy(pcm->name, "LX_IP_MADI");

	err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
			snd_dma_pci_data(chip->pci), size, size);
	if (err < 0) {
		dev_err(chip->card->dev,
		"%s, snd_pcm_lib_preallocate_pages_for_all failed", __func__);
		return err;
	}

	chip->pcm = pcm;
	chip->capture_stream.is_capture = 1;

	return 0;
}
static int snd_ip_create(struct snd_card *card, struct pci_dev *pci,
		struct lx_chip **rchip)
{
	struct lx_chip *chip;
	int err;
	unsigned int idx;
	struct snd_kcontrol *kcontrol;
	static struct snd_device_ops ops = {
			.dev_free = snd_lx_dev_free,
	};

/*	printk(KERN_DEBUG "%s\n", __func__);*/

	*rchip = NULL;

	/* enable PCI device */
	err = pci_enable_device(pci);
	if (err < 0)
		return err;

	pci_set_master(pci);

	/* check if we can restrict PCI DMA transfers to 32 bits */
#if KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE
	err = dma_set_mask(&pci->dev, DMA_BIT_MASK(32));
#elif KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
	err = pci_set_dma_mask(pci, DMA_BIT_MASK(32));
#elif KERNEL_VERSION(3, 10, 17) == LINUX_VERSION_CODE
	err = pci_set_dma_mask(pci, DMA_BIT_MASK(32));
#else
#error "kernel not supported"
#endif

	if (err < 0) {
		dev_err(&pci->dev,
	"%s, architecture does not support 32bit PCI busmaster DMA\n",
		__func__);
		pci_disable_device(pci);
		return -ENXIO;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		err = -ENOMEM;
		goto alloc_failed;
	}

	atomic_set(&chip->irq_pending, 0);
	atomic_set(&chip->play_xrun_advertise, 0);
	atomic_set(&chip->capture_xrun_advertise, 0);
	atomic_set(&chip->debug_irq.atomic_irq_handled, 0);

	chip->card = card;
	chip->pci = pci;
	chip->irq = -1;
	if (pci->subsystem_device ==
	PCI_SUBDEVICE_ID_DIGIGRAM_LXIP_SUBSYSTEM) {
		chip->lx_type = LX_IP;
	} else if (pci->subsystem_device ==
	PCI_SUBDEVICE_ID_DIGIGRAM_LXIP_MADI_SUBSYSTEM) {
		chip->lx_type = LX_IP_MADI;
	}

/*	set default internal card conf to local*/
	chip->pcm_hw = lx_ip_caps;

	/* initialize synchronization structs */
	mutex_init(&chip->msg_lock);
	mutex_init(&chip->setup_mutex);
	chip->lx_chip_index = lx_chips_count;

	/* request resources */
	if (chip->lx_type == LX_IP)
		err = pci_request_regions(pci, "LX_IP");
	else
		err = pci_request_regions(pci, "LX_IP_MADI");

	if (err < 0)
		goto request_regions_failed;

	/* plx port */
	chip->port_plx = pci_resource_start(pci, 1);
	chip->port_plx_remapped = pci_iomap(pci, 1, 0);

	/* dsp port */
	chip->port_dsp_bar = pci_ioremap_bar(pci, 2);

	chip->capture_stream.status = LX_STREAM_STATUS_STOPPED;
	chip->playback_stream.status = LX_STREAM_STATUS_STOPPED;

	chip->debug_irq.irq_all = 0;
	chip->debug_irq.irq_wakeup_thread = 0;
	chip->debug_irq.irq_play_begin = 0;
	chip->debug_irq.irq_play = 0;
	chip->debug_irq.irq_play_unhandled = 0;
	chip->debug_irq.irq_record = 0;
	chip->debug_irq.irq_record_unhandled = 0;
	chip->debug_irq.irq_play_and_record = 0;
	chip->debug_irq.irq_none = 0;
	chip->debug_irq.irq_handled = 0;
	atomic_set(&chip->debug_irq.atomic_irq_handled, 0);
	chip->debug_irq.irq_urun = 0;
	chip->debug_irq.irq_orun = 0;
	chip->debug_irq.irq_freq = 0;
	chip->debug_irq.irq_esa = 0;
	chip->debug_irq.irq_timer = 0;
	chip->debug_irq.irq_eot = 0;
	chip->debug_irq.irq_xes = 0;
	chip->debug_irq.wakeup_thread = 0;
	chip->debug_irq.thread_play = 0;
	chip->debug_irq.thread_record_but_stop = 0;
	chip->debug_irq.thread_record = 0;
	chip->debug_irq.thread_play_but_stop = 0;
	chip->debug_irq.thread_play_and_record = 0;
	chip->debug_irq.async_event_eobi = 0;
	chip->debug_irq.async_event_eobo = 0;
	chip->debug_irq.async_urun = 0;
	chip->debug_irq.async_event_eobo = 0;
	chip->debug_irq.cmd_irq_waiting = 0;
	chip->jiffies_start = -1;
	chip->jiffies_1st_irq = -1;

	chip->irq = -1;

	if (chip->lx_type == LX_IP) {
		err = request_threaded_irq(pci->irq, lx_interrupt, NULL,
		IRQF_SHARED, "LX-IP", chip);
	} else {
		err = request_threaded_irq(pci->irq, lx_interrupt, NULL,
		IRQF_SHARED, "LX-IP-MADI", chip);
	}
	if (err) {
		dev_err(&pci->dev,
		"%s, unable to grab IRQ %d\n", __func__, pci->irq);
		goto request_irq_failed;
	}
	chip->irq = pci->irq;

	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0)
		goto device_new_failed;

	err = lx_init_dsp(chip);
	if (err < 0) {
		dev_err(&pci->dev,
			"%s, error during DSP initialization\n",
			__func__);
		goto device_new_failed;
	}

	err = lx_ip_pcm_create(chip);
	if (err < 0) {
		dev_err(&pci->dev,
			"%s,lx_ip_pcm_create failed\n", __func__);
		goto device_new_failed;
	}

	err = lx_proc_create(card, chip);
	if (err < 0) {
		dev_err(&pci->dev,
			"%s,lx_proc_create failed\n", __func__);
		goto device_new_failed;
	}
	err = lx_ip_proc_create(card, chip);
	if (err < 0) {
		dev_err(&pci->dev,
			"%s,lx_proc_create failed\n", __func__);
		goto device_new_failed;
	}

	for (idx = 0; idx < ARRAY_SIZE(snd_lxip_controls); idx++) {
		kcontrol = snd_ctl_new1(&snd_lxip_controls[idx], chip);
		err = snd_ctl_add(card, kcontrol);
		if (err < 0) {
			dev_err(&pci->dev,
				"%s,snd_ctl_add failed\n", __func__);
			goto device_new_failed;
		}
	}
#if KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE
/*	nothing*/
#elif KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
	snd_card_set_dev(card, &pci->dev);
#elif KERNEL_VERSION(3, 10, 17) <= LINUX_VERSION_CODE
	snd_card_set_dev(card, &pci->dev);
#endif
	*rchip = chip;

	err = lx_pipe_open(chip, 0, LXIP_USE_CHANNELS_MAX);
	if (err < 0) {
		dev_err(&pci->dev,
			"setting lx_pipe_open failed\n");
		goto device_new_failed;
	}

	err = lx_pipe_open(chip, 1, LXIP_USE_CHANNELS_MAX);
	if (err < 0) {
		dev_err(&pci->dev,
			"setting lx_pipe_open failed\n");
		goto device_new_failed;
	}

	lx_chips[lx_chips_count++] = chip;

	return 0;

device_new_failed:
	if (chip->irq >= 0)
		free_irq(pci->irq, chip);

request_irq_failed:
	pci_release_regions(pci);

request_regions_failed:
	kfree(chip);

alloc_failed:
	pci_disable_device(pci);

	return err;
}

static int snd_lxip_probe(struct pci_dev *pci,
		const struct pci_device_id *pci_id)
{
	static int dev;
	struct snd_card *card;
	struct lx_chip *chip;
	int err;
/*        printk(KERN_DEBUG  "%s\n", __func__);*/

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

/*4.1.9*/
#if KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE
	err = snd_card_new(&pci->dev, index[dev], id[dev], THIS_MODULE,
			0, &card);
#elif KERNEL_VERSION(3, 19, 0) <= LINUX_VERSION_CODE
	err = snd_card_new(&pci->dev, index[dev], id[dev], THIS_MODULE,
			0, &card);
#elif KERNEL_VERSION(3, 10, 17) == LINUX_VERSION_CODE
	err = snd_card_create(index[dev], id[dev], THIS_MODULE, 0, &card);
#else
#error "kernel not supported"
#endif
	if (err < 0)
		return err;

	err = snd_ip_create(card, pci, &chip);
	if (err < 0) {
		dev_err(&pci->dev,
			"%s, error during snd_lxip_create\n", __func__);
		return -ENOENT;
	}
	card->private_data = chip;

	if (chip->lx_type == LX_IP) {
		strcpy(card->driver, "LX_IP");
		sprintf(card->id, "LX_IP");
		sprintf(card->longname, "%s at 0x%lx, 0x%p, irq %i",
				card->shortname, chip->port_plx,
				chip->port_dsp_bar, chip->irq);
		strcpy(chip->card->shortname, "LX-IP");

	} else if (chip->lx_type == LX_IP_MADI) {
		strcpy(card->driver, "LX_IP_MADI");
		sprintf(card->id, "LX_IP_MADI");
		strcpy(chip->card->shortname, "LX-IP-MADI");

	}

	err = snd_card_register(card);
	if (err < 0)
		goto out_free;

/*	printk(KERN_DEBUG "%s, initialization successful\n", __func__);*/
	pci_set_drvdata(pci, card);

	dev++;
	return 0;

out_free: snd_card_free(card);
	return err;

}

static void snd_lxip_remove(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct lx_chip *chip = card->private_data;
	int is_capture = 0;
	int err = 0;

	for (is_capture = 0; is_capture <= 1; is_capture++) {
		if (chip->hardware_running[is_capture] > 1) {
			err = lx_pipe_stop(chip, is_capture);
			if (err < 0) {
				dev_err(&pci->dev,
				"%s, failed to stop hardware. Error code %d\n",
				__func__, err);
			}

			chip->hardware_running[is_capture] = 1;
		}
		if (chip->hardware_running[is_capture] == 1) {

			err = lx_pipe_close(chip, is_capture);
			if (err < 0) {
				dev_err(&pci->dev,
				"%s failed to close hardware. Error code %d\n",
				__func__, err);
			}
			chip->hardware_running[is_capture] = 0;
		}
	}

	lx_chips_count--;

	snd_card_free(pci_get_drvdata(pci));
#if KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE
/*	nothing*/
#elif KERNEL_VERSION(3, 19, 0) >= LINUX_VERSION_CODE
	pci_set_drvdata(pci, NULL);
#elif KERNEL_VERSION(3, 10, 17) == LINUX_VERSION_CODE
	pci_set_drvdata(pci, NULL);
#endif
}

static struct pci_driver lxip_driver = {
/*        .name =     KBUILD_MODNAME,*/
	.name = "LX_IP_MADI",
	.id_table = snd_lxip_ids,
	.probe = snd_lxip_probe,
	.remove = snd_lxip_remove,
};

module_pci_driver(lxip_driver);

