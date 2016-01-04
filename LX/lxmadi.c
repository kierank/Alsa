/* -*- linux-c -*- *
 *
 * ALSA driver for the digigram lx6464es interface
 *
 * Copyright (c) 2008, 2009 Tim Blechmann <tim@klingt.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
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


MODULE_SUPPORTED_DEVICE("{digigram lxmadi{}}");

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;

module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for Digigram LXMadi interface.");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for  Digigram LXMadi interface.");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable/disable specific Digigram LXMadi soundcards.");

//TODOstatic const char card_name[] = "LXMadi";

#define PCI_DEVICE_ID_PLX_LXMADI		PCI_DEVICE_ID_PLX_9056

static DEFINE_PCI_DEVICE_TABLE(snd_lxmadi_ids) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_PLX, PCI_DEVICE_ID_PLX_LXMADI),
	  .subvendor = PCI_VENDOR_ID_DIGIGRAM,
	  .subdevice = PCIEX_SUBDEVICE_ID_DIGIGRAM_LXMADI_SERIAL_SUBSYSTEM
	},	/* LXMADI */
	{ 0, },
};

MODULE_DEVICE_TABLE(pci, snd_lxmadi_ids);


/* defaults */
#define MADI_USE_RATE			(SNDRV_PCM_RATE_44100 | \
								SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
								SNDRV_PCM_RATE_96000)
#define USE_RATE_MIN			44100
#define USE_RATE_MAX			96000
#define MADI_USE_CHANNELS_MIN	2
#define MADI_USE_CHANNELS_MAX	64
#define MADI_USE_PERIODS_MIN	2
#define MADI_USE_PERIODS_MAX	128
#define MADI_GRANULARITY_MIN	8
#define MADI_GRANULARITY_MAX	64
#define MADI_PERIOD_MULTIPLE_GRAN_MIN	1
#define MADI_PERIOD_MULTIPLE_GRAN_MAX	255
#define MADI_SAMPLE_SIZE_MIN	2 //16bits/sample
#define MADI_SAMPLE_SIZE_MAX	3 //24bits/sample
/* alsa callbacks */
static struct snd_pcm_hardware lx_madi_caps = {
	.info             = (SNDRV_PCM_INFO_MMAP |
			             SNDRV_PCM_INFO_INTERLEAVED |
			             SNDRV_PCM_INFO_MMAP_VALID |
			             SNDRV_PCM_INFO_SYNC_START),
	.formats	      = (SNDRV_PCM_FMTBIT_S24_3LE |
			             SNDRV_PCM_FMTBIT_S24_3BE),
	.rates            = MADI_USE_RATE,
	.rate_min         = 44100,
	.rate_max         = 96000,
	.channels_min     = MADI_USE_CHANNELS_MIN,
	.channels_max     = MADI_USE_CHANNELS_MAX,
	.buffer_bytes_max = MADI_USE_CHANNELS_MAX*MADI_GRANULARITY_MAX*MADI_PERIOD_MULTIPLE_GRAN_MAX*MADI_SAMPLE_SIZE_MAX*MADI_USE_PERIODS_MAX, //channels_max*period_bytes_max
	.period_bytes_min = MADI_USE_CHANNELS_MIN*MADI_GRANULARITY_MIN*MADI_PERIOD_MULTIPLE_GRAN_MIN*MADI_SAMPLE_SIZE_MIN,
	.period_bytes_max = MADI_USE_CHANNELS_MAX*MADI_GRANULARITY_MAX*MADI_PERIOD_MULTIPLE_GRAN_MAX*MADI_SAMPLE_SIZE_MAX,
	.periods_min      = MADI_USE_PERIODS_MIN,
	.periods_max      = MADI_USE_PERIODS_MAX,
};


struct clocks_info {
	unsigned int  madi_freq;
	unsigned int  word_clock_freq;
	unsigned char diviseur;
	unsigned char wo;              //sens du word clock in/out
	unsigned char cm;              //changement de freq sur la clock madi
	unsigned char cw;              //changement de freq sur la word clock
	unsigned int internal_freq;    //valeur de la clock interne
	unsigned char clock_sync;      //source de synchro madi/word clock, interne

};

static unsigned int internal_freq_conversion[] = {44100, 48000, 88200, 96000};
static unsigned int external_freq_conversion[] = {8000, 11025, 12000, 160000,
                                                  22050, 24000, 32000, 44100,
                                                  48000, 64000, 88200, 96000,
                                                  128000, 176400, 192000, 0};


#define MADI_CLOCK_SYNC_MASK 0x0000003
#define MADI_GET_CLOCK_SYNC(val) (val & MADI_CLOCK_SYNC_MASK)
#define MADI_INTERNAL_FREQ_MASK 0x000000C
#define MADI_GET_INTERNAL_FREQ(val) ((val & MADI_INTERNAL_FREQ_MASK)>>2)
#define MADI_CW_MASK 0x0000010
#define MADI_GET_CW(val)	((val & MADI_CW_MASK)>> 4)
#define MADI_CM_MASK 0x0000020
#define MADI_GET_CM(val)	((val & MADI_CM_MASK)>> 5)
#define MADI_WO_MASK 0x0000040
#define MADI_GET_WO(val)	((val & MADI_WO_MASK)>> 6)
#define MADI_DIVISEUR_MASK 0x0000080
#define MADI_GET_DIVISEUR(val)	((val & MADI_DIVISEUR_MASK)>> 7)
#define MADI_EXT_WORK_CLOCK_FREQ_MASK 0x0000F00
#define MADI_GET_EXT_WORK_CLOCK_FREQ(val)	((val & MADI_EXT_WORK_CLOCK_FREQ_MASK)>> 8)
#define MADI_EXT_MADI_FREQ_MASK 0x000F000
#define MADI_GET_EXT_MADI_FREQ(val)	((val & MADI_EXT_MADI_FREQ_MASK)>> 12)


int lx_madi_get_clocks_status(struct lx6464es *chip, struct clocks_info *clocks_informations)
{
	int err = 0;
	unsigned long clocks_status;

	clocks_status = lx_dsp_reg_read(chip, eReg_MADI_RAVENNA_CLOCK_CFG);
//	printk(KERN_DEBUG "%s  %lx\n", __func__, clocks_status);
	if( clocks_informations != NULL){
		clocks_informations->madi_freq = external_freq_conversion[MADI_GET_EXT_MADI_FREQ(clocks_status)];
		clocks_informations->word_clock_freq = external_freq_conversion[MADI_GET_EXT_WORK_CLOCK_FREQ(clocks_status)];
		clocks_informations->diviseur = MADI_GET_DIVISEUR(clocks_status);
		clocks_informations->wo = MADI_GET_WO(clocks_status);
		clocks_informations->cm = MADI_GET_CM(clocks_status);
		clocks_informations->cw = MADI_GET_CW(clocks_status);
		clocks_informations->internal_freq = internal_freq_conversion[MADI_GET_INTERNAL_FREQ(clocks_status)];
		clocks_informations->clock_sync = (clocks_status & MADI_CLOCK_SYNC_MASK);
	}
	return err;
}


int lx_madi_set_clock_diviseur(struct lx6464es *chip, unsigned char clock_diviseur)
{
	int err = 0;
	unsigned long clock_status;

	clock_status = lx_dsp_reg_read(chip, eReg_MADI_RAVENNA_CLOCK_CFG);
	clock_status &= ~(MADI_DIVISEUR_MASK);
	clock_status |= (((0x01)&clock_diviseur)<<7);
	lx_dsp_reg_write(chip, eReg_MADI_RAVENNA_CLOCK_CFG, clock_status);

	return err;
}

int lx_madi_set_clock_frequency(struct lx6464es *chip, int clock_frequency)
{
	int err = 0;
	unsigned i = 0;
	struct clocks_info clocks_informations;
	unsigned char fpga_freq = 0;
	unsigned long clock_status;
	for(i=0; i<4; i++){
		if(internal_freq_conversion[i] == clock_frequency)
			fpga_freq = i;
	}

	clock_status = lx_dsp_reg_read(chip, eReg_MADI_RAVENNA_CLOCK_CFG);

//SJR verrue	clock_status &= ~(MADI_DIVISEUR_MASK);

	clock_status &= ~(MADI_INTERNAL_FREQ_MASK);
	clock_status |= (fpga_freq<<2);
	lx_dsp_reg_write(chip, eReg_MADI_RAVENNA_CLOCK_CFG, clock_status);

	if(chip == lx_chips_master){
		if(lx_chips_slave != NULL){
			//should be updated in 50ms
			for(i = 0; i< 10; i++){
				lx_madi_get_clocks_status(lx_chips_slave, &clocks_informations);
				if(clocks_informations.word_clock_freq == clock_frequency){
//					printk(KERN_DEBUG "%s, slave had been updated in %d loop\n", __func__, i);
					break;
				}else{
					mdelay(10);
				}
			}
			if(clocks_informations.word_clock_freq != clock_frequency){
				printk(KERN_ERR "%s, be careful Master and Slave looks not synchronize by wordclock\n", __func__);
			}
		}
	}

	return err;
}

struct madi_status {
	unsigned char mute;
	unsigned char channel_mode;
	unsigned char tx_frame_mode;
	unsigned char rx_frame_mode;
	unsigned char carrier_error;
	unsigned char lock_error;
	unsigned char async_error;
	unsigned char madi_freq;	//je ne sais pas ce qu apporte cette mesure de frequence
};

#define MADI_MUTE_MASK 0x0000001
#define MADI_GET_MUTE(val) (val & MADI_MUTE_MASK)
#define MADI_CHANNEL_MODE_MASK 0x0000002
#define MADI_GET_CHANNEL_MODE(val) ((val & MADI_CHANNEL_MODE_MASK) >> 1)
#define MADI_TX_FRAME_MODE_MASK 0x0000004
#define MADI_GET_TX_FRAME_MODE(val) ((val & MADI_TX_FRAME_MODE_MASK) >> 2)
#define MADI_RX_FRAME_MODE_MASK 0x0000008
#define MADI_GET_RX_FRAME_MODE(val) ((val & MADI_RX_FRAME_MODE_MASK) >> 3)

#define MADI_CARRIER_ERROR_MASK 0x0000001
#define MADI_GET_CARRIER_ERROR(val) (val & MADI_CARRIER_ERROR_MASK)
#define MADI_LOCK_ERROR_MASK 0x0000002
#define MADI_GET_LOCK_ERROR(val) ((val & MADI_LOCK_ERROR_MASK)>>1)
#define MADI_ASYNC_ERROR_MASK 0x0000004
#define MADI_GET_ASYNC_ERROR(val) ((val & MADI_ASYNC_ERROR_MASK)>>2)
#define MADI_MADI_FREQ_MASK 0x0000030
#define MADI_GET_MADI_FREQ(val) ((val & MADI_MADI_FREQ_MASK)>>4)


int lx_madi_get_madi_state(struct lx6464es *chip, struct madi_status *status)
{
	int err;
//	unsigned long flags;

	mutex_lock(&chip->msg_lock);
	lx_message_init(&chip->rmh, CMD_14_GET_MADI_STATE);

	err = lx_message_send_atomic(chip, &chip->rmh);
	if(err < 0){
		printk(KERN_ERR  "%s->lx_message_send_atomic failed...\n", __func__);
	}

//	printk(KERN_DEBUG  "%s %x    %x\n", __func__, chip->rmh.stat[0], chip->rmh.stat[1]);

	if(status != NULL){

		status->mute          = MADI_GET_MUTE(chip->rmh.stat[0]);
        status->channel_mode  = MADI_GET_CHANNEL_MODE(chip->rmh.stat[0]);
        status->tx_frame_mode = MADI_GET_TX_FRAME_MODE(chip->rmh.stat[0]);
        status->rx_frame_mode = MADI_GET_RX_FRAME_MODE(chip->rmh.stat[0]);

        status->carrier_error = MADI_GET_CARRIER_ERROR(chip->rmh.stat[1]);
        status->lock_error    = MADI_GET_LOCK_ERROR(chip->rmh.stat[1]);
        status->async_error   = MADI_GET_ASYNC_ERROR(chip->rmh.stat[1]);
        status->madi_freq     = MADI_GET_MADI_FREQ(chip->rmh.stat[1]);
	}

	mutex_unlock(&chip->msg_lock);
	return err;
}



int lx_madi_set_clock_sync(struct lx6464es *chip, int clock_sync)
{
	int err = 0;
	unsigned long clock_status;

	clock_status = lx_dsp_reg_read(chip, eReg_MADI_RAVENNA_CLOCK_CFG);

	clock_status &= ~(0x0000003);
	clock_status |= (clock_sync);
	lx_dsp_reg_write(chip, eReg_MADI_RAVENNA_CLOCK_CFG, clock_status);

	return err;
}

int lx_madi_set_word_clock_direction(struct lx6464es *chip, unsigned char clock_dir)
{
	int err = 0;
	unsigned long clock_status;

	clock_status = lx_dsp_reg_read(chip, eReg_MADI_RAVENNA_CLOCK_CFG);

	clock_status &= ~(0x0000040);
	clock_status |= (clock_dir<<6);
	lx_dsp_reg_write(chip, eReg_MADI_RAVENNA_CLOCK_CFG, clock_status);

	return err;
}
//Mixer 		 -> "Internal", "Madi In", "Word Clock In"
//internal value -> "Madi In", "Word Clock In","Internal"
const char *const sync_names[] = {  "Internal", "Madi In", "Word Clock In"};
const char const mixer_to_control[] = {2,0,1};
const char const control_to_mixer[] = {1,2,0};
static int snd_clock_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{
	info->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;

	return snd_ctl_enum_info(info, 1, 3, sync_names);
}

static int snd_clock_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = control_to_mixer[chip->use_clock_sync];

	return 0;
}

static int snd_clock_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int changed;

	if (value->value.enumerated.item[0] > 2)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != control_to_mixer[chip->use_clock_sync];
	if (changed) {
		chip->use_clock_sync = mixer_to_control[value->value.enumerated.item[0]];
		lx_madi_set_clock_sync(chip, chip->use_clock_sync);
//		printk(KERN_DEBUG "\t\t\t%s %d\n", __func__, chip->use_clock_sync);

//		lx_madi_set_clock_frequency(chip, 48000);
//		lx_madi_get_clocks_status(chip, NULL);
	}

	return changed;
}

static int snd_clock_rate_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{
	info->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	info->count = 3;
	info->value.integer.min = 0;		/* clock not present */
	info->value.integer.max = 192000;	/* max sample rate 192 kHz */
	return 0;
}

static int snd_clock_rate_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	struct clocks_info clocks_informations;
	lx_madi_get_clocks_status(chip, &clocks_informations);
	//"Internal"
	value->value.integer.value[0] = clocks_informations.internal_freq;
	//"Madi In",
	value->value.integer.value[1] = clocks_informations.madi_freq;
	// "Word Clock In"
	value->value.integer.value[2] = clocks_informations.word_clock_freq;

	return 0;
}

const char *const word_clock_names[] = {  "In", "Out" };

static int snd_word_clock_direction_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{

	return snd_ctl_enum_info(info, 1, 2, word_clock_names);
}

static int snd_word_clock_direction_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = chip->word_clock_out;

	return 0;
}

static int snd_word_clock_direction_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct clocks_info clocks_informations;
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int changed;

	if (value->value.enumerated.item[0] > 2)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != chip->word_clock_out;
	if (changed) {
		//test before if a worldclock already present, it could damaged cards
		if(chip->word_clock_out == LXMADI_WORD_CLOCK_IN){

			lx_madi_get_clocks_status(chip, &clocks_informations);
			if(clocks_informations.word_clock_freq != 0){
				printk(KERN_WARNING "%s, won t set wordclock out. A wordclock is already present\n", __func__);
				return -EACCES;
			}
		}
		chip->word_clock_out = value->value.enumerated.item[0];
		lx_madi_set_word_clock_direction(chip, chip->word_clock_out);
	}

	return changed;
}


const char *const madi_internal_clock_frequency_names[] = {  "44100", "48000", "88200", "96000" };
const int const madi_internal_clock_frequency_value[] = {  44100, 48000, 88200, 96000 };

static int snd_internal_clock_frequency_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{

	return snd_ctl_enum_info(info, 1, 4, madi_internal_clock_frequency_names);
}

static int snd_internal_clock_frequency_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = chip->madi_frequency_selector;

	return 0;
}

static int snd_internal_clock_frequency_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int changed;

	if (value->value.enumerated.item[0] > 4)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != chip->madi_frequency_selector;
	if (changed) {
		chip->madi_frequency_selector = value->value.enumerated.item[0];
//		lx_madi_set_word_clock_direction(chip, chip->word_clock_out);
		lx_madi_set_clock_frequency(chip, madi_internal_clock_frequency_value[chip->madi_frequency_selector]);
	}

	return changed;
}
#ifdef FULL_MADI_MODE
const char *const madi_rx_tx_mode_names[] = {  "SMUX", "LEGACY"};
const int const madi_rx_tx_mode_value[] = {  0, 1 };

static int snd_rx_tx_mode_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{

	return snd_ctl_enum_info(info, 1, 2, madi_rx_tx_mode_names);
}

static int snd_rx_tx_mode_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = chip->rx_tx_mode;

	return 0;
}

static int snd_rx_tx_mode_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int changed;

	if (value->value.enumerated.item[0] > 2)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != chip->rx_tx_mode;
	if (changed) {
		chip->rx_tx_mode = value->value.enumerated.item[0];
//		chip->diviseur_mode = value->value.enumerated.item[0];
		lx_madi_set_madi_state(chip);
		lx_madi_set_clock_diviseur(chip, (unsigned char)chip->diviseur_mode);

	}

	return changed;
}
const char *const madi_diviseur_mode_names[] = {  "256", "512"};
const int const madi_diviseur_mode_value[] = {  0, 1 };

static int snd_diviseur_mode_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{

	return snd_ctl_enum_info(info, 1, 2, madi_diviseur_mode_names);
}

static int snd_diviseur_mode_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = chip->diviseur_mode;

	return 0;
}
static int snd_diviseur_mode_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int changed;

	if (value->value.enumerated.item[0] > 2)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != chip->diviseur_mode;
	if (changed) {
		chip->diviseur_mode = value->value.enumerated.item[0];
		//lx_madi_set_clock_frequency(chip, madi_internal_clock_frequency_value[chip->madi_frequency_selector]);
		//DO something
		lx_madi_set_clock_diviseur(chip, (unsigned char)chip->diviseur_mode);
	}

	return changed;
}
#endif

const char *const madi_channel_mode_names[] = {  "56/24", "64/32"};
const int const madi_channel_mode_value[] = {  0, 1 };

static int snd_channel_mode_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{

	return snd_ctl_enum_info(info, 1, 2, madi_channel_mode_names);
}

static int snd_channel_mode_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = chip->channel_mode;

	return 0;
}

static int snd_channel_mode_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int changed;

	if (value->value.enumerated.item[0] > 2)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != chip->channel_mode;
	if (changed) {
		chip->channel_mode = value->value.enumerated.item[0];
		lx_madi_set_madi_state(chip);
	}

	return changed;
}



const char *const madi_sync_mode_names[] = {  "Independent", "Master", "Slave"};
const int const madi_sync_mode_value[] = {  0, 1, 2 };

static int snd_sync_mode_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{

	return snd_ctl_enum_info(info, 1, 3, madi_sync_mode_names);
}

static int snd_sync_mode_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);

	value->value.enumerated.item[0] = chip->multi_card_sync_mode;

	return 0;
}

static int snd_sync_mode_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	struct snd_ctl_elem_value valueToChange;
	int changed;
	int err = 0;

	if (value->value.enumerated.item[0] > 2)
		return -EINVAL;

	changed = value->value.enumerated.item[0] != chip->multi_card_sync_mode;
	if (changed) {
		switch (value->value.enumerated.item[0]){
			case LXMADI_SYNC_INDEPENDENT :
				//Nothing special
				//Activer les controles pour sens du wordclock
				chip->mixer_wordclock_out_ctl->vd[0].access &= ~SNDRV_CTL_ELEM_ACCESS_INACTIVE;
				break;

			case LXMADI_SYNC_MASTER :
				//Desactiver les controles pour sens du wordclock
				//wordclock out si possible sinon blam
				valueToChange.value.enumerated.item[0] = LXMADI_WORD_CLOCK_OUT;
				err = chip->mixer_wordclock_out_ctl->put(chip->mixer_wordclock_out_ctl, &valueToChange);
				if(err == -EACCES){
					return err;
				}
				chip->mixer_wordclock_out_ctl->vd[0].access |= SNDRV_CTL_ELEM_ACCESS_INACTIVE;
				lx_chips_master = chip;

				break;

			case LXMADI_SYNC_SLAVE :
				//Desactiver les controles pour sens du wordclock
				//wordclock in
				valueToChange.value.enumerated.item[0] = LXMADI_WORD_CLOCK_IN;
				chip->mixer_wordclock_out_ctl->put(chip->mixer_wordclock_out_ctl, &valueToChange);
				chip->mixer_wordclock_out_ctl->vd[0].access |= SNDRV_CTL_ELEM_ACCESS_INACTIVE;
				valueToChange.value.enumerated.item[0] = control_to_mixer[LXMADI_CLOCK_SYNC_WORDCLOCK];
				chip->mixer_current_clock_ctl->put(chip->mixer_wordclock_out_ctl, &valueToChange);
				lx_chips_slave = chip;
				break;
		}
		chip->multi_card_sync_mode = value->value.enumerated.item[0];
		snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE|SNDRV_CTL_EVENT_MASK_INFO, &chip->mixer_wordclock_out_ctl->id);

	}

	return changed;
}



const char *const madi_granularity_names[] = {  "8", "16", "32", "64", "128", "256", "512"};
const int const madi_granularity_value[] = {  8, 16, 32, 64, 128, 256, 512 };

static int snd_granularity_iobox_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *info)
{

	return snd_ctl_enum_info(info, 1, 4, madi_granularity_names);
}

static int snd_granularity_iobox_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int i =0;

	for(i=0; i<7; i++ ){
		if(madi_granularity_value[i]==chip->pcm_granularity){
			value->value.enumerated.item[0] = i;
			break;
		}
	}
//	value->value.enumerated.item[0] = chip->channel_mode;

	return 0;
}

static int snd_granularity_iobox_put(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *value)
{
	struct lx6464es *chip = snd_kcontrol_chip(kcontrol);
	int changed;
	int err;
	if (value->value.enumerated.item[0] > 4)
		return -EINVAL;

	changed = madi_granularity_value[value->value.enumerated.item[0]] != chip->pcm_granularity;
	if (changed) {
//		chip->pcm_granularity = madi_granularity_value[value->value.enumerated.item[0]];
		//set_gran if allow
		printk(KERN_DEBUG "%s set gran to %d\n", __func__, chip->pcm_granularity);
		err = lx_set_granularity(chip, madi_granularity_value[value->value.enumerated.item[0]]);
		if (err < 0) {
	//		dev_err(chip->card->dev, "setting granularity to %ld failed\n",
	//			   period_size);
			printk(KERN_ERR "setting granularity to %d failed\n",
					madi_granularity_value[value->value.enumerated.item[0]]);
			return err;
		}
	}

	return changed;
}

static struct snd_kcontrol_new snd_lxmadi_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Clock Mode",
		.info  = snd_clock_iobox_info,
		.get   = snd_clock_iobox_get,
		.put   = snd_clock_iobox_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Work clock direction",
		.info  = snd_word_clock_direction_iobox_info,
		.get   = snd_word_clock_direction_iobox_get,
		.put   = snd_word_clock_direction_iobox_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Internal Clock Frequency",
		.info  = snd_internal_clock_frequency_iobox_info,
		.get   = snd_internal_clock_frequency_iobox_get,
		.put   = snd_internal_clock_frequency_iobox_put,
	},
// will be accessible (without bug) in next xilinx fw
#ifdef FULL_MADI_MODE
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Rx Tx Mode",
		.info  = snd_rx_tx_mode_iobox_info,
		.get   = snd_rx_tx_mode_iobox_get,
		.put   = snd_rx_tx_mode_iobox_put,
		.private_value = 2	,
		.index = 0,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Xilinx diviseur",
		.info  = snd_diviseur_mode_iobox_info,
		.get   = snd_diviseur_mode_iobox_get,
		.put   = snd_diviseur_mode_iobox_put,
		.private_value = 1,
		.index = 0,
	},
#endif
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Channel Mode",
		.info  = snd_channel_mode_iobox_info,
		.get   = snd_channel_mode_iobox_get,
		.put   = snd_channel_mode_iobox_put,
		.private_value = 1,
		.index = 0,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "Clock Sync",
		.info  = snd_sync_mode_iobox_info,
		.get   = snd_sync_mode_iobox_get,
		.put   = snd_sync_mode_iobox_put,
		.private_value = 1,
		.index = 0,
	},

	{
		.access =	SNDRV_CTL_ELEM_ACCESS_READ,
		.iface =	SNDRV_CTL_ELEM_IFACE_CARD,
		.name =		"Clock Rates",
		.info =		snd_clock_rate_info,
		.get =		snd_clock_rate_get,
	},


	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name  = "DMA Granularity",
		.info  = snd_granularity_iobox_info,
		.get   = snd_granularity_iobox_get,
		.put   = snd_granularity_iobox_put,
		.private_value = 1,
		.index = 0,
	},

};


void lx_madi_proc_get_clocks_status(struct snd_info_entry *entry,
				struct snd_info_buffer *buffer)
{
	struct clocks_info clocks_informations;
	struct lx6464es *chip = entry->private_data;

	lx_madi_get_clocks_status(chip, &clocks_informations);

	snd_iprintf(buffer, "Madi In freq :            %d Hz \n" \
						"Word Clock In freq :      %d Hz\n" \
						"Clock In diviseur :       %s\n" \
						"Word Clock direction :    %s\n" \
						"Madi freq change :        %s\n" \
						"Word Clock freq change :  %s\n" \
						"Internal frequency :      %d Hz\n" \
						"Sync :                    %s\n",
						clocks_informations.madi_freq,
						clocks_informations.word_clock_freq,
						clocks_informations.diviseur?"512":"256",
						word_clock_names[clocks_informations.wo],
						clocks_informations.cm?"True":"False",
						clocks_informations.cw?"True":"False",
						clocks_informations.internal_freq,
						sync_names[control_to_mixer[clocks_informations.clock_sync]]);
}

void lx_madi_proc_get_madi_status(struct snd_info_entry *entry,
                                  struct snd_info_buffer *buffer)
{
	struct madi_status status;
	struct lx6464es *chip = entry->private_data;

	lx_madi_get_madi_state(chip, &status);

	snd_iprintf(buffer, "Mute : \t%s\n" \
						"channel_mode :\t%d\n" \
						"tx_frame_mode :\t%d\n" \
						"rx_frame_mode :\t%d\n" \
						"carrier_error :\t%d\n" \
						"lock_error :\t%d\n" \
						"async_error :\t%d\n" \
						"madi_freq :\t0x%x\n",
						status.mute?"On":"Off",
						status.channel_mode?64:56,
						status.tx_frame_mode,
						status.rx_frame_mode,
						status.carrier_error,
						status.lock_error,
						status.async_error,
						status.rx_frame_mode);
}


int lx_madi_proc_create(struct snd_card *card, struct lx6464es *chip)
{
	struct snd_info_entry *entry;
	//clocks
	int err = snd_card_proc_new(card, "Clocks", &entry);
	if (err < 0){
		printk(KERN_ERR "%s, snd_card_proc_new clocks\n", __func__);
		return err;
	}

	snd_info_set_text_ops(entry, chip, lx_madi_proc_get_clocks_status);

	//madi state
	err = snd_card_proc_new(card, "States", &entry);

	if (err < 0){
		printk(KERN_ERR "%s, snd_card_proc_new States\n", __func__);
		return err;
	}

	snd_info_set_text_ops(entry, chip, lx_madi_proc_get_madi_status);

	return 0;
}


int lx_madi_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct lx6464es *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct clocks_info clocks_informations;
	int err = 0;
	int i = 0;
//	printk(KERN_DEBUG  "%s %d\n", __func__, runtime->rate);
	mutex_lock(&chip->setup_mutex);

	lx_madi_get_clocks_status(chip, &clocks_informations);


	switch(chip->use_clock_sync) {
		case LXMADI_CLOCK_SYNC_MADI:
			//is there a clock ?
			if(clocks_informations.madi_freq<=0){
				//clock not present
				printk(KERN_WARNING  "%s madi clock not present\n", __func__);
				err = -EINVAL;
				goto exit;
			}
			//clock rate ?
			if( runtime->rate != clocks_informations.madi_freq){
				printk(KERN_WARNING  "%s expected rate and madi clock are different expected %d found %d\n", __func__, runtime->rate, clocks_informations.madi_freq);
				err = -EINVAL;
				goto exit;
			}

			if(chip == lx_chips_master){
				if(lx_chips_slave != NULL){
					//should be updated in 50ms
					for(i = 0; i< 10; i++){
						lx_madi_get_clocks_status(lx_chips_slave, &clocks_informations);
						if(clocks_informations.word_clock_freq == runtime->rate){
		//					printk(KERN_DEBUG "%s, slave had been updated in %d loop\n", __func__, i);
							break;
						}else{
							mdelay(10);
						}
					}
					if(clocks_informations.word_clock_freq != runtime->rate){
						printk(KERN_ERR "%s, be careful Master and Slave looks not synchronize by wordclock\n", __func__);
					}
				}
			}
			break;

		case LXMADI_CLOCK_SYNC_WORDCLOCK:
			//is there a clock ?
			if(clocks_informations.word_clock_freq<=0){
				//clock not present
				printk(KERN_WARNING  "%s madi clock not present\n", __func__);
				err = -EINVAL;
				goto exit;
			}
			//clock rate ?
			if( runtime->rate != clocks_informations.word_clock_freq){
				printk(KERN_WARNING  "%s expected rate and word clock are different expected %d found %d\n", __func__, runtime->rate, clocks_informations.word_clock_freq);
				err = -EINVAL;
				goto exit;
			}
			break;
		case LXMADI_CLOCK_SYNC_INTERNAL:
//			runtime->rate = clocks_informations.internal_freq;
			for(i = 0; i<4; i++){
				if(madi_internal_clock_frequency_value[i] == runtime->rate){
					chip->madi_frequency_selector = i;
				}
			}
			if(madi_internal_clock_frequency_value[chip->madi_frequency_selector] != runtime->rate){
				printk(KERN_WARNING  "%s unsupported rate\n", __func__);

				err = -EINVAL;
				goto exit;
			}

			lx_madi_set_clock_frequency(chip, madi_internal_clock_frequency_value[chip->madi_frequency_selector]);

			break;
		default:
			//unknown sync....
			break;
	}
	if(runtime->rate > 48000){
		if(chip->channel_mode == LXMADI_32_64_CHANNELS){
			runtime->hw.channels_max = 32;
		}else{
			runtime->hw.channels_max = 24;
		}
	}else {
		if(chip->channel_mode == LXMADI_32_64_CHANNELS){
			runtime->hw.channels_max = 64;
		}else{
			runtime->hw.channels_max = 56;
		}
	}
	if(runtime->channels > runtime->hw.channels_max ){
		printk(KERN_WARNING  "%s nb channels max %d\n", __func__, runtime->hw.channels_max);
		err = -EINVAL;
		goto exit;
	}


	lx_madi_set_clock_diviseur(chip, (unsigned char)chip->diviseur_mode);
	lx_madi_set_madi_state(chip);
    lx_madi_set_clock_frequency(chip, substream->runtime->rate);

	mutex_unlock(&chip->setup_mutex);
	err = lx_pcm_prepare(substream);
	return err;

exit:
	mutex_unlock(&chip->setup_mutex);
	return err;

}


#define SYNC_START
static int lxmadi_pcm_trigger(struct snd_pcm_substream *substream, int cmd){

	int err = 0;
#ifdef SYNC_START
	struct lx6464es *chip = snd_pcm_substream_chip(substream);
	struct lx6464es *link_chip = NULL;
	struct snd_pcm_substream *s;
	struct lx_stream *link_lx_stream; //, *link_lx_stream_play, *link_lx_stream_record;

	printk(KERN_DEBUG  "%s cmd %x chip %p\n", __func__, cmd, chip);
#endif
#ifndef SYNC_START
	err = lx_pcm_trigger(substream, cmd);
	return err;
#endif
#ifdef SYNC_START
	if(chip->multi_card_sync_mode == LXMADI_SYNC_INDEPENDENT || cmd == SNDRV_PCM_TRIGGER_STOP ){
//		printk(KERN_DEBUG  "%s NORMAL TRIG\n", __func__);
		err = lx_pcm_trigger(substream, cmd);

	}
	//Normal startup
	else { //if(chip->multi_card_sync_mode == LXMADI_SYNC_MASTER){
		//check if there is a slave and a master
//		printk(KERN_DEBUG  "%s SYNC TRIG for START\n", __func__);
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			if (snd_pcm_stream_linked(substream)) {
				snd_pcm_group_for_each_entry(s, substream) {
					link_chip = snd_pcm_substream_chip(s);
					if(s == link_chip->capture_stream.stream){
						link_lx_stream = &link_chip->capture_stream;
					}else {
						link_lx_stream = &link_chip->playback_stream;
					}

					while(link_lx_stream->status == LX_STREAM_STATUS_SCHEDULE_STOP);//if tasklet pending

					link_lx_stream->status = LX_STREAM_STATUS_SCHEDULE_RUN;
					snd_pcm_trigger_done(s, substream);
				}

				lx_trigger_pipes_start(lx_chips_master);
				lx_trigger_pipes_start(lx_chips_slave);
			}
		default:
			//je ne dois pas passer ici.
			break;
		}


	}
	printk(KERN_DEBUG "%s, err %d\n", __func__, err);

	return err;
#endif
}

static struct snd_pcm_ops lx_ops_playback = {
	.open      = lx_pcm_open,
	.close     = lx_pcm_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.prepare   = lx_madi_pcm_prepare,
	.hw_params = lx_pcm_hw_params,
	.hw_free   = lx_pcm_hw_free,
	.trigger   = lxmadi_pcm_trigger,
	.pointer   = lx_pcm_stream_pointer,
};

static struct snd_pcm_ops lx_ops_capture = {
	.open      = lx_pcm_open,
	.close     = lx_pcm_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.prepare   = lx_madi_pcm_prepare,
	.hw_params = lx_pcm_hw_params,
	.hw_free   = lx_pcm_hw_free,
	.trigger   = lxmadi_pcm_trigger,
	.pointer   = lx_pcm_stream_pointer,
};



int lx_madi_pcm_create(struct lx6464es *chip)
{
	int err;
	struct snd_pcm *pcm;

	u32 size =  MADI_USE_CHANNELS_MAX *		  /* channels */
				MADI_SAMPLE_SIZE_MAX *		  /* 24 bit samples */
				MADI_USE_PERIODS_MAX *        /* periods */
				MADI_GRANULARITY_MAX *        /* frames per period */
				MADI_PERIOD_MULTIPLE_GRAN_MAX;/* max period size */

//	printk(KERN_DEBUG  "%s\n", __func__);
	size = PAGE_ALIGN(size);

	/* hardcoded device name & channel count */
	err = snd_pcm_new(chip->card, (char *)"LX MADI", 0, 1, 1, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = chip;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &lx_ops_playback);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &lx_ops_capture);
	pcm->info_flags = 0;
	strcpy(pcm->name, "LX MADI");
	err = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
						    snd_dma_pci_data(chip->pci),
						    size, size);
	if (err < 0){
		printk(KERN_ERR "%s, snd_pcm_lib_preallocate_pages_for_all failed", __func__);
		return err;
	}
	chip->pcm = pcm;
	chip->capture_stream.is_capture = 1;
	return 0;
}
static cpumask_t irq_enabled_cpus;//TODO mettre dans la structure

static int snd_lxmadi_create(struct snd_card *card,
			       struct pci_dev *pci,
			       struct lx6464es **rchip)
{
	struct lx6464es *chip;
	int err;
	unsigned int idx;
	struct snd_kcontrol *kcontrol;
	unsigned char nbCpu = num_online_cpus();
	int i=0;

	static struct snd_device_ops ops = {
		.dev_free = snd_lx6464es_dev_free,
	};

//	printk(KERN_DEBUG  "%s\n", __func__);

	*rchip = NULL;

	/* enable PCI device */
	err = pci_enable_device(pci);
	if (err < 0)
		return err;

	pci_set_master(pci);

	/* check if we can restrict PCI DMA transfers to 32 bits */
//SJR 4.1	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
	err = dma_set_mask(&pci->dev, DMA_BIT_MASK(32));
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0)
	err = pci_set_dma_mask(pci, DMA_BIT_MASK(32));
#elif LINUX_VERSION_CODE == KERNEL_VERSION(3,10,17)
	err = pci_set_dma_mask(pci, DMA_BIT_MASK(32));
#else
#error "kernel not supported"
#endif

	if (err < 0) {
		printk(KERN_ERR "%s, architecture does not support "
			   "32bit PCI busmaster DMA\n", __func__);
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
	if(pci->subsystem_device == PCIEX_SUBDEVICE_ID_DIGIGRAM_LXMADI_SERIAL_SUBSYSTEM){
		chip->lx_type = LX_MADI;


	}

	//set default internal card conf to local
	chip->pcm_hw               = lx_madi_caps;
	chip->use_clock_sync       = LXMADI_CLOCK_SYNC_INTERNAL; //valeur par defaut de la carte
	chip->lx_type              = LX_MADI;
	chip->channel_mode         = LXMADI_32_64_CHANNELS; //64 default
	chip->diviseur_mode        = LXMADI_512; //512
	chip->rx_tx_mode           = LXMADI_SMUX;	// smux
	chip->word_clock_out       = LXMADI_WORD_CLOCK_IN;
	chip->multi_card_sync_mode = LXMADI_SYNC_INDEPENDENT;
	/* initialize synchronization structs */
	mutex_init(&chip->msg_lock);
	mutex_init(&chip->setup_mutex);
	chip->lx_chip_index =lx_chips_count;

	/* request resources */
	err = pci_request_regions(pci, card_name);
	if (err < 0)
		goto request_regions_failed;

	/* plx port */
	chip->port_plx = pci_resource_start(pci, 1);
//	chip->port_plx_remapped = ioport_map(chip->port_plx,
//					     pci_resource_len(pci, 1));

	chip->port_plx_remapped = pci_iomap(pci, 1, 0);

	/* dsp port */
	chip->port_dsp_bar = pci_ioremap_bar(pci, 2);
	
	chip->capture_stream.status = LX_STREAM_STATUS_STOPPED;
	chip->playback_stream.status = LX_STREAM_STATUS_STOPPED;



//	atomic_set(&chip->debug_irq.irq_all, 0);
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
//	err = request_irq(pci->irq, lx_interrupt,
//					IRQF_SHARED, "LX-MADI", chip);


	err = request_threaded_irq(pci->irq, lx_interrupt, NULL,
							IRQF_SHARED, "LX-MADI", chip);

	if (err) {
		printk(KERN_ERR "%s, unable to grab IRQ %d\n", __func__, pci->irq);
		goto request_irq_failed;
	}
	chip->irq = pci->irq;



	//#ifdef ONE_HANDLER_TO_RULE_THEM_ALL
//	if(lx_chips_count == 1){
//		err = request_irq(pci->irq, lx_interrupt2,
//						IRQF_SHARED, "LX-MADI", chip);
//
//		if (err) {
//			printk(KERN_ERR "%s, unable to grab IRQ %d\n", __func__, pci->irq);
//			goto request_irq_failed;
//		}
//		chip->irq = pci->irq;
//	}
//#else
//	if(lx_chips_count == 0){
//		err = request_threaded_irq(pci->irq, lx_interrupt, lx_threaded_irq,
//				IRQF_SHARED, "LX-MADI", chip);
////		err = request_irq(pci->irq, lx_interrupt,
////						IRQF_SHARED, "LX-MADI", chip);
//
//	}
//	else{
//		err = request_threaded_irq(pci->irq, lx_interrupt2, lx_threaded_irq2,
//				IRQF_SHARED, "LX-MADI", chip);
////		err = request_irq(pci->irq, lx_interrupt2,
////						IRQF_SHARED, "LX-MADI2", chip);
//
//	}
////		err = request_irq(pci->irq, lx_interrupt,
////				IRQF_SHARED, "LX-MADI", chip);
//	if (err) {
//		printk(KERN_ERR "%s, unable to grab IRQ %d\n", __func__, pci->irq);
//		goto request_irq_failed;
//	}
//#endif


	nbCpu = num_online_cpus();
	i=0;
	for(i=0; i< nbCpu; i++){
		cpumask_clear_cpu(i, &irq_enabled_cpus);
	}
	//		cpumask_set_cpu(nbCpu-indexcpu[0], &irq_enabled_cpus);
	cpumask_set_cpu(0x07, &irq_enabled_cpus);
	//irq_set_affinity(chip->irq, &irq_enabled_cpus);
//	printk(KERN_DEBUG "%s nbcpu %x  mask %x %x", __func__, nbCpu, NR_CPUS, irq_enabled_cpus);

//#ifdef NO_LOW_LATENCY_IRQ

//	if(chip->pThread == NULL)
//		lx_create_thread(chip);
//#endif


	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, chip, &ops);
	if (err < 0)
		goto device_new_failed;

	err = lx_init_dsp(chip);
	if (err < 0) {
		printk(KERN_ERR "%s, error during DSP initialization\n", __func__);
		goto device_new_failed;
	}

	err = lx_madi_pcm_create(chip);
	if (err < 0){
		printk(KERN_ERR "%s,lx_madi_pcm_create failed\n", __func__);
		goto device_new_failed;
	}

	err = lx_proc_create(card, chip);
	if (err < 0){
		printk(KERN_ERR "%s,lx_proc_create failed\n", __func__);
		goto device_new_failed;
	}
	err = lx_madi_proc_create(card, chip);
	if (err < 0){
		printk(KERN_ERR "%s,lx_proc_create failed\n", __func__);
		goto device_new_failed;
	}


//	err = snd_ctl_add(card, snd_ctl_new1(&lx_control_playback_switch,chip));
//	if (err < 0){
//		printk(KERN_ERR "%s,snd_ctl_add failed\n", __func__);
//		return err;
//	}

	for (idx = 0; idx < ARRAY_SIZE(snd_lxmadi_controls); idx++) {
		kcontrol = snd_ctl_new1(&snd_lxmadi_controls[idx], chip);
		err = snd_ctl_add(card, kcontrol);
		if (err < 0){
			printk(KERN_ERR "%s,snd_ctl_add failed\n", __func__);
			goto device_new_failed;
		}
		if (!strcmp(kcontrol->id.name, "Work clock direction"))
			chip->mixer_wordclock_out_ctl = kcontrol;

		if (!strcmp(kcontrol->id.name, "Clock Mode"))
			chip->mixer_current_clock_ctl = kcontrol;


//		if (!strcmp(kcontrol->id.name, "CD Volume"))
//			dummy->cd_volume_ctl = kcontrol;
//		else if (!strcmp(kcontrol->id.name, "CD Capture Switch"))
//			dummy->cd_switch_ctl = kcontrol;

	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
	//nothing
#elif LINUX_VERSION_CODE == KERNEL_VERSION(3,19,0)
	snd_card_set_dev(card, &pci->dev);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,17)
	snd_card_set_dev(card, &pci->dev);
#endif
	*rchip = chip;


	lx_madi_set_clock_diviseur(chip, (unsigned char)chip->diviseur_mode);
	lx_madi_set_madi_state(chip);



	err = lx_pipe_open(chip,0,MADI_USE_CHANNELS_MAX);
	if (err < 0) {
//		dev_err(chip->card->dev, "setting granularity to %ld failed\n",
//			   period_size);
		printk(KERN_ERR "setting lx_pipe_open failed\n");
		goto device_new_failed;
	}

	err = lx_pipe_open(chip, 1, MADI_USE_CHANNELS_MAX);
	if (err < 0) {
//		dev_err(chip->card->dev, "setting granularity to %ld failed\n",
//			   period_size);
		printk(KERN_ERR "setting lx_pipe_open failed\n");
		goto device_new_failed;
	}

	lx_chips[lx_chips_count++] = chip;

	return 0;

device_new_failed:
	if(chip->irq >= 0){
		free_irq(pci->irq, chip);
	}

request_irq_failed:
	pci_release_regions(pci);

request_regions_failed:
	kfree(chip);

alloc_failed:
	pci_disable_device(pci);

	return err;
}

static int snd_lxmadi_probe(struct pci_dev *pci,
			      const struct pci_device_id *pci_id)
{
	static int dev;
	struct snd_card *card;
	struct lx6464es *chip;
	int err;
//	printk(KERN_DEBUG  "%s\n", __func__);

	if (dev >= SNDRV_CARDS)
		return -ENODEV;
	if (!enable[dev]) {
		dev++;
		return -ENOENT;
	}

//4.1.9
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
	err = snd_card_new(&pci->dev, index[dev], id[dev], THIS_MODULE,
			   0, &card);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0)
	err = snd_card_new(&pci->dev, index[dev], id[dev], THIS_MODULE,
			   0, &card);
#elif LINUX_VERSION_CODE == KERNEL_VERSION(3,10,17)
	err = snd_card_create(index[dev], id[dev], THIS_MODULE, 0, &card);
#else
#error "kernel not supported"
#endif
	if (err < 0)
		return err;


	err = snd_lxmadi_create(card, pci, &chip);
	if (err < 0) {
		printk(KERN_ERR "%s, error during snd_lxmadi_create\n", __func__);
		goto out_free;
	}

	card->private_data = chip;
	strcpy(card->driver, "LXMADI");
	sprintf(card->id, "LX_MADI");
	sprintf(card->longname, "%s at 0x%lx, 0x%p, irq %i",
		card->shortname, chip->port_plx,
		chip->port_dsp_bar, chip->irq);


	err = snd_card_register(card);
	if (err < 0)
		goto out_free;

	printk(KERN_ERR  "%s, initialization successful dev %d lxcount %d\n", __func__,dev,lx_chips_count);
	pci_set_drvdata(pci, card);

	dev++;
	return 0;

out_free:
	snd_card_free(card);
	return err;

}

static void snd_lxmadi_remove(struct pci_dev *pci)
{
	struct snd_card *card  = pci_get_drvdata(pci);
	struct lx6464es *chip = card->private_data;
	int is_capture = 0;
	int err = 0;

	for(is_capture = 0; is_capture <= 1; is_capture++){
		if (chip->hardware_running[is_capture] > 1) {
			err = lx_pipe_stop(chip, is_capture);
			if (err < 0) {
				printk(KERN_ERR "%s, failed to stop hardware. "
					   "Error code %d\n",__func__, err);
//				goto exit;
			}

			chip->hardware_running[is_capture] = 1;
		}
		if (chip->hardware_running[is_capture] == 1) {

			err = lx_pipe_close(chip, is_capture);
			if (err < 0) {
				printk(KERN_ERR "%sfailed to close hardware. "
					   "Error code %d\n",__func__, err);
//				goto exit;
			}
			chip->hardware_running[is_capture] = 0;
		}
	}

	lx_chips_count--;

//	if (chip->pThread != NULL)
//		lx_stop_thread(chip);

	snd_card_free(pci_get_drvdata(pci));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,0)
	//nothing
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,19,0)
	pci_set_drvdata(pci, NULL);
#elif LINUX_VERSION_CODE == KERNEL_VERSION(3,10,17)
	pci_set_drvdata(pci, NULL);
#endif
}


static struct pci_driver lxmadi_driver = {
//	.name =     KBUILD_MODNAME,
	.name =     "LX-MADI",
	.id_table = snd_lxmadi_ids,
	.probe =    snd_lxmadi_probe,
	.remove = snd_lxmadi_remove,
};

module_pci_driver(lxmadi_driver);


MODULE_AUTHOR("Sylvain Jubier <jubier@digigram.com> ");
MODULE_DESCRIPTION("digigram lxmadi");
MODULE_LICENSE("GPL v2");
