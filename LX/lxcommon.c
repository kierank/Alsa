/*
 *
 * ALSA driver for the digigram lx audio interface
 *
 * Copyright (c) 2016 Jubier Sylvain <alsa@digigram.com>
 */

#include <linux/printk.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include "lxcommon.h"
#include "lxmadi.h"

int lx_chips_count;
struct lx_chip *lx_chips[SNDRV_CARDS] = {NULL};

int lx_set_granularity(struct lx_chip *chip, u32 gran)
{
	int err = 0;
	u32 snapped_gran = MICROBLAZE_IBL_MIN;

/*printk(KERN_DEBUG  "\t%s granularity = %d,
*                        chip->pcm_granularity %d\n",
*                        __func__,
*                        gran,
*                        chip->pcm_granularity);
*/
	/* blocksize is a power of 2 */
	while ((snapped_gran < gran) && (snapped_gran < MICROBLAZE_IBL_MAX))
		snapped_gran *= 2;

	if (snapped_gran == (u32)chip->pcm_granularity)
		return 0;

	err = lx_dsp_set_granularity(chip, snapped_gran);
	if (err < 0) {
		dev_warn(chip->card->dev,
			"%s, could not set granularity\n",
			__func__);
		err = -EAGAIN;
	}

	if (snapped_gran != gran)
		dev_warn(chip->card->dev,
			"snapped blocksize to %d\n", snapped_gran);

	dev_warn(chip->card->dev,
		"%s, set blocksize on board %d\n",
		__func__, snapped_gran);
	chip->pcm_granularity = (u16)snapped_gran;

	return err;
}

int lx_pipe_open(struct lx_chip *chip, int is_capture, int channels)
{
	int err = 0;
/*        printk(KERN_DEBUG "\t%s, allocating pipe for %d channels\n",
*                        __func__,
*                        channels);
*/
	err = lx_pipe_allocate(chip, 0, is_capture, channels);
	if (err < 0) {
		dev_err(chip->card->dev, "allocating pipe failed\n");
		return err;
	}
	return err;
}

int lx_stream_create_and_start(struct lx_chip *chip,
		struct snd_pcm_substream *substream)
{
	int err = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int is_capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);

/*        printk(KERN_DEBUG  "\t%s is_capture : %d\n", __func__, is_capture); */
	/* setting stream format */
	err = lx_stream_def(chip, runtime, 0, is_capture);
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s : setting lx stream format failed\n",
			__func__);
		return err;
	}
	err = lx_stream_start(chip, 0, is_capture);
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, couldn't start lxstream\n",
			__func__);
	}
	chip->hardware_running[is_capture] = 1;

	return err;
}

int lx_pipe_stop(struct lx_chip *chip, int is_capture)
{
	int err = 0;

	err = lx_pipe_wait_for_idle(chip, 0, is_capture);
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, waiting for pipe failed\n", __func__);
		return err;
	}

	err = lx_pipe_stop_single(chip, 0, is_capture);
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, stopping pipe failed\n", __func__);
		return err;
	}
	return err;
}

int lx_pipe_close(struct lx_chip *chip, int is_capture)
{
	int err = 0;
/*        printk(KERN_DEBUG  "\t%s\n", __func__); */

	err = lx_pipe_wait_for_idle(chip, 0, is_capture);
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, lx_pipe_wait_for_idle failed\n", __func__);
		err = lx_pipe_pause_single(chip, 0, is_capture);
		if (err < 0) {
			dev_err(chip->card->dev,
				"%s, lx_pipe_pause_multiple failed\n",
				__func__);
			return err;
		}
	}

	err = lx_pipe_release(chip, 0, is_capture);
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, releasing pipe failed\n", __func__);
	}

	return err;
}

int lx_pcm_open(struct snd_pcm_substream *substream)
{
	struct lx_chip *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err = 0;
	int board_rate;

/*        printk(KERN_DEBUG "%s is_capture %d\n",
*                        __func__,
*                        (substream->stream == SNDRV_PCM_STREAM_CAPTURE));
*/
	mutex_lock(&chip->setup_mutex);

	/* copy the struct snd_pcm_hardware struct */
	runtime->hw = chip->pcm_hw;

	chip->jiffies_start = -1;
	chip->jiffies_1st_irq = -1;

	switch (chip->lx_type) {
	case LX_ETHERSOUND:
		/* the clock rate cannot be changed */
		board_rate = chip->board_sample_rate;
		err = snd_pcm_hw_constraint_minmax(runtime,
		SNDRV_PCM_HW_PARAM_RATE, board_rate, board_rate);
		if (err < 0) {
			dev_err(chip->card->dev,
				"%s, could not constrain periods\n",
				__func__);
			goto exit;
		}

		/* constrain period size */
		err = snd_pcm_hw_constraint_minmax(runtime,
		SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
		MICROBLAZE_IBL_MIN,
		MICROBLAZE_IBL_MAX);
		if (err < 0) {
			dev_err(chip->card->dev,
				"%s, could not constrain period size\n",
				__func__);
			goto exit;
		}
		err = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 32);
		if (err < 0) {
			dev_err(chip->card->dev,
			"%s, snd_pcm_hw_constraint_step failed\n", __func__);
			goto exit;
		}

		break;
	case LX_MADI:
	case LX_IP:
	case LX_IP_MADI:
		err = snd_pcm_hw_constraint_list(runtime, 0,
		SNDRV_PCM_HW_PARAM_RATE, &lx_madi_hw_constraints_sample_rates);
		if (err < 0) {
			dev_err(chip->card->dev,
				"%s, could not constrain freq rate\n",
				__func__);
			goto exit;
		}
		/* buffer-size should better be multiple of period-size */
/*		err = snd_pcm_hw_constraint_integer(runtime,
*				SNDRV_PCM_HW_PARAM_PERIODS);
*/
		break;

	default:
		return -ENODEV;
	}
	snd_pcm_set_sync(substream);
	if (err > 0)
		err = 0;

exit:
	runtime->private_data = chip;
	mutex_unlock(&chip->setup_mutex);
	return err;
}

static void lx_trigger_stream_stop(struct lx_chip *chip,
		unsigned int is_capture)
{
	int err;
/*        printk(KERN_DEBUG  "%s\n", __func__);*/

	err = lx_stream_stop(chip, 0, is_capture);
	if (err < 0) {
		dev_err(chip->card->dev, LXP "couldn't stop pipe\n");
	} else {
		if (is_capture == 0)
			chip->playback_stream.status = LX_STREAM_STATUS_STOPPED;
		else
			chip->capture_stream.status = LX_STREAM_STATUS_STOPPED;
	}
}

int lx_pcm_close(struct snd_pcm_substream *substream)
{
	int err = 0;
	int is_capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	struct lx_chip *chip = snd_pcm_substream_chip(substream);

/*        printk(KERN_DEBUG  "%s is_capture : %d chip -> %p\n",
*                        __func__,
*                        (substream->stream == SNDRV_PCM_STREAM_CAPTURE),
*                        chip);
*/
	if (is_capture)
		chip->capture_stream.stream = NULL;
	else
		chip->playback_stream.stream = NULL;

/*        printk(KERN_DEBUG  "%s is_capture : %d end\n",
*                        __func__,
*                        (substream->stream == SNDRV_PCM_STREAM_CAPTURE));
*/
	return err;
}

snd_pcm_uframes_t lx_pcm_stream_pointer(struct snd_pcm_substream *substream)
{
	struct lx_chip *chip = snd_pcm_substream_chip(substream);
	snd_pcm_uframes_t pos;
	int is_capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	int xrun = 0;
	struct lx_stream *lx_stream = is_capture ?
			&chip->capture_stream : &chip->playback_stream;

	/* printk(KERN_DEBUG  "%s\n", __func__); */
	if (is_capture == 0) {
		xrun = atomic_read(&chip->play_xrun_advertise);
		atomic_set(&chip->play_xrun_advertise, 0);
	} else {
		xrun = atomic_read(&chip->capture_xrun_advertise);
		atomic_set(&chip->capture_xrun_advertise, 0);
	}
	if (xrun != 0) {
		pos = SNDRV_PCM_POS_XRUN;
		dev_err(chip->card->dev,
			"%s advertise XRUN to userspace\n",
			__func__);
	} else {
		pos = lx_stream->frame_pos * substream->runtime->period_size;
	}
	return pos;
}

int lx_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct lx_chip *chip = snd_pcm_substream_chip(substream);
	int err = 0;
	const int is_capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	u32 periods = substream->runtime->periods;
	const u32 channels = substream->runtime->channels;
	dma_addr_t buf;
	u32 buffer_size = 0;
	u32 buffer_index = 0;
	unsigned char period_multiple_gran = 0;
	struct lx_stream *lx_stream = is_capture ?
			&chip->capture_stream : &chip->playback_stream;

/*        printk(KERN_DEBUG
*        "%s chip %p,  is_capture : %d, nb period : %d, nb channels : %d\n",
*        __func__,
*        chip,
*        is_capture,
*        periods,
*        channels);
*/

	while ((lx_stream->status == LX_STREAM_STATUS_SCHEDULE_STOP))
		;
	mutex_lock(&chip->setup_mutex);

	if (substream->runtime->period_size < chip->pcm_granularity) {
		dev_err(chip->card->dev,
		"period size (%d) has to be multiple of dma granularity (%d)\n",
		(unsigned int)substream->runtime->period_size,
		chip->pcm_granularity);
		err = -EPERM;
		goto exit;

	} else {
		if ((substream->runtime->period_size % chip->pcm_granularity)
				!= 0) {
			dev_err(chip->card->dev,
	"period size (%d) has to be multiple of dma granularity (%d) %d\n",
			(unsigned int)substream->runtime->period_size,
			chip->pcm_granularity,
			(unsigned int)substream->runtime->period_size
						% chip->pcm_granularity);

			err = -EPERM;
			goto exit;

		}
		period_multiple_gran = substream->runtime->period_size
				/ chip->pcm_granularity;
		if (is_capture == 0)
			chip->play_period_multiple_gran = period_multiple_gran;
		else
			chip->capture_period_multiple_gran =
					period_multiple_gran;
	}
/*        printk(KERN_DEBUG "%s, %d %d %d\n",
*                        __func__,
*                        period_multiple_gran,
*                        chip->play_period_multiple_gran,
*                        chip->capture_period_multiple_gran);
*/
	err = lx_stream_create_and_start(chip, substream);
	if (err < 0) {
/*	dev_err(chip->card->dev, "setting granularity to %ld failed\n",
*		period_size);
*/
		dev_err(chip->card->dev, "setting lx_pipe_start failed\n");
		goto exit;
	}

	if (chip->board_sample_rate != substream->runtime->rate)
		if (!err)
			chip->board_sample_rate = substream->runtime->rate;

	/* prepare lx buffer */
	buf = substream->dma_buffer.addr;

	if (is_capture == 0)
		chip->playback_stream.frame_pos = 0;
	else
		chip->capture_stream.frame_pos = 0;

	buffer_size =	/* 24 bit samples | frame size  */
			channels * 3 *
			/* frames per channels | gran */
			periods * substream->runtime->period_size;

	err = lx_buffer_give(chip, 0, is_capture, buffer_size,
			lower_32_bits(buf), upper_32_bits(buf), &buffer_index,
			period_multiple_gran);
	if (err < 0)
		dev_err(chip->card->dev,
			"%s, lx_buffer_give err = %d\n", __func__, err);

exit:
	mutex_unlock(&chip->setup_mutex);
	if (err >= 0) {
		err = 0;
		if (is_capture == 1)
			chip->capture_stream_prerared = 1;
		else
			chip->playback_stream_prerared = 1;
	} else if (err < 0)
		dev_err(chip->card->dev,
			"%s,------------>err = %d\n", __func__, err);
/*        printk(KERN_DEBUG "%s END is_capture : %d, err %x\n",
*                        __func__,
*                        is_capture,
*                        err);
*/
	return err;
}

int lx_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	struct lx_chip *chip = snd_pcm_substream_chip(substream);
	int err = 0;
	int is_capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
/*        printk(KERN_DEBUG  "\t%s is_capture : %d\n",
*                        __func__,
*                     (substream->stream == SNDRV_PCM_STREAM_CAPTURE));
*/
	mutex_lock(&chip->setup_mutex);

	/* set dma buffer */
	err = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, snd_pcm_lib_malloc_pages. Error code %d\n",
			__func__, err);
		goto exit;
	}
	if (is_capture)
		chip->capture_stream.stream = substream;
	else
		chip->playback_stream.stream = substream;

exit:
	mutex_unlock(&chip->setup_mutex);
	return err;
}

int lx_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct lx_chip *chip = snd_pcm_substream_chip(substream);
	int err = 0;
	int is_capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	int i = 0;
	struct lx_stream *lx_stream = is_capture ?
			&chip->capture_stream : &chip->playback_stream;

/*        printk(KERN_DEBUG  "%s\n", __func__); */
/*        printk(KERN_DEBUG
*                "%s is_capture : %d   chip -> %p\n",
*                __func__,
*                (substream->stream == SNDRV_PCM_STREAM_CAPTURE),
*                chip);
*/
	/* if command pending */
	while ((lx_stream->status == LX_STREAM_STATUS_SCHEDULE_STOP)
			&& (i < 1000)) {

		udelay(1);
		i++;
	}
	if (i >= 1000) {
		dev_err(chip->card->dev, "%s TIMEOUT\n", __func__);
		return -EIO;
	}
	lx_trigger_stream_stop(chip, is_capture);

	mutex_lock(&chip->setup_mutex);
	for (i = 0; i < MICROBLAZE_LX_PCI_PERIODS_MAX; i++)
		lx_buffer_cancel(chip, 0, is_capture, i);

	err = snd_pcm_lib_free_pages(substream);

	mutex_unlock(&chip->setup_mutex);
/*        printk(KERN_DEBUG  "%s  err %d\n", __func__, err); */

	return err;
}

void lx_trigger_pipe_start(struct lx_chip *chip, unsigned int is_capture)
{
	int err;
/*        printk(KERN_DEBUG  "%s %d\n", __func__, is_capture); */
	if (is_capture == 0)
		chip->irq_audio_cpt_play = (unsigned int)-1;
	else
		chip->irq_audio_cpt_record = (unsigned int)-1;

	err = lx_pipe_start_single(chip, 0, is_capture);
	chip->jiffies_start = jiffies;

	if (err < 0) {
		dev_err(chip->card->dev,
			"%s : starting lx pipe failed\n", __func__);
		dev_err(chip->card->dev,
			"%s, couldn't start alsa stream\n", __func__);
	} else {
		chip->hardware_running[is_capture] = 2;
		if (is_capture == 0)
			chip->playback_stream.status = LX_STREAM_STATUS_RUNNING;
		else
			chip->capture_stream.status = LX_STREAM_STATUS_RUNNING;
	}
}

/*for linked streams*/
void lx_trigger_pipes_start(struct lx_chip *chip)
{
	int err;

/*        printk(KERN_DEBUG  "%s %p\n", __func__, chip);*/
	chip->irq_audio_cpt_play = (unsigned int)-1;
	chip->irq_audio_cpt_record = (unsigned int)-1;
	err = lx_pipe_start_multiple(chip);
	chip->jiffies_start = jiffies;
	if (err < 0) {
		chip->capture_stream.status = LX_STREAM_STATUS_STOPPED;
		chip->playback_stream.status = LX_STREAM_STATUS_STOPPED;
		dev_err(chip->card->dev,
			"%s : starting lx pipe failed\n", __func__);
		dev_err(chip->card->dev,
			"%s, couldn't start alsa stream\n", __func__);
	} else {
		chip->hardware_running[0] = 2;
		chip->hardware_running[1] = 2;
		chip->capture_stream.status = LX_STREAM_STATUS_RUNNING;
		chip->playback_stream.status = LX_STREAM_STATUS_RUNNING;
	}
}

/*for linked stream*/
void lx_trigger_pipes_stop(struct lx_chip *chip)
{
	int err;
/*        printk(KERN_DEBUG  "%s %p\n", __func__, chip); */

	err = lx_pipe_pause_multiple(chip);
	if (err < 0) {
		chip->capture_stream.status = LX_STREAM_STATUS_RUNNING;
		chip->playback_stream.status = LX_STREAM_STATUS_RUNNING;
		dev_err(chip->card->dev,
			"%s : pause lx pipe failed\n", __func__);
		dev_err(chip->card->dev,
			"%s, couldn't pause alsa stream\n", __func__);
	} else {
		chip->hardware_running[0] = 1;
		chip->hardware_running[1] = 1;
		chip->capture_stream.status = LX_STREAM_STATUS_STOPPED;
		chip->playback_stream.status = LX_STREAM_STATUS_STOPPED;
	}
}
static void lx_trigger_pipe_stop(struct lx_chip *chip, unsigned int is_capture)
{
	int err;
/*        printk(KERN_DEBUG  "%s\n", __func__);*/

	err = lx_pipe_pause_single(chip, 0, is_capture);
	if (err < 0)
		dev_err(chip->card->dev, LXP "couldn't stop pipe\n");
	else {
		err = lx_pipe_wait_for_idle(chip, 0, is_capture);
		if (err < 0)
			dev_err(chip->card->dev, "%s : wait for idle failed for %d\n",
					__func__, is_capture);
		if (is_capture == 0)
			chip->playback_stream.status = LX_STREAM_STATUS_STOPPED;
		else
			chip->capture_stream.status = LX_STREAM_STATUS_STOPPED;
	}
}

static void lx_trigger_finalize(struct lx_chip *chip,
		struct snd_pcm_substream *substream)
{
	struct lx_chip *link_chip = NULL;
	struct snd_pcm_substream *s;
	struct lx_stream *link_lx_stream_play, *link_lx_stream_record;

	unsigned long j1;
/*        printk(KERN_DEBUG  "%s\n", __func__);*/

	snd_pcm_group_for_each_entry(s, substream) {
		link_chip = snd_pcm_substream_chip(s);
		link_lx_stream_record = &link_chip->capture_stream;
		link_lx_stream_play = &link_chip->playback_stream;
		if ((link_lx_stream_record->status ==
					LX_STREAM_STATUS_SCHEDULE_RUN) &&
			(link_lx_stream_play->status ==
					LX_STREAM_STATUS_SCHEDULE_RUN)) {
			lx_interrupt_debug_events(link_chip);
			lx_trigger_pipes_start(link_chip);
/*                      printk (KERN_DEBUG "%s, "
*                              "lx_trigger_pipes_start chip %p link_chip %p\n",
*                              __func__,
*                              chip,
*                              link_chip);
*/
		} else if (link_lx_stream_record->status ==
				LX_STREAM_STATUS_SCHEDULE_RUN) {
			lx_interrupt_debug_events(chip);
			lx_trigger_pipe_start(chip, 1);
		} else if (link_lx_stream_play->status ==
				LX_STREAM_STATUS_SCHEDULE_RUN) {
			lx_interrupt_debug_events(chip);
			lx_trigger_pipe_start(chip, 0);
		} else if ((link_lx_stream_record->status ==
					LX_STREAM_STATUS_SCHEDULE_STOP) &&
				(link_lx_stream_play->status ==
					LX_STREAM_STATUS_SCHEDULE_STOP)) {

			lx_trigger_pipes_stop(link_chip);
			atomic_set(&chip->play_xrun_advertise, 0);
			atomic_set(&chip->capture_xrun_advertise, 0);

/*                        printk (KERN_DEBUG
*                        "%s, lx_trigger_pipes_stop chip %p link_chip %p\n",
*                        __func__,
*                        chip,
*                        link_chip);
*/
		} else if (link_lx_stream_record->status ==
				LX_STREAM_STATUS_SCHEDULE_STOP) {
			lx_trigger_pipe_stop(chip, 1);
			atomic_set(&chip->capture_xrun_advertise, 0);
		} else if (link_lx_stream_play->status ==
				LX_STREAM_STATUS_SCHEDULE_STOP) {
			lx_trigger_pipe_stop(chip, 0);
			atomic_set(&chip->play_xrun_advertise, 0);
		}
	}
	j1 = jiffies;
/*        printk(KERN_DEBUG  "%s %u END\n", __func__, (unsigned int)j1); */
}

int lx_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int err = 0;
	struct lx_chip *chip = snd_pcm_substream_chip(substream);
	struct lx_chip *link_chip;
	const int is_capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	struct lx_stream *lx_stream = is_capture ?
			&chip->capture_stream : &chip->playback_stream;
	struct lx_stream *link_lx_stream;
	struct snd_pcm_substream *s;

/*	printk(KERN_DEBUG "%s cmd %x\n", __func__, cmd); */
/*        mutex_lock(&chip->lock); */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (snd_pcm_stream_linked(substream)) {
			snd_pcm_group_for_each_entry(s, substream) {
				link_chip = snd_pcm_substream_chip(s);
				if (s == link_chip->capture_stream.stream) {
					link_lx_stream =
					&link_chip->capture_stream;
				} else {
					link_lx_stream =
					&link_chip->playback_stream;
				}

				/* if command pending */
/*
 *
 *				while (link_lx_stream->status ==
 *						LX_STREAM_STATUS_SCHEDULE_STOP)
 *					;
 */
				if (link_lx_stream->status ==
						LX_STREAM_STATUS_SCHEDULE_STOP)
					dev_err(link_chip->card->dev,
					"%s, SCHEDULE_STOP forbidden state\n",
						__func__);

				link_lx_stream->status =
				LX_STREAM_STATUS_SCHEDULE_RUN;
				snd_pcm_trigger_done(s, substream);
			}
			lx_trigger_finalize(chip, substream);

		} else {
			/* if command pending */
/*			while (lx_stream->status
*					== LX_STREAM_STATUS_SCHEDULE_STOP)
*				;
*/
			if (lx_stream->status ==
					LX_STREAM_STATUS_SCHEDULE_STOP)
				dev_err(link_chip->card->dev,
					"%s, SCHEDULE_STOP forbidden state\n",
					__func__);

			lx_stream->status = LX_STREAM_STATUS_SCHEDULE_RUN;
			lx_trigger_finalize(chip, substream);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		lx_stream->status = LX_STREAM_STATUS_SCHEDULE_STOP;
		if (snd_pcm_stream_linked(substream)) {
			snd_pcm_group_for_each_entry(s, substream) {
				link_chip = snd_pcm_substream_chip(s);

				if (s == link_chip->capture_stream.stream) {
					link_lx_stream =
					&link_chip->capture_stream;
				} else {
					link_lx_stream =
					&link_chip->playback_stream;
				}
				/* if command pending */
/*
 * 			while (link_lx_stream->status ==
 *						LX_STREAM_STATUS_SCHEDULE_RUN)
 *					;
 */
				if (link_lx_stream->status ==
						LX_STREAM_STATUS_SCHEDULE_RUN)
					dev_err(link_chip->card->dev,
					"%s, SCHEDULE_RUN forbidden state\n",
						__func__);

				link_lx_stream->status =
				LX_STREAM_STATUS_SCHEDULE_STOP;

				snd_pcm_trigger_done(s, substream);
			}
			lx_trigger_finalize(chip, substream);

		} else {
			/*if command pending*/
/*
 *			while (lx_stream->status ==
 *					LX_STREAM_STATUS_SCHEDULE_RUN)
 *				;
 */
			if (lx_stream->status ==
					LX_STREAM_STATUS_SCHEDULE_RUN)
				dev_err(link_chip->card->dev,
				"%s, SCHEDULE_RUN forbidden state\n",
					__func__);

			lx_stream->status = LX_STREAM_STATUS_SCHEDULE_STOP;
			lx_trigger_finalize(chip, substream);
		}
		break;
	default:
		err = -EINVAL;
	}

/*        mutex_unlock(&chip->lock); */
	return err;
}

int snd_lx_free(struct lx_chip *chip)
{
/*        printk(KERN_DEBUG  "%s\n", __func__); */
	lx_irq_disable(chip);
	if (chip->irq >= 0)
		free_irq(chip->irq, chip);
	iounmap(chip->port_dsp_bar);
	ioport_unmap(chip->port_plx_remapped);
	pci_release_regions(chip->pci);
	pci_disable_device(chip->pci);
	kfree(chip);

	return 0;
}

int snd_lx_dev_free(struct snd_device *device)
{
/*        printk(KERN_DEBUG "%s\n", __func__); */
	return snd_lx_free(device->device_data);
}

#define START_PLX_INIT                (0xDECADECA)
#define PLX_READY                     (0xCAFECAFE)
#define CHIPSC_RESET_XILINX           (1L<<16)
#define CHIPSC_RESET_PLX              (1L<<30)

/* reset the dsp during initialization */
int lx_init_xilinx_reset(struct lx_chip *chip)
{
	int i;
	int loop = 100;

	u32 plx_reg = lx_plx_reg_read(chip, ePLX_CHIPSC);

	/* PLX LRESET */
	plx_reg |= CHIPSC_RESET_PLX;
	lx_plx_reg_write(chip, ePLX_CHIPSC, plx_reg);
	plx_reg &= ~CHIPSC_RESET_PLX;
	lx_plx_reg_write(chip, ePLX_CHIPSC, plx_reg);

	usleep_range(1000, 10000);

	/* activate reset of xilinx */
	plx_reg &= ~CHIPSC_RESET_XILINX;
	lx_plx_reg_write(chip, ePLX_CHIPSC, plx_reg);

	lx_plx_reg_write(chip, ePLX_MBOX3, 0);

	plx_reg |= CHIPSC_RESET_XILINX;
	lx_plx_reg_write(chip, ePLX_CHIPSC, plx_reg);

	if ((chip->lx_type == LX_IP) || (chip->lx_type == LX_IP_MADI)) {
		usleep_range(1000, 2000);
		lx_plx_reg_write(chip, ePLX_MBOX3, 0);
		lx_plx_reg_write(chip, ePLX_MBOX3, START_PLX_INIT);
		loop = 4000;
		/* deactivate reset of xilinx */
		for (i = 0; i < loop; ++i) {
			u32 reg_mbox3;

			usleep_range(10000, 20000);
			reg_mbox3 = lx_plx_reg_read(chip, ePLX_MBOX3);
			if (reg_mbox3 == PLX_READY) {
				dev_warn(chip->card->dev,
					"%s, xilinx reset done\n",
					__func__);
/*                              printk(KERN_DEBUG
*                                "%s, xilinx took %d loops %x\n",
*                                __func__,
*                                i,
*                                reg_mbox3);
*/				break;
			}
/*
*			else
*				printk(KERN_DEBUG
*					"%s, xilinx took %d loops %x\n",
*					__func__,
*					i,
*					reg_mbox3);
*/
		}
	} else {
		loop = 100;
		/* deactivate reset of xilinx */
		for (i = 0; i < loop; ++i) {
			u32 reg_mbox3;

			usleep_range(10000, 20000);
			reg_mbox3 = lx_plx_reg_read(chip, ePLX_MBOX3);
			if (reg_mbox3) {
				dev_warn(chip->card->dev,
				"%s, xilinx reset done\n", __func__);
/*
*				printk(KERN_DEBUG
*					"%s, xilinx took %d loops\n",
*					__func__,
*					i);
*/
				break;
			}
		}
	}

	/* clear mr */
	lx_dsp_reg_write(chip, eReg_CSM, 0);

	/* le xilinx ES peut ne pas etre encore pret, on attend. */
	msleep(600);

	return 0;
}

int lx_init_xilinx_test(struct lx_chip *chip)
{
	u32 reg;

/*	printk(KERN_DEBUG  "%s\n", __func__);*/

	/* TEST if we have access to Xilinx/MicroBlaze */
	lx_dsp_reg_write(chip, eReg_CSM, 0);

	reg = lx_dsp_reg_read(chip, eReg_CSM);

	if (reg) {
		dev_err(chip->card->dev,
			"%s, Problem: Reg_CSM %x.\n", __func__, reg);

		/* PCI9056_SPACE0_REMAP */
		lx_plx_reg_write(chip, ePLX_PCICR, 1);

		reg = lx_dsp_reg_read(chip, eReg_CSM);
		if (reg) {
			dev_err(chip->card->dev,
				"%s, Error: Reg_CSM %x.\n",
				__func__, reg);
			return -EAGAIN; /* seems to be appropriate */
		}
	}

	dev_warn(chip->card->dev,
		"%s, Xilinx/MicroBlaze access test successful\n",
		__func__);

	return 0;
}

/* initialize ethersound */
int lx_init_ethersound_config(struct lx_chip *chip)
{
	int i;
	u32 orig_conf_es = lx_dsp_reg_read(chip, eReg_CONFES);

	/* configure 64 io channels */
	u32 conf_es = (orig_conf_es & CONFES_READ_PART_MASK)
			| (64 << IOCR_INPUTS_OFFSET)
			| (64 << IOCR_OUTPUTS_OFFSET)
			| (FREQ_RATIO_SINGLE_MODE << FREQ_RATIO_OFFSET);

/*	printk(KERN_DEBUG  "%s\n", __func__);*/

	chip->freq_ratio = FREQ_RATIO_SINGLE_MODE;

	/*
	 * write it to the card !
	 * this actually kicks the ES xilinx, the first time since poweron.
	 * the MAC address in the Reg_ADMACESMSB Reg_ADMACESLSB registers
	 * is not ready before this is done, and the bit 2 in Reg_CSES is set.
	 */
	lx_dsp_reg_write(chip, eReg_CONFES, conf_es);

	for (i = 0; i != 1000; ++i) {
		if (lx_dsp_reg_read(chip, eReg_CSES) & 4) {
/*			printk(KERN_DEBUG
*				"%s, ethersound initialized after %dms\n",
*				__func__,
*				i);
*/
			goto ethersound_initialized;
		}
		usleep_range(1000, 5000);
	}
	dev_warn(chip->card->dev,
	"%s, ethersound could not be initialized after %dms\n", __func__, i);
	return -ETIMEDOUT;

ethersound_initialized:
	dev_warn(chip->card->dev,
		"%s, LX initialized\n",
		__func__);
	return 0;
}

int lx_init_get_version_features(struct lx_chip *chip)
{
	u32 dsp_version;

	int err;

/*	printk(KERN_DEBUG  "%s\n", __func__);*/

	err = lx_dsp_get_version(chip, &dsp_version);

	if (err == 0) {
		u32 freq;

		dev_warn(chip->card->dev,
			"%s, DSP version: V%02d.%02d #%d\n", __func__,
			(dsp_version >> 16) & 0xff,
			(dsp_version >> 8) & 0xff, dsp_version & 0xff);

		/* later: what firmware version do we expect?
		*  retrieve Play/Rec features
		*  done here because we may have to handle alternate
		*  DSP files. later.
		*/

		/* init the EtherSound sample rate */
		err = lx_dsp_get_clock_frequency(chip, &freq);
		if (err == 0)
			chip->board_sample_rate = freq;

		dev_warn(chip->card->dev,
		"%s, actual clock frequency %d\n", __func__, freq);
	} else {
		dev_err(chip->card->dev,
			"%s, DSP corrupted\n", __func__);
		err = -EAGAIN;
	}

	return err;
}

/* initialize and test the xilinx dsp chip */
int lx_init_dsp(struct lx_chip *chip)
{
	int err;
	int i;

/*	printk(KERN_DEBUG  "%s\n", __func__);*/

	/*initialize board*/
	err = lx_init_xilinx_reset(chip);
	if (err) {
		dev_err(chip->card->dev,
			"%s, lx_init_xilinx_reset failed\n", __func__);
		return err;
	}

/*	printk(KERN_DEBUG  "%s, testing board\n", __func__);*/
	err = lx_init_xilinx_test(chip);
	if (err) {
		dev_err(chip->card->dev,
			"%s, lx_init_xilinx_test failed\n", __func__);
		return err;
	}

/*	printk(KERN_DEBUG  "%s, initialize ethersound configuration\n",
*			__func__);
*/
	err = lx_init_ethersound_config(chip);
	if (err) {
		dev_err(chip->card->dev,
		"%s, lx_init_ethersound_config failed\n", __func__);
		return err;
	}

	lx_irq_enable(chip);

	if (chip->lx_type == LX_ETHERSOUND) {

		/** \todo the mac address should be ready by not, but it isn't,
		 *  so we wait for it
		 */
		for (i = 0; i != 1000; ++i) {
			err = lx_dsp_get_mac(chip);
			if (err != 0) {
				dev_err(chip->card->dev,
				"%s, lx_dsp_get_mac failed\n", __func__);
				return err;
			}
			if (chip->mac_address[0] || chip->mac_address[1]
					|| chip->mac_address[2]
					|| chip->mac_address[3]
					|| chip->mac_address[4]
					|| chip->mac_address[5]) {
/*                        printk(KERN_DEBUG
*                        "%s, mac address ready read after: %d ms\n",
*                        __func__,
*                        i);
*/
				dev_warn(chip->card->dev,
			"%s, mac address: %02X.%02X.%02X.%02X.%02X.%02X\n",
					__func__, chip->mac_address[0],
					chip->mac_address[1],
					chip->mac_address[2],
					chip->mac_address[3],
					chip->mac_address[4],
					chip->mac_address[5]);

				goto mac_ready;
			}
			usleep_range(1000, 10000);
		}
		return -ETIMEDOUT;
	}
mac_ready:
	err = lx_init_get_version_features(chip);
	if (err != 0) {
		dev_err(chip->card->dev,
		"%s, lx_init_get_version_features failed\n", __func__);
		return err;
	}

	err = lx_set_granularity(chip, MICROBLAZE_IBL_DEFAULT);
	if (err != 0) {
		dev_err(chip->card->dev,
				"%s, lx_set_granularity failed\n", __func__);
	}

	chip->playback_mute = 0;

	return err;
}

static struct snd_pcm_ops lx_ops_playback = {
	.open      = lx_pcm_open,
	.close     = lx_pcm_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.prepare   = lx_pcm_prepare,
	.hw_params = lx_pcm_hw_params,
	.hw_free   = lx_pcm_hw_free,
	.trigger   = lx_pcm_trigger,
	.pointer   = lx_pcm_stream_pointer,
};

static struct snd_pcm_ops lx_ops_capture = {
	.open      = lx_pcm_open,
	.close     = lx_pcm_close,
	.ioctl     = snd_pcm_lib_ioctl,
	.prepare   = lx_pcm_prepare,
	.hw_params = lx_pcm_hw_params,
	.hw_free   = lx_pcm_hw_free,
	.trigger   = lx_pcm_trigger,
	.pointer   = lx_pcm_stream_pointer,
};



int lx_pcm_create(struct lx_chip *chip)
{
	int err;
	struct snd_pcm *pcm;
	u32 size;

	size =	MICROBLAZE_CHANNELS_MAX *	/* channels */
		3 *				/* 24 bit samples */
		MAX_STREAM_BUFFER *		/* periods */
		MICROBLAZE_IBL_MAX *		/* frames per period */
		2;				/* duplex */

	/*printk(KERN_DEBUG  "%s\n", __func__);*/
	size = PAGE_ALIGN(size);

	/* hardcoded device name & channel count */
	err = snd_pcm_new(chip->card, (char *)card_name, 0, 1, 1, &pcm);
	if (err < 0)
		return err;

	pcm->private_data = chip;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &lx_ops_playback);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &lx_ops_capture);

	pcm->info_flags = 0;
	/*SJR for 4.1*/
	/*pcm->nonatomic = true; TODO SJR check if use*/
	strcpy(pcm->name, card_name);

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

void lx_proc_levels_read(struct snd_info_entry *entry,
		struct snd_info_buffer *buffer)
{
	u32 levels[64];
	int err;
	int i, j;
	struct lx_chip *chip = entry->private_data;
	/*printk(KERN_DEBUG  "%s\n", __func__);*/

	snd_iprintf(buffer, "capture levels:\n");
	err = lx_level_peaks(chip, 1, 64, levels);
	if (err < 0)
		return;

	for (i = 0; i != 8; ++i) {
		for (j = 0; j != 8; ++j)
			snd_iprintf(buffer, "%08x ", levels[i * 8 + j]);
		snd_iprintf(buffer, "\n");
	}

	snd_iprintf(buffer, "\nplayback levels:\n");

	err = lx_level_peaks(chip, 0, 64, levels);
	if (err < 0)
		return;

	for (i = 0; i != 8; ++i) {
		for (j = 0; j != 8; ++j)
			snd_iprintf(buffer, "%08x ", levels[i * 8 + j]);
		snd_iprintf(buffer, "\n");
	}
	snd_iprintf(buffer, "\n");
}

int lx_proc_create(struct snd_card *card, struct lx_chip *chip)
{
	struct snd_info_entry *entry;
	int err = 0;
/*	still have bugs
*        err = snd_card_proc_new(card, "levels", &entry);
*        printk(KERN_DEBUG "%s\n", __func__);
*        if (err < 0) {
*                dev_err(chip->card->dev, "%s, snd_card_proc_new\n", __func__);
*                return err;
*        }
*
*        snd_info_set_text_ops(entry, chip, lx_proc_levels_read);
*/

	err = snd_card_proc_new(card, "Irqs", &entry);
	/*dev_err(chip->card->dev, "%s\n", __func__);*/
	if (err < 0) {
		dev_err(chip->card->dev,
			"%s, snd_card_proc_new Irqs\n",
			__func__);
		return err;
	}

	snd_info_set_text_ops(entry, chip, lx_proc_get_irq_counter);

	return 0;
}
