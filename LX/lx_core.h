/*
 *
 * ALSA driver for the digigram lx audio interface
 *
 * Copyright (c) 2016 Jubier Sylvain <alsa@digigram.com>
 */

#ifndef LX_CORE_H
#define LX_CORE_H

#include <linux/interrupt.h>
#include <sound/info.h>

#include "lx_defs.h"

#define REG_CRM_NUMBER                12

struct lx_chip;

/* low-level register access */

/* dsp register access */
enum {
	eReg_BASE,
	eReg_CSM,
	eReg_CRM1,
	eReg_CRM2,
	eReg_CRM3,
	eReg_CRM4,
	eReg_CRM5,
	eReg_CRM6,
	eReg_CRM7,
	eReg_CRM8,
	eReg_CRM9,
	eReg_CRM10,
	eReg_CRM11,
	eReg_CRM12,

	eReg_ICR,
	eReg_CVR,
	eReg_ISR,
	eReg_RXHTXH,
	eReg_RXMTXM,
	eReg_RHLTXL,
	eReg_RESETDSP,

	eReg_CSUF,
	eReg_CSES,
	eReg_CRESMSB,
	eReg_CRESLSB,
	eReg_ADMACESMSB,
	eReg_ADMACESLSB,
	eReg_CONFES,
	eReg_MADI_RAVENNA_CLOCK_CFG,

	eMaxPortLx
};

unsigned long lx_dsp_reg_read(struct lx_chip *chip, int port);
void lx_dsp_reg_write(struct lx_chip *chip, int port, unsigned data);

/* plx register access */
enum {
	ePLX_PCICR,

	ePLX_MBOX0,
	ePLX_MBOX1,
	ePLX_MBOX2,
	ePLX_MBOX3,
	ePLX_MBOX4,
	ePLX_MBOX5,
	ePLX_MBOX6,
	ePLX_MBOX7,

	ePLX_L2PCIDB,
	ePLX_IRQCS,
	ePLX_CHIPSC,

	eMaxPort
};

unsigned long lx_plx_reg_read(struct lx_chip *chip, int port);
void lx_plx_reg_write(struct lx_chip *chip, int port, u32 data);

/* rhm */
struct lx_rmh {
	u16 cmd_len; /* length of the command to send (WORDs) */
	u16 stat_len; /* length of the status received (WORDs) */
	u16 dsp_stat; /* status type, RMP_SSIZE_XXX */
	u16 cmd_idx; /* index of the command */
	u32 cmd[REG_CRM_NUMBER];
	u32 stat[REG_CRM_NUMBER];
};

/* low-level dsp access */
void lx_message_init(struct lx_rmh *rmh, enum cmd_mb_opcodes cmd);
int lx_message_send_atomic(struct lx_chip *chip, struct lx_rmh *rmh);
int lx_dsp_get_version(struct lx_chip *chip, u32 *rdsp_version);
int lx_dsp_get_clock_frequency(struct lx_chip *chip, u32 *rfreq);
int lx_dsp_set_granularity(struct lx_chip *chip, u32 gran);
int lx_dsp_read_async_events(struct lx_chip *chip, u32 *data);
int lx_dsp_get_mac(struct lx_chip *chip);

/* low-level pipe handling */
int lx_pipe_allocate(struct lx_chip *chip, u32 pipe, int is_capture,
		int channels);
int lx_pipe_release(struct lx_chip *chip, u32 pipe, int is_capture);
int lx_pipe_sample_count(struct lx_chip *chip, u32 pipe, int is_capture,
		u64 *rsample_count);
int lx_pipe_state(struct lx_chip *chip, u32 pipe, int is_capture, u16 *rstate);
int lx_pipe_stop_single(struct lx_chip *chip, u32 pipe, int is_capture);

/* start and pause could be merge... */
int lx_pipe_start_single(struct lx_chip *chip, u32 pipe, int is_capture);
int lx_pipe_pause_single(struct lx_chip *chip, u32 pipe, int is_capture);

int lx_pipe_start_multiple(struct lx_chip *chip);
int lx_pipe_pause_multiple(struct lx_chip *chip);

int lx_pipe_start_pause_play_and_record_dual(struct lx_chip *master_chip,
		struct lx_chip *slave_chip);

int lx_pipe_wait_for_start(struct lx_chip *chip, u32 pipe, int is_capture);
int lx_pipe_wait_for_idle(struct lx_chip *chip, u32 pipe, int is_capture);

/* low-level stream handling */
int lx_stream_def(struct lx_chip *chip, struct snd_pcm_runtime *runtime,
		u32 pipe, int is_capture);

int lx_stream_sample_position(struct lx_chip *chip, u32 pipe, int is_capture,
		u64 *r_bytepos);

int lx_stream_set_state(struct lx_chip *chip, u32 pipe, int is_capture,
		enum stream_state_t state);

int lx_madi_set_madi_state(struct lx_chip *chip);

static inline int lx_stream_start(struct lx_chip *chip, u32 pipe,
		int is_capture)
{
	return lx_stream_set_state(chip, pipe, is_capture, SSTATE_RUN);
}

static inline int lx_stream_pause(struct lx_chip *chip, u32 pipe,
		int is_capture)
{
	return lx_stream_set_state(chip, pipe, is_capture, SSTATE_PAUSE);
}

static inline int lx_stream_stop(struct lx_chip *chip, u32 pipe, int is_capture)
{
	snd_printdd("->lx_stream_stop\n");
	return lx_stream_set_state(chip, pipe, is_capture, SSTATE_STOP);
}

/* low-level buffer handling */
int lx_buffer_ask(struct lx_chip *chip, u32 pipe, int is_capture, u32 *r_needed,
		u32 *r_freed, u32 *size_array);
int lx_buffer_give(struct lx_chip *chip, u32 pipe, int is_capture,
		u32 buffer_size, u32 buf_address_lo, u32 buf_address_hi,
		u32 *r_buffer_index, unsigned char period_multiple_gran);

int lx_buffer_cancel(struct lx_chip *chip, u32 pipe, int is_capture,
		u32 buffer_index);

/* low-level gain/peak handling */
int lx_level_unmute(struct lx_chip *chip, int is_capture, int unmute);
int lx_level_peaks(struct lx_chip *chip, int is_capture, int channels,
		u32 *r_levels);

/* interrupt handling */
irqreturn_t lx_interrupt(int irq, void *dev_id);

void lx_irq_enable(struct lx_chip *chip);
void lx_irq_disable(struct lx_chip *chip);

/* debug */
void lx_proc_get_irq_counter(struct snd_info_entry *entry,
		struct snd_info_buffer *buffer);

/* Stream Format Header Defines (for LIN and IEEE754) */
#define HEADER_FMT_BASE         HEADER_FMT_BASE_LIN
#define HEADER_FMT_BASE_LIN     0xFED00000
#define HEADER_FMT_BASE_FLOAT   0xFAD00000
/* bit 23 in header_lo. WARNING: old bit 22 is ignored in float format */
#define HEADER_FMT_MONO         0x00000080
#define HEADER_FMT_INTEL        0x00008000
#define HEADER_FMT_16BITS       0x00002000
#define HEADER_FMT_24BITS       0x00004000
/* frequency is less or equ. to 11k*/
#define HEADER_FMT_UPTO11       0x00000200
/* frequency is over 11k and less then 32k.*/
#define HEADER_FMT_UPTO32       0x00000100

#define BIT_FMP_HEADER          23
#define BIT_FMP_SD              22
#define BIT_FMP_MULTICHANNEL    19

#define START_STATE             1
#define PAUSE_STATE             0

/* from PcxAll_e.h */
/* Start/Pause condition for pipes (PCXStartPipe, PCXPausePipe) */
#define START_PAUSE_IMMEDIATE           0
#define START_PAUSE_ON_SYNCHRO          1
#define START_PAUSE_ON_TIME_CODE        2

/* Pipe / Stream state */
#define START_STATE             1
#define PAUSE_STATE             0

static inline void unpack_pointer(dma_addr_t ptr, u32 *r_low, u32 *r_high)
{
	*r_low = (u32)(ptr & 0xffffffff);
#if BITS_PER_LONG == 32
	*r_high = 0;
#else
	*r_high = (u32)((u64)ptr >> 32);
#endif
}

void lx_wakeup_audio_thread(struct lx_chip *chip);

extern struct lx_chip *lx_chips_slave;
extern struct lx_chip *lx_chips_master;

int lx_interrupt_debug_events(struct lx_chip *chip);

#endif /* LX_CORE_H */
