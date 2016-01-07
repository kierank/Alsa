/*
 *
 * ALSA driver for the digigram lx audio interface
 *
 * Copyright (c) 2016 Jubier Sylvain <alsa@digigram.com>
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


/* #define RMH_DEBUG 1 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <linux/irqreturn.h>
#include <linux/freezer.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>

#include "lxcommon.h"
#include "lx_core.h"
static int indexcpu[SNDRV_CARDS] = {7, 6, 5};          /* Index 0-MAX */
//MODULE_AUTHOR("Sylvain Jubier");
//MODULE_LICENSE("GPL");
//MODULE_DESCRIPTION("digigram lxmadi");
//MODULE_SUPPORTED_DEVICE("{digigram lxmadi{}}");

module_param_array(indexcpu, int, NULL, 0444);
MODULE_PARM_DESC(indexcpu, "CPU affinity for irq.");

struct lx_chip* lx_chips_slave;
struct lx_chip* lx_chips_master;


DECLARE_WAIT_QUEUE_HEAD( thread_wait_queue );
/* low-level register access */

static const unsigned long dsp_port_offsets[] = {
        0,
        0x400,
        0x401,
        0x402,
        0x403,
        0x404,
        0x405,
        0x406,
        0x407,
        0x408,
        0x409,
        0x40a,
        0x40b,
        0x40c,

        0x410,
        0x411,
        0x412,
        0x413,
        0x414,
        0x415,
        0x416,

        0x420,
        0x430,
        0x431,
        0x432,
        0x433,
        0x434,
        0x440,
        0x500
};

static void __iomem *lx_dsp_register(struct lx_chip *chip, int port)
{
        void __iomem *base_address = chip->port_dsp_bar;
        return base_address + dsp_port_offsets[port]*4;
}

unsigned long lx_dsp_reg_read(struct lx_chip *chip, int port)
{
        void __iomem *address = lx_dsp_register(chip, port);
        return ioread32(address);
}

static void lx_dsp_reg_readbuf(struct lx_chip *chip, int port, u32 *data,
                               u32 len)
{
        u32 __iomem *address = lx_dsp_register(chip, port);
        int i;

        /* we cannot use memcpy_fromio */
        for (i = 0; i != len; ++i) {
                data[i] = ioread32(address + i);
        }
}


void lx_dsp_reg_write(struct lx_chip *chip, int port, unsigned data)
{
        void __iomem *address = lx_dsp_register(chip, port);
        iowrite32(data, address);
}

static void lx_dsp_reg_writebuf(struct lx_chip *chip, int port,
                                const u32 *data, u32 len)
{
        u32 __iomem *address = lx_dsp_register(chip, port);
        int i;

        /* we cannot use memcpy_to */
        for (i = 0; i != len; ++i) {
                iowrite32(data[i], address + i);
        }
}


static const unsigned long plx_port_offsets[] = {
        0x04,
        0x40,
        0x44,
        0x48,
        0x4c,
        0x50,
        0x54,
        0x58,
        0x5c,
        0x64,
        0x68,
        0x6C
};

static void __iomem *lx_plx_register(struct lx_chip *chip, int port)
{
        void __iomem *base_address = chip->port_plx_remapped;
        return base_address + plx_port_offsets[port];
}

unsigned long lx_plx_reg_read(struct lx_chip *chip, int port)
{
        void __iomem *address = lx_plx_register(chip, port);
        return ioread32(address);
}

void lx_plx_reg_write(struct lx_chip *chip, int port, u32 data)
{
        void __iomem *address = lx_plx_register(chip, port);
        iowrite32(data, address);
}

/* rmh */

#ifdef CONFIG_SND_DEBUG
#define CMD_NAME(a) a
#else
#define CMD_NAME(a) NULL
#endif

#define Reg_CSM_MR                        0x00000002
#define Reg_CSM_MC                        0x00000001

struct dsp_cmd_info {
        u32    dcCodeOp;        /* Op Code of the command (usually 1st 24-bits
                                 * word).*/
        u16    dcCmdLength;     /* Command length in words of 24 bits.*/
        u16    dcStatusType;    /* Status type: 0 for fixed length, 1 for
                                 * random. */
        u16    dcStatusLength;  /* Status length (if fixed).*/
        char  *dcOpName;
};

/*
  Initialization and control data for the Microblaze interface
  - OpCode:
    the opcode field of the command set at the proper offset
  - CmdLength
    the number of command words
  - StatusType
    offset in the status registers: 0 means that the return value may be
    different from 0, and must be read
  - StatusLength
    the number of status words (in addition to the return value)
*/

static struct dsp_cmd_info dsp_commands[] =
{
        { (CMD_00_INFO_DEBUG << OPCODE_OFFSET),
            1 /*custom*/,
            1,
            0,
            CMD_NAME("INFO_DEBUG") },
        { (CMD_01_GET_SYS_CFG << OPCODE_OFFSET),
            1,
            1,
            2,
            CMD_NAME("GET_SYS_CFG") },
        { (CMD_02_SET_GRANULARITY << OPCODE_OFFSET),
            1,
            1,
            0,
            CMD_NAME("SET_GRANULARITY") },
        { (CMD_03_SET_TIMER_IRQ << OPCODE_OFFSET),
            1,
            1,
            0,
            CMD_NAME("SET_TIMER_IRQ") },
        { (CMD_04_GET_EVENT << OPCODE_OFFSET),
            1,
            1,
            0 /*up to 10*/,
            CMD_NAME("GET_EVENT") },
        { (CMD_05_GET_PIPES << OPCODE_OFFSET),
            1,
            1,
            2 /*up to 4*/,
            CMD_NAME("GET_PIPES") },
        { (CMD_06_ALLOCATE_PIPE << OPCODE_OFFSET),
            1,
            0,
            0,
            CMD_NAME("ALLOCATE_PIPE") },
        { (CMD_07_RELEASE_PIPE << OPCODE_OFFSET),
            1,
            0,
            0,
            CMD_NAME("RELEASE_PIPE") },
        { (CMD_08_ASK_BUFFERS << OPCODE_OFFSET),
            1,
            1,
            MAX_STREAM_BUFFER,
            CMD_NAME("ASK_BUFFERS") },
        { (CMD_09_STOP_PIPE << OPCODE_OFFSET),
            1,
            0,
            0 /*up to 2*/,
            CMD_NAME("STOP_PIPE") },
        { (CMD_0A_GET_PIPE_SPL_COUNT << OPCODE_OFFSET),
            1,
            1,
            1 /*up to 2*/,
            CMD_NAME("GET_PIPE_SPL_COUNT") },
        { (CMD_0B_TOGGLE_PIPE_STATE << OPCODE_OFFSET),
            1 /*up to 5*/,
            1,
            0,
            CMD_NAME("TOGGLE_PIPE_STATE") },
        { (CMD_0C_DEF_STREAM << OPCODE_OFFSET),
            1 /*up to 4*/,
            1,
            0,
            CMD_NAME("DEF_STREAM") },
        { (CMD_0D_SET_MUTE  << OPCODE_OFFSET),
            3,
            1,
            0,
            CMD_NAME("SET_MUTE") },
        { (CMD_0E_GET_STREAM_SPL_COUNT << OPCODE_OFFSET),
            1,
            1,
            2,
            CMD_NAME("GET_STREAM_SPL_COUNT") },
        { (CMD_0F_UPDATE_BUFFER << OPCODE_OFFSET),
            3 /*up to 4*/,
            0,
            1,
            CMD_NAME("UPDATE_BUFFER") },
        { (CMD_10_GET_BUFFER << OPCODE_OFFSET),
            1,
            1,
            4,
            CMD_NAME("GET_BUFFER") },
        { (CMD_11_CANCEL_BUFFER << OPCODE_OFFSET),
            1,
            1,
            1 /*up to 4*/,
            CMD_NAME("CANCEL_BUFFER") },
        { (CMD_12_GET_PEAK << OPCODE_OFFSET),
            1,
            1,
            1,
            CMD_NAME("GET_PEAK") },
        { (CMD_13_SET_STREAM_STATE << OPCODE_OFFSET),
            1,
            1,
            0,
            CMD_NAME("SET_STREAM_STATE") },
        { (CMD_14_GET_MADI_STATE << OPCODE_OFFSET),
            1,
            1,
            1,
            CMD_NAME("GET_MADI_STATE") },
        { (CMD_15_SET_MADI_STATE << OPCODE_OFFSET),
            2,
            0,
            0,
            CMD_NAME("SET_MADI_STATE") },
};

void lx_message_init(struct lx_rmh *rmh, enum cmd_mb_opcodes cmd)
{
        snd_BUG_ON(cmd >= CMD_INVALID);

        rmh->cmd[0]   = dsp_commands[cmd].dcCodeOp;
        rmh->cmd_len  = dsp_commands[cmd].dcCmdLength;
        rmh->stat_len = dsp_commands[cmd].dcStatusLength;
        rmh->dsp_stat = dsp_commands[cmd].dcStatusType;
        rmh->cmd_idx  = cmd;
        memset(&rmh->cmd[1], 0, (REG_CRM_NUMBER - 1) * sizeof(u32));

#ifdef CONFIG_SND_DEBUG
        memset(rmh->stat, 0, REG_CRM_NUMBER * sizeof(u32));
#endif
#ifdef RMH_DEBUG
        rmh->cmd_idx = cmd;
#endif
}
//#define RMH_DEBUG
#ifdef RMH_DEBUG
#define LXRMH "lx_chip rmh: "
static void lx_message_dump(struct lx_rmh *rmh)
{
        u8 idx = rmh->cmd_idx;
        int i;

        snd_printk(LXRMH "command %s\n", dsp_commands[idx].dcOpName);

        for (i = 0; i != rmh->cmd_len; ++i)
                snd_printk(LXRMH "\tcmd[%d] %08x\n", i, rmh->cmd[i]);

        for (i = 0; i != rmh->stat_len; ++i)
                snd_printk(LXRMH "\tstat[%d]: %08x\n", i, rmh->stat[i]);
        snd_printk("\n");
}
#else
static inline void lx_message_dump(struct lx_rmh *rmh)
{}
#endif



/* sleep 500 - 100 = 400 times 100us -> the timeout is >= 40 ms */
#define XILINX_TIMEOUT_MS       40
#define XILINX_POLL_NO_SLEEP    100
#define XILINX_POLL_ITERATIONS  150

int lx_message_send_atomic(struct lx_chip *chip, struct lx_rmh *rmh)
{
        u32 reg = ED_DSP_TIMED_OUT;
        int dwloop;

        if (lx_dsp_reg_read(chip, eReg_CSM) & (Reg_CSM_MC | Reg_CSM_MR)) {
                dev_err(chip->card->dev, "PIOSendMessage eReg_CSM %x\n", reg);
                return -EBUSY;
        }

        /* write command */
        lx_dsp_reg_writebuf(chip, eReg_CRM1, rmh->cmd, rmh->cmd_len);
        atomic_set(&chip->message_pending, 1);
        /* MicroBlaze gogogo */
        lx_dsp_reg_write(chip, eReg_CSM, Reg_CSM_MC);

        /* wait for device to answer */
        dwloop = 0;
        while ((atomic_read(&chip->message_pending) == 1) && (dwloop++ < 40000)){
                udelay(1);
        }

        if (atomic_read(&chip->message_pending)) {
                printk(KERN_ERR "%s, message_pending timeout...\n", __func__);
        }

        if (lx_dsp_reg_read(chip, eReg_CSM) & Reg_CSM_MR) {
                if (rmh->dsp_stat == 0) {
                        reg = lx_dsp_reg_read(chip, eReg_CRM1);
                }
                else {
                        reg = 0;
                }
                goto polling_successful;
        }

        dev_warn(chip->card->dev, "TIMEOUT lx_message_send_atomic! "
                   "polling failed\n");

polling_successful:
        if ((reg & ERROR_VALUE) == 0) {
                /* read response */
                if (rmh->stat_len) {
                        snd_BUG_ON(rmh->stat_len >= (REG_CRM_NUMBER-1));
                        lx_dsp_reg_readbuf(chip, eReg_CRM2, rmh->stat,
                                           rmh->stat_len);
                }
        } else {
                dev_err(chip->card->dev, "rmh error: %08x\n", reg);
        }

        /* clear Reg_CSM_MR */
        lx_dsp_reg_write(chip, eReg_CSM, 0);

        switch (reg) {
        case ED_DSP_TIMED_OUT:
                dev_warn(chip->card->dev, "lx_message_send: dsp timeout\n");
                return -ETIMEDOUT;

        case ED_DSP_CRASHED:
                dev_warn(chip->card->dev, "lx_message_send: dsp crashed\n");
                return -EAGAIN;
        }
//        lx_message_dump(rmh);

        return reg;
}

//for 1st commands we don t have interrupt
int lx_message_send_atomic_poll(struct lx_chip *chip, struct lx_rmh *rmh)
{
        u32 reg = ED_DSP_TIMED_OUT;
        int dwloop;
        if (lx_dsp_reg_read(chip, eReg_CSM) & (Reg_CSM_MC | Reg_CSM_MR)) {
                dev_err(chip->card->dev, "PIOSendMessage eReg_CSM %x\n", reg);
                return -EBUSY;
        }

        /* write command */
        lx_dsp_reg_writebuf(chip, eReg_CRM1, rmh->cmd, rmh->cmd_len);

        /* MicroBlaze gogogo */
        lx_dsp_reg_write(chip, eReg_CSM, Reg_CSM_MC);

        /* wait for device to answer */
         for (dwloop = 0; dwloop != XILINX_TIMEOUT_MS * 1000; ++dwloop) {
                 if (lx_dsp_reg_read(chip, eReg_CSM) & Reg_CSM_MR) {
                                if (rmh->dsp_stat == 0) {
                                        reg = lx_dsp_reg_read(chip, eReg_CRM1);
                                }
                                else {
                                        reg = 0;
                                }
                                goto polling_successful;
                 }
                 udelay(1);
         }

        dev_warn(chip->card->dev, "TIMEOUT lx_message_send_atomic_poll! "
                   "polling failed\n");

polling_successful:
        if ((reg & ERROR_VALUE) == 0) {
                /* read response */
                if (rmh->stat_len) {
                        snd_BUG_ON(rmh->stat_len >= (REG_CRM_NUMBER-1));
                        lx_dsp_reg_readbuf(chip, eReg_CRM2, rmh->stat,
                                           rmh->stat_len);
                }
        } else {
                dev_err(chip->card->dev, "rmh error: %08x\n", reg);
        }

        /* clear Reg_CSM_MR */
        lx_dsp_reg_write(chip, eReg_CSM, 0);

        switch (reg) {
        case ED_DSP_TIMED_OUT:
                dev_warn(chip->card->dev, "lx_message_send: dsp timeout\n");
                return -ETIMEDOUT;

        case ED_DSP_CRASHED:
                dev_warn(chip->card->dev, "lx_message_send: dsp crashed\n");
                return -EAGAIN;
        }

//        lx_message_dump(rmh);

        return reg;
}

/* low-level dsp access */
int lx_dsp_get_version(struct lx_chip *chip, u32 *rdsp_version)
{
        u16 ret;

        mutex_lock(&chip->msg_lock);

        lx_message_init(&chip->rmh, CMD_01_GET_SYS_CFG);
        ret = lx_message_send_atomic(chip, &chip->rmh);

        *rdsp_version = chip->rmh.stat[1];
        mutex_unlock(&chip->msg_lock);

        return ret;
}

int lx_dsp_get_clock_frequency(struct lx_chip *chip, u32 *rfreq)
{
        u16 ret = 0;
        u32 freq_raw = 0;
        u32 freq = 0;
        u32 frequency = 0;

        mutex_lock(&chip->msg_lock);

        lx_message_init(&chip->rmh, CMD_01_GET_SYS_CFG);
        ret = lx_message_send_atomic(chip, &chip->rmh);

        if (ret == 0) {
                freq_raw = chip->rmh.stat[0] >> FREQ_FIELD_OFFSET;
                freq = freq_raw & XES_FREQ_COUNT8_MASK;

                if ((freq < XES_FREQ_COUNT8_48_MAX) ||
                    (freq > XES_FREQ_COUNT8_44_MIN) ) {
                        frequency = 0; /* unknown */
                }
                else if (freq >= XES_FREQ_COUNT8_44_MAX) {
                        frequency = 44100;
                }
                else {
                        frequency = 48000;
                }
        }

        mutex_unlock(&chip->msg_lock);

        *rfreq = frequency * chip->freq_ratio;

        return ret;
}

int lx_dsp_get_mac(struct lx_chip *chip)
{
        u32 macmsb, maclsb;

        macmsb = lx_dsp_reg_read(chip, eReg_ADMACESMSB) & 0x00FFFFFF;
        maclsb = lx_dsp_reg_read(chip, eReg_ADMACESLSB) & 0x00FFFFFF;

        /* todo: endianess handling */
        chip->mac_address[5] = ((u8 *)(&maclsb))[0];
        chip->mac_address[4] = ((u8 *)(&maclsb))[1];
        chip->mac_address[3] = ((u8 *)(&maclsb))[2];
        chip->mac_address[2] = ((u8 *)(&macmsb))[0];
        chip->mac_address[1] = ((u8 *)(&macmsb))[1];
        chip->mac_address[0] = ((u8 *)(&macmsb))[2];

        return 0;
}


int lx_dsp_set_granularity(struct lx_chip *chip, u32 gran)
{
        int ret;

        mutex_lock(&chip->msg_lock);

        lx_message_init(&chip->rmh, CMD_02_SET_GRANULARITY);
        chip->rmh.cmd[0] |= gran;

        ret = lx_message_send_atomic(chip, &chip->rmh);
        mutex_unlock(&chip->msg_lock);
        return ret;
}

int lx_dsp_read_async_events(struct lx_chip *chip, u32 *data)
{
        int ret;

        mutex_lock(&chip->msg_lock);

        lx_message_init(&chip->rmh, CMD_04_GET_EVENT);
        /* we don't necessarily need the full length */
        chip->rmh.stat_len = 10;

        ret = lx_message_send_atomic_poll(chip, &chip->rmh);

        if (!ret) {
                memcpy(data, chip->rmh.stat, chip->rmh.stat_len * sizeof(u32));
        }

        mutex_unlock(&chip->msg_lock);
        return ret;
}

#define PIPE_INFO_TO_CMD(capture, pipe)\
        ((u32)((u32)(pipe) | ((capture) ? ID_IS_CAPTURE : 0L)) << ID_OFFSET)



/* low-level pipe handling */
int lx_pipe_allocate(struct lx_chip *chip, u32 pipe, int is_capture,
                     int channels)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_06_ALLOCATE_PIPE);

        chip->rmh.cmd[0] |= pipe_cmd;
        chip->rmh.cmd[0] |= channels;

        err = lx_message_send_atomic(chip, &chip->rmh);
        mutex_unlock(&chip->msg_lock);

        if (err != 0) {
                dev_err(chip->card->dev, "could not allocate pipe\n");
        }

        return err;
}

int lx_pipe_release(struct lx_chip *chip, u32 pipe, int is_capture)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_07_RELEASE_PIPE);

        chip->rmh.cmd[0] |= pipe_cmd;

        err = lx_message_send_atomic(chip, &chip->rmh);
        mutex_unlock(&chip->msg_lock);

        return err;
}

int lx_buffer_ask(struct lx_chip *chip, u32 pipe, int is_capture,
                  u32 *r_needed, u32 *r_freed, u32 *size_array)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

#ifdef CONFIG_SND_DEBUG
        if (size_array)
                memset(size_array, 0, sizeof(u32)*MAX_STREAM_BUFFER);
#endif

        *r_needed = 0;
        *r_freed = 0;

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_08_ASK_BUFFERS);

        chip->rmh.cmd[0] |= pipe_cmd;

        err = lx_message_send_atomic(chip, &chip->rmh);

        if (!err) {
                int i;
                for (i = 0; i < MAX_STREAM_BUFFER; ++i) {
                        u32 stat = chip->rmh.stat[i];
                        if (stat & (BF_EOB << BUFF_FLAGS_OFFSET)) {
                                /* finished */
                                *r_freed += 1;
                                if (size_array)
                                        size_array[i] = stat & MASK_DATA_SIZE;
                        } else if ((stat & (BF_VALID << BUFF_FLAGS_OFFSET))
                                   == 0)
                                /* free */
                                *r_needed += 1;
                }
#ifdef CONFIG_SND_DEBUG

                dev_dbg(chip->card->dev,
                        "CMD_08_ASK_BUFFERS: needed %d, freed %d\n",
                            *r_needed, *r_freed);
                for (i = 0; i < MAX_STREAM_BUFFER; ++i) {
                        for (i = 0; i != chip->rmh.stat_len; ++i)
                                dev_dbg(chip->card->dev,
                                        "  stat[%d]: %x, %x\n", i,
                                            chip->rmh.stat[i],
                                            chip->rmh.stat[i] & MASK_DATA_SIZE);
                }
#endif
        }
        else {
                dev_err(chip->card->dev, "lx_buffer_ask failed\n");
        }

        mutex_unlock(&chip->msg_lock);
        return err;
}


int lx_pipe_stop_single(struct lx_chip *chip, u32 pipe, int is_capture)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);
        //printk(KERN_DEBUG "\t\t%s, is_capture: %d\n", __func__, is_capture);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_09_STOP_PIPE);

        chip->rmh.cmd[0] |= pipe_cmd;

        err = lx_message_send_atomic(chip, &chip->rmh);

        mutex_unlock(&chip->msg_lock);
        return err;
}

static int lx_pipe_toggle_state(struct lx_chip *chip, u32 pipe, int is_capture)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);
        //printk(KERN_DEBUG "\t\t%s, is_capture: %d\n", __func__, is_capture);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_0B_TOGGLE_PIPE_STATE);

        chip->rmh.cmd[0] |= pipe_cmd;

        err = lx_message_send_atomic_poll(chip, &chip->rmh);

        mutex_unlock(&chip->msg_lock);
        return err;
}

static int lx_pipe_toggle_state_play_and_record(struct lx_chip *chip)
{
        int err;
        u32 pipe_cmd = 1<<12;

        mutex_lock(&chip->msg_lock);

        lx_message_init(&chip->rmh, CMD_0B_TOGGLE_PIPE_STATE);
        /* in this case we have to specify pipes mask for play and record */
        chip->rmh.cmd_len = 5;
        chip->rmh.cmd[0] |= pipe_cmd;
        chip->rmh.cmd[1] = 0;
        chip->rmh.cmd[2] = 1;
        chip->rmh.cmd[3] = 0;
        chip->rmh.cmd[4] = 1;

        err = lx_message_send_atomic_poll(chip, &chip->rmh);

        mutex_unlock(&chip->msg_lock);
        return err;
}

static int lx_pipe_toggle_state_play_and_record_dual(
        struct lx_chip *master_chip, struct lx_chip *slave_chip)
{
        int err;
        u32 pipe_cmd = 1<<12;
//printk(KERN_DEBUG "\t\t\t%s %p %p\n", __func__, master_chip, slave_chip);

        mutex_lock(&master_chip->msg_lock);
        lx_message_init(&master_chip->rmh, CMD_0B_TOGGLE_PIPE_STATE);
        /* in this case we have to specify pipes mask for play and record */
        master_chip->rmh.cmd_len = 5;
        master_chip->rmh.cmd[0] |= pipe_cmd;
        master_chip->rmh.cmd[1] = 0;
        master_chip->rmh.cmd[2] = 1;
        master_chip->rmh.cmd[3] = 0;
        master_chip->rmh.cmd[4] = 1;


        lx_message_init(&slave_chip->rmh, CMD_0B_TOGGLE_PIPE_STATE);
        /* in this case we have to specify pipes mask for play and record */
        slave_chip->rmh.cmd_len = 5;
        slave_chip->rmh.cmd[0] |= pipe_cmd;
        slave_chip->rmh.cmd[1] = 0;
        slave_chip->rmh.cmd[2] = 1;
        slave_chip->rmh.cmd[3] = 0;
        slave_chip->rmh.cmd[4] = 1;

        err = lx_message_send_atomic(master_chip, &master_chip->rmh);

        err += lx_message_send_atomic(slave_chip, &slave_chip->rmh);

        mutex_unlock(&master_chip->msg_lock);
        return err;
}

int lx_pipe_start_single(struct lx_chip *chip, u32 pipe, int is_capture)
{
        int err;

        err = lx_pipe_toggle_state(chip, pipe, is_capture);
        if (err < 0) {
                dev_err(chip->card->dev, "%s: lx_pipe_toggle_state failed\n",
                                __func__);
                return err;
        }

        return err;
}

int lx_pipe_pause_single(struct lx_chip *chip, u32 pipe, int is_capture)
{
        int err = 0;
        //printk(KERN_DEBUG "\t%s, is_capture: %d\n", __func__, is_capture);

        err = lx_pipe_wait_for_start(chip, pipe, is_capture);
        if (err < 0) {
                printk(KERN_ERR "\t\t%s error wait for start err %d,"
                                " is_capture %d\n",__func__, err, is_capture);
                return err;
        }

        err = lx_pipe_toggle_state(chip, pipe, is_capture);
        if (err < 0) {
                printk(KERN_ERR "\t\t%s error lx_pipe_toggle_state err %d,"
                                " is_capture %d\n", __func__, err, is_capture);
                return err;
        }

        lx_pipe_wait_for_idle(chip, pipe, is_capture);
        if (err < 0) {
                printk(KERN_ERR "\t\t%s error wait for idle err %d,"
                                " is_capture %d\n", __func__, err, is_capture);
                return err;
        }

        return err;
}

int lx_pipe_start_multiple(struct lx_chip *chip)
{
        int err;

        err = lx_pipe_toggle_state_play_and_record(chip);
        if (err < 0) {
                dev_err(chip->card->dev, "%s: lx_pipe_toggle_state failed\n",
                                __func__);
        }
        return err;
}
int lx_pipe_pause_multiple(struct lx_chip *chip)
{
        int err;

        err = lx_pipe_wait_for_start(chip, 0, 0);
        if (err < 0) {
                dev_err(chip->card->dev, "%s: lx_pipe_toggle_state failed\n",
                                __func__);
                return err;
        }

        err = lx_pipe_wait_for_start(chip, 0, 1);
        if (err < 0) {
                dev_err(chip->card->dev, "%s: lx_pipe_toggle_state failed\n",
                                __func__);
                return err;
        }

        err = lx_pipe_toggle_state_play_and_record(chip);
        if (err < 0) {
                dev_err(chip->card->dev, "%s: lx_pipe_toggle_state failed\n",
                                __func__);
        }

        err = lx_pipe_wait_for_idle(chip, 0, 0);
        if (err < 0) {
                printk(KERN_ERR "\t\t%s error wait for play idle %d\n",
                                __func__, err);
                return err;
        }

        err = lx_pipe_wait_for_idle(chip, 0, 1);
        if (err < 0) {
                printk(KERN_ERR "\t\t%s error wait for capture idle %d\n",
                                __func__, err);
                return err;
        }

        return err;
}
int lx_pipe_start_pause_play_and_record_dual(
                struct lx_chip *master_chip, struct lx_chip *slave_chip)
{
        int err;
//printk(KERN_DEBUG "\t\t%s\n", __func__);

        err = lx_pipe_toggle_state_play_and_record_dual(master_chip,
                                                        slave_chip);
        if (err < 0) {
                dev_err(master_chip->card->dev,"%s: failed\n", __func__);
                dev_err(slave_chip->card->dev, "%s: failed\n", __func__);
        }

        return err;
}

int lx_pipe_sample_count(struct lx_chip *chip, u32 pipe, int is_capture,
                         u64 *rsample_count)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_0A_GET_PIPE_SPL_COUNT);

        chip->rmh.cmd[0] |= pipe_cmd;
        chip->rmh.stat_len = 2;        /* need all words here! */

        err = lx_message_send_atomic(chip, &chip->rmh);

        if (err != 0)
                dev_err(chip->card->dev,
                        "could not query pipe's sample count\n");
        else {
                *rsample_count = ((u64)(chip->rmh.stat[0] & MASK_SPL_COUNT_HI)
                                  << 24)     /* hi part */
                        + chip->rmh.stat[1]; /* lo part */
        }

        mutex_unlock(&chip->msg_lock);
        return err;
}

int lx_pipe_state(struct lx_chip *chip, u32 pipe, int is_capture, u16 *rstate)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

        mutex_lock(&chip->msg_lock);

        lx_message_init(&chip->rmh, CMD_0A_GET_PIPE_SPL_COUNT);
        chip->rmh.cmd[0] |= pipe_cmd;
        err = lx_message_send_atomic_poll(chip, &chip->rmh);

        if (err != 0) {
                dev_err(chip->card->dev, "could not query pipe's state\n");
        }
        else {
                *rstate = (chip->rmh.stat[0] >> PSTATE_OFFSET) & 0x0F;
        }
        mutex_unlock(&chip->msg_lock);

        return err;
}

static int lx_pipe_wait_for_state(struct lx_chip *chip, u32 pipe,
                                  int is_capture, u16 state)
{
        int i;
        u16 current_state;

        /* max 2*PCMOnlyGranularity = 2*1024 at 44100 = < 50 ms:
         * timeout 50 ms */
        for (i = 0; i < 100; ++i) {
                int err = lx_pipe_state(chip, pipe, is_capture, &current_state);

                if (err < 0) {
                        printk(KERN_ERR"\t\t\t %s : lx_pipe_state failed %d\n",
                                        __func__, err);
                        return err;
                }

                if (current_state == state) {
//printk(KERN_DEBUG"\t\t\t %s : %d ms\n", __func__,i);
                        return 0;
                }
                mdelay(1);
        }

        if (current_state != state) {
                printk(KERN_ERR"\t\t\t%s failed.... current state is %d\n",
                                __func__,current_state);
        }
        return -ETIMEDOUT;
}

int lx_pipe_wait_for_start(struct lx_chip *chip, u32 pipe, int is_capture)
{
//printk(KERN_DEBUG"\t%s\n", __func__);
        return lx_pipe_wait_for_state(chip, pipe, is_capture, PSTATE_RUN);
}

int lx_pipe_wait_for_idle(struct lx_chip *chip, u32 pipe, int is_capture)
{
//printk(KERN_DEBUG"\t%s\n", __func__);
        return lx_pipe_wait_for_state(chip, pipe, is_capture, PSTATE_IDLE);
}

/* low-level stream handling */
int lx_stream_set_state(struct lx_chip *chip, u32 pipe,
                        int is_capture, enum stream_state_t state)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_13_SET_STREAM_STATE);

        chip->rmh.cmd[0] |= pipe_cmd;
        chip->rmh.cmd[0] |= state;

        err = lx_message_send_atomic(chip, &chip->rmh);
        if (err != 0) {
                printk(KERN_ERR "%s->lx_message_send_atomic failed...\n",
                                __func__);
        }
        mutex_unlock(&chip->msg_lock);

        return err;
}

int lx_stream_def(struct lx_chip *chip, struct snd_pcm_runtime *runtime,
                         u32 pipe, int is_capture)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);
        u32 channels = runtime->channels;

        if (runtime->channels != channels)
                dev_err(chip->card->dev, "channel count mismatch: %d vs %d",
                           runtime->channels, channels);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_0C_DEF_STREAM);

        chip->rmh.cmd[0] |= pipe_cmd;

        chip->rmh.cmd[0] |= MASK_STREAM_IS_ALSA;

// not use now...
//        if (runtime->sample_bits == 16)
//                /* 16 bit format */
//                chip->rmh.cmd[0] |= (STREAM_FMT_16b << STREAM_FMT_OFFSET);

        if (snd_pcm_format_little_endian(runtime->format))
                /* little endian/intel format */
                chip->rmh.cmd[0] |= (STREAM_FMT_intel << STREAM_FMT_OFFSET);

        chip->rmh.cmd[0] |= channels-1;

        err = lx_message_send_atomic(chip, &chip->rmh);
        mutex_unlock(&chip->msg_lock);

        return err;
}

int lx_stream_state(struct lx_chip *chip, u32 pipe, int is_capture,
                int *rstate)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_0E_GET_STREAM_SPL_COUNT);

        chip->rmh.cmd[0] |= pipe_cmd;

        err = lx_message_send_atomic(chip, &chip->rmh);

        *rstate = (chip->rmh.stat[0] & SF_START) ? START_STATE : PAUSE_STATE;

        mutex_unlock(&chip->msg_lock);
        return err;
}

int lx_stream_sample_position(struct lx_chip *chip, u32 pipe, int is_capture,
                              u64 *r_bytepos)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_0E_GET_STREAM_SPL_COUNT);

        chip->rmh.cmd[0] |= pipe_cmd;

        err = lx_message_send_atomic(chip, &chip->rmh);

        *r_bytepos =
                        ((u64) (chip->rmh.stat[0] & MASK_SPL_COUNT_HI) << 32) +
                                chip->rmh.stat[1];

        mutex_unlock(&chip->msg_lock);
        return err;
}

/* low-level buffer handling */
int lx_buffer_give(struct lx_chip *chip, u32 pipe, int is_capture,
                   u32 buffer_size, u32 buf_address_lo, u32 buf_address_hi,
                   u32 *r_buffer_index, unsigned char period_multiple_gran)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);
//printk(KERN_DEBUG "%s, period_multiple_gran %d\n", __func__,
//        period_multiple_gran);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_0F_UPDATE_BUFFER);

        chip->rmh.cmd[0] |= pipe_cmd;
        /* request interrupt notification */
        chip->rmh.cmd[0] |= BF_NOTIFY_EOB | BF_CIRCULAR;

        chip->rmh.cmd[1] =
                    (buffer_size & MASK_DATA_SIZE) |
                    ((u32)period_multiple_gran << PERIOD_MULTIPLE_GRAN_OFFSET);
//printk(KERN_DEBUG "%s %x\n", __func__, chip->rmh.cmd[1]);
        chip->rmh.cmd[2] = buf_address_lo;

        if (buf_address_hi) {
                chip->rmh.cmd_len = 4;
                chip->rmh.cmd[3] = buf_address_hi;
                chip->rmh.cmd[0] |= BF_64BITS_ADR;
        }

        err = lx_message_send_atomic(chip, &chip->rmh);

        if (err == 0) {
                *r_buffer_index = chip->rmh.stat[0];
                goto done;
        }

        if (err == EB_RBUFFERS_TABLE_OVERFLOW)
                dev_err(chip->card->dev,
                        "lx_buffer_give EB_RBUFFERS_TABLE_OVERFLOW\n");

        if (err == EB_INVALID_STREAM)
                dev_err(chip->card->dev,
                        "lx_buffer_give EB_INVALID_STREAM\n");

        if (err == EB_CMD_REFUSED)
                dev_err(chip->card->dev,
                        "lx_buffer_give EB_CMD_REFUSED\n");

 done:
        mutex_unlock(&chip->msg_lock);
        return err;
}

int lx_buffer_cancel(struct lx_chip *chip, u32 pipe, int is_capture,
                     u32 buffer_index)
{
        int err;
        u32 pipe_cmd = PIPE_INFO_TO_CMD(is_capture, pipe);
//printk(KERN_DEBUG "%s\n", __func__);

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_11_CANCEL_BUFFER);

        chip->rmh.cmd[0] |= pipe_cmd;
        chip->rmh.cmd[0] |= buffer_index;

        err = lx_message_send_atomic(chip, &chip->rmh);
        mutex_unlock(&chip->msg_lock);

        return err;
}


/* low-level gain/peak handling
 *
 * \todo: can we unmute capture/playback channels independently?
 *
 * */
int lx_level_unmute(struct lx_chip *chip, int is_capture, int unmute)
{
        int err;
        /* bit set to 1: channel muted */
        u64 mute_mask = unmute ? 0 : 0xFFFFFFFFFFFFFFFFLLU;

        mutex_lock(&chip->msg_lock);
        lx_message_init(&chip->rmh, CMD_0D_SET_MUTE);

        chip->rmh.cmd[0] |= PIPE_INFO_TO_CMD(is_capture, 0);

        chip->rmh.cmd[1] = (u32)(mute_mask >> (u64)32);        /* hi part */
        chip->rmh.cmd[2] = (u32)(mute_mask & (u64)0xFFFFFFFF); /* lo part */

        err = lx_message_send_atomic(chip, &chip->rmh);

        mutex_unlock(&chip->msg_lock);
        return err;
}

static u32 peak_map[] = {
        0x00000109, /* -90.308dB */
        0x0000083B, /* -72.247dB */
        0x000020C4, /* -60.205dB */
        0x00008273, /* -48.030dB */
        0x00020756, /* -36.005dB */
        0x00040C37, /* -30.001dB */
        0x00081385, /* -24.002dB */
        0x00101D3F, /* -18.000dB */
        0x0016C310, /* -15.000dB */
        0x002026F2, /* -12.001dB */
        0x002D6A86, /* -9.000dB */
        0x004026E6, /* -6.004dB */
        0x005A9DF6, /* -3.000dB */
        0x0065AC8B, /* -2.000dB */
        0x00721481, /* -1.000dB */
        0x007FFFFF, /* FS */
};

int lx_level_peaks(struct lx_chip *chip, int is_capture, int channels,
                   u32 *r_levels)
{
        int err = 0;
        int i;

        mutex_lock(&chip->msg_lock);
        for (i = 0; i < channels; i += 4) {
                u32 s0, s1, s2, s3;

                lx_message_init(&chip->rmh, CMD_12_GET_PEAK);
                chip->rmh.cmd[0] |= PIPE_INFO_TO_CMD(is_capture, i);

                err = lx_message_send_atomic(chip, &chip->rmh);

                if (err == 0) {
                        s0 = peak_map[chip->rmh.stat[0] & 0x0F];
                        s1 = peak_map[(chip->rmh.stat[0] >>  4) & 0xf];
                        s2 = peak_map[(chip->rmh.stat[0] >>  8) & 0xf];
                        s3 = peak_map[(chip->rmh.stat[0] >>  12) & 0xf];
                } else
                        s0 = s1 = s2 = s3 = 0;

                r_levels[0] = s0;
                r_levels[1] = s1;
                r_levels[2] = s2;
                r_levels[3] = s3;

                r_levels += 4;
        }

        mutex_unlock(&chip->msg_lock);
        return err;
}

/* interrupt handling */
#define PCX_IRQ_NONE 0
#define IRQCS_ACTIVE_PCIDB        BIT(13)
#define IRQCS_ENABLE_PCIIRQ       BIT(8)
#define IRQCS_ENABLE_PCIDB        BIT(9)

static u32 lx_interrupt_test_ack(struct lx_chip *chip)
{
        u32 irqcs = lx_plx_reg_read(chip, ePLX_IRQCS);

        /* Test if PCI Doorbell interrupt is active */
        if (irqcs & IRQCS_ACTIVE_PCIDB) {
                u32 temp;
                irqcs = PCX_IRQ_NONE;

                while ((temp = lx_plx_reg_read(chip, ePLX_L2PCIDB))) {
                        /* RAZ interrupt */
                        irqcs |= temp;
                        lx_plx_reg_write(chip, ePLX_L2PCIDB, temp);
                }
                return irqcs;
        }
        return PCX_IRQ_NONE;
}

int lx_interrupt_debug_events(struct lx_chip *chip)
{
    int err;
    u32 stat[10];       /* answer from CMD_04_GET_EVENT */

    err = lx_dsp_read_async_events(chip, stat);
    if (err < 0) {
        dev_err(chip->card->dev, "lx_dsp_read_async_events: dsp timeout\n");
        return err;
    }
//    printk(KERN_DEBUG
//           "%s, %u %u %u %u\n",
//           __func__,
//           stat[1],
//           stat[2],
//           stat[3],
//           stat[4] );
    return err;
}

void lx_wakeup_audio_thread( struct lx_chip *chip )
{
//        printk(KERN_DEBUG  "%s\n", __func__);

        // Phase 1: Top half, in IRQ.
        if (chip->pThread != NULL)
        {
                chip->thread_wakeup++;
                wake_up( &thread_wait_queue ); // => bh_handler
        }
};

irqreturn_t lx_interrupt(int irq, void *dev_id)
{
        struct lx_chip *chip = dev_id;
        u32 irqsrc;
        u32 audio_irq_cpt;

        chip->debug_irq.irq_all++;
        irqsrc = lx_interrupt_test_ack(chip);
        if (irqsrc == PCX_IRQ_NONE) {
                chip->debug_irq.irq_none++;
                return IRQ_NONE; /* this device did not cause the interrupt */
        }

        if (irqsrc & MASK_SYS_STATUS_CMD_DONE) {
                chip->debug_irq.irq_handled++;
                atomic_inc(&chip->debug_irq.atomic_irq_handled);
                if (atomic_read(&chip->message_pending) == 1) {
                        atomic_set(&chip->message_pending, 0);
                }
        }

        if (irqsrc & MASK_SYS_STATUS_URUN) {
                dev_err(chip->card->dev, "interrupt: URUN\n");
//                atomic_inc(&chip->xrun_advertise);
//                chip->playback_stream.frame_pos = SNDRV_PCM_POS_XRUN;
        }

        if (irqsrc & MASK_SYS_STATUS_ORUN) {
                dev_err(chip->card->dev, "interrupt: ORUN\n");
//                atomic_inc(&chip->xrun_advertise);
//                chip->capture_stream.frame_pos = SNDRV_PCM_POS_XRUN;
        }

        if (irqsrc & MASK_SYS_STATUS_EOBI) {
                struct lx_stream         *lx_stream  = &(chip->capture_stream);
                struct snd_pcm_substream *substream  = lx_stream->stream;
                u32                      pos         = lx_stream->frame_pos;

                if (LX_STREAM_STATUS_RUNNING == lx_stream->status) {
                        pos = ((pos+1) >= substream->runtime->periods) ?
                                        0 : pos + 1;
                        lx_stream->frame_pos = pos;
                        snd_pcm_period_elapsed(substream);
                }
                else {
                        chip->debug_irq.irq_record_unhandled++;
                }
        }

        if (irqsrc & MASK_SYS_STATUS_EOBO) {
                struct lx_stream         *lx_stream  = &(chip->playback_stream);
                struct snd_pcm_substream *substream  = lx_stream->stream;
                u32                      pos         = lx_stream->frame_pos;

                if (LX_STREAM_STATUS_RUNNING == lx_stream->status) {
                        pos = ((pos+1) >= substream->runtime->periods) ?
                                        0 : pos + 1;
                        lx_stream->frame_pos = pos;
                        snd_pcm_period_elapsed(substream);
                }
                else {
                        chip->debug_irq.irq_play_unhandled++;

                }
        }

        if (irqsrc & MASK_SYS_STATUS_FREQ) {
                chip->debug_irq.irq_freq++;
        }
        if (irqsrc & MASK_SYS_STATUS_ESA) {
                chip->debug_irq.irq_esa++;
        }
        if (irqsrc & MASK_SYS_STATUS_TIMER) {
                chip->debug_irq.irq_timer++;
        }
        if (irqsrc & MASK_SYS_STATUS_EOT_PLX) {
                chip->debug_irq.irq_eot++;
        }
        if (irqsrc & MASK_SYS_STATUS_XES) {
                chip->debug_irq.irq_xes++;
        }
        if (irqsrc & MASK_SYS_STATUS_URUN) {
                chip->debug_irq.irq_urun++;
        }
        if (irqsrc & MASK_SYS_STATUS_ORUN) {
                chip->debug_irq.irq_orun++;
        }
        audio_irq_cpt = ( irqsrc & 0x0000ffff);
        if ((irqsrc & MASK_SYS_STATUS_EOBI) &&
           (irqsrc & MASK_SYS_STATUS_EOBO) &&
           (chip->capture_stream.status  == LX_STREAM_STATUS_RUNNING) &&
           (chip->playback_stream.status == LX_STREAM_STATUS_RUNNING) ) {
                //in order to calculate start duration
                if (chip->debug_irq.irq_play_and_record == 0) {
                        chip->jiffies_1st_irq = jiffies;
                }
                chip->debug_irq.irq_play_and_record++;
                if (chip->irq_audio_cpt_play == (unsigned int)-1) {
                        chip->irq_audio_cpt_play = audio_irq_cpt;
                }else {
                        chip->irq_audio_cpt_play +=
                                        chip->play_period_multiple_gran;
                        chip->irq_audio_cpt_play &= 0x0000ffff;
                        if (chip->irq_audio_cpt_play != audio_irq_cpt) {
                                atomic_inc(&chip->play_xrun_advertise);
                                chip->debug_irq.irq_urun++;
                                /* this is a awful stuff...
                                 * if driver miss irq today there is no way to
                                 * know how many irq had been missed so
                                 * there is no way to resync audio to IRQ...
                                 * userspace SW has to stop and restart audio
                                 */
                                chip->irq_audio_cpt_play = audio_irq_cpt;
                        }
                }
                if (chip->irq_audio_cpt_record == (unsigned int)-1) {
                        chip->irq_audio_cpt_record = audio_irq_cpt;
                }else {
                        chip->irq_audio_cpt_record +=
                                        chip->capture_period_multiple_gran;
                        chip->irq_audio_cpt_record &= 0x0000ffff;
                        if (chip->irq_audio_cpt_record != audio_irq_cpt) {
                                atomic_inc(&chip->capture_xrun_advertise);
                                chip->debug_irq.irq_orun++;
                                /* this is a awful stuff...
                                 * if driver miss irq today there is no way to
                                 * know how many irq had been missed so
                                 * there is no way to resync audio to IRQ...
                                 * userspace SW has to stop and restart audio
                                 */
                                chip->irq_audio_cpt_record = audio_irq_cpt;
                        }
                }
        }
        else if ((irqsrc & MASK_SYS_STATUS_EOBO) &&
                (chip->playback_stream.status == LX_STREAM_STATUS_RUNNING)) {
                if (chip->debug_irq.irq_play == 0) {
                        chip->jiffies_1st_irq = jiffies;
                }
                chip->debug_irq.irq_play++;
                if (chip->debug_irq.irq_play_and_record == 0) {
                        chip->debug_irq.irq_play_begin++;
                }

                if (chip->irq_audio_cpt_play == (unsigned int)-1) {
                        chip->irq_audio_cpt_play = audio_irq_cpt;
                }else {
                        chip->irq_audio_cpt_play +=
                                        chip->play_period_multiple_gran;
                        chip->irq_audio_cpt_play &= 0x0000ffff;
                        if (chip->irq_audio_cpt_play != audio_irq_cpt) {
                                atomic_inc(&chip->play_xrun_advertise);
                                atomic_inc(&chip->capture_xrun_advertise);
                                /* this is a awful stuff...
                                 * if driver miss irq today there is no way to
                                 * know how many irq had been missed so
                                 * there is no way to resync audio to IRQ...
                                 * userspace SW has to stop and restart audio
                                 */
                                chip->irq_audio_cpt_play = audio_irq_cpt;
                        }
                }
        }
        else if ((irqsrc & MASK_SYS_STATUS_EOBI) &&
                (chip->capture_stream.status == LX_STREAM_STATUS_RUNNING)) {
                if (chip->debug_irq.irq_record == 0) {
                        chip->jiffies_1st_irq = jiffies;
                }
                chip->debug_irq.irq_record++;
                if (chip->irq_audio_cpt_record == (unsigned int)-1) {
                        chip->irq_audio_cpt_record = audio_irq_cpt;
                }else {
                        chip->irq_audio_cpt_record +=
                                        chip->capture_period_multiple_gran;
                        chip->irq_audio_cpt_record &= 0x0000ffff;
                        if (chip->irq_audio_cpt_record != audio_irq_cpt) {
                                atomic_inc(&chip->capture_xrun_advertise);
                                /* this is a awful stuff...
                                 * if driver miss irq today there is no way to
                                 * know how many irq had been missed so
                                 * there is no way to resync audio to IRQ...
                                 * userspace SW has to stop and restart audio
                                 */
                                chip->irq_audio_cpt_record = audio_irq_cpt;
                        }
                }
        }

        return IRQ_HANDLED;
}

//Debug file.
//use to check interruptions
void lx_proc_get_irq_counter(struct snd_info_entry *entry,
                                struct snd_info_buffer *buffer)
{
        struct lx_chip *chip = entry->private_data;

        snd_iprintf(buffer, "IRQ HANDLER : \n" \
                            "\tirq_all :                   %d\n" \
                            "\tirq_wakeup_thread :         %d\n" \
                            "\tirq_play_begin :            %d\n" \
                            "\tirq_play :                  %d\n" \
                            "\tirq_play_unhandled :        %d\n" \
                            "\tirq_record :                %d\n" \
                            "\tirq_record_unhandled :      %d\n" \
                            "\tirq_play_and_record :       %d\n" \
                            "\tirq_none :                  %d\n" \
                            "\tirq_handled :               %d\n" \
                            "\tatomic_irq_handled :        %d\n" \
                            "\tirq_urun :                  %d\n" \
                            "\tirq_orun :                  %d\n" \
                            "\tirq_freq :                  %d\n" \
                            "\tirq_esa :                   %d\n" \
                            "\tirq_timer :                 %d\n" \
                            "\tirq_eot :                   %d\n" \
                            "\tirq_xes :                   %d\n" \
                            "THREAD IRQ : \n" \
                            "\twakeup_thread :             %d\n" \
                            "\tthread_play :               %d\n" \
                            "\thread_record_but_stop :     %d\n" \
                            "\tthread_record :             %d\n" \
                            "\tthread_record_but_stop :    %d\n" \
                            "\tthread_play_and_record :    %d\n" \
                            "\tasync_event_eobi :          %llu\n" \
                            "\tasync_event_eobo :          %llu\n" \
                            "\tasync_urun :                %llu\n" \
                            "\tasync_orun :                %llu\n" \
                            "commands : \n"  \
                            "\tcmd_irq_waiting:            %d\n" \
                            "MISC : \n" \
                            "\tstart time :                %d\n" \
                            "\tirq time :                %d\n" \
                            "\tstart delay :               %d\n",
                            chip->debug_irq.irq_all,
                            //atomic_read(&chip->debug_irq.irq_all),
                            chip->debug_irq.irq_wakeup_thread,
                            chip->debug_irq.irq_play_begin,
                            chip->debug_irq.irq_play,
                            chip->debug_irq.irq_play_unhandled,
                            chip->debug_irq.irq_record,
                            chip->debug_irq.irq_record_unhandled,
                            chip->debug_irq.irq_play_and_record,
                            chip->debug_irq.irq_none,
                            chip->debug_irq.irq_handled,
                            atomic_read(&chip->debug_irq.atomic_irq_handled),
                            chip->debug_irq.irq_urun,
                            chip->debug_irq.irq_orun,
                            chip->debug_irq.irq_freq,
                            chip->debug_irq.irq_esa,
                            chip->debug_irq.irq_timer,
                            chip->debug_irq.irq_eot,
                            chip->debug_irq.irq_xes,
                            chip->debug_irq.wakeup_thread,
                            chip->debug_irq.thread_play,
                            chip->debug_irq.thread_record_but_stop,
                            chip->debug_irq.thread_record,
                            chip->debug_irq.thread_play_but_stop,
                            chip->debug_irq.thread_play_and_record,
                            chip->debug_irq.async_event_eobi,
                            chip->debug_irq.async_event_eobo,
                            chip->debug_irq.async_urun,
                            chip->debug_irq.async_event_eobo,
                            chip->debug_irq.cmd_irq_waiting,
                            (unsigned int)chip->jiffies_start,
                            (unsigned int)chip->jiffies_1st_irq,
                            (unsigned int)(chip->jiffies_1st_irq-chip->jiffies_start));

}
static void lx_irq_set(struct lx_chip *chip, int enable)
{
        u32 reg = lx_plx_reg_read(chip, ePLX_IRQCS);

        /* enable/disable interrupts
         *
         * Set the Doorbell and PCI interrupt enable bits
         *
         * */
        if (enable) {
                reg |=  (IRQCS_ENABLE_PCIIRQ | IRQCS_ENABLE_PCIDB);
        }
        else {
                reg &= ~(IRQCS_ENABLE_PCIIRQ | IRQCS_ENABLE_PCIDB);
        }
        lx_plx_reg_write(chip, ePLX_IRQCS, reg);
}

void lx_irq_enable(struct lx_chip *chip)
{
        lx_irq_set(chip, 1);
}

void lx_irq_disable(struct lx_chip *chip)
{
        lx_irq_set(chip, 0);
}

int lx_madi_set_madi_state(struct lx_chip *chip)
{
        int err;

        mutex_lock(&chip->msg_lock);

        lx_message_init(&chip->rmh, CMD_15_SET_MADI_STATE);
        chip->rmh.cmd[0] |= (0x01 & chip->channel_mode);
        if (chip->rx_tx_mode ==  0)
                chip->rmh.cmd[1] |= 0x00000000 ;
        else
                chip->rmh.cmd[1] |= 0x00000003 ;

        err = lx_message_send_atomic(chip, &chip->rmh);
        if (err != 0) {
                printk(KERN_ERR  "%s->lx_message_send_atomic failed...\n",
                                __func__);
        }

        mutex_unlock(&chip->msg_lock);
        return err;
}
MODULE_LICENSE("GPL");
