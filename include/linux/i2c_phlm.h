/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author: Srinivas Rao L <srinivas.rao@entropic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef I2C_PHLM_H
#define I2C_PHLM_H

#include <linux/types.h>

#define I2C_M_WR    0x00

#define I2C_SET_SLAVE_ENABLE      _IOW('G', 0x0731, int)
#define I2C_SET_SLAVE_DISABLE     _IO('G', 0x0732)

#define I2C_GET_SLAVE_INFO        _IOR('G', 0x0733, unsigned char*)

#define I2C_SLAVE_RD_DONE         _IO('G', 0x0734)
#define I2C_SLAVE_WR_DONE         _IO('G', 0x0735)
#define I2C_SET_SPEED             _IOW('G', 0x0736, unsigned char*)
#define I2C_SLAVE_BUS_ERROR       _IO('G', 0x0737)
#define I2C_SET_MASTER_TIMEOUT    _IOW('G', 0x738, unsigned int)
#define I2C_SOFT_RESET            _IOW('G',0x739,unsigned int)
#define I2C_SET_SLAVE_TIMEOUT     _IOW('G',0x740,unsigned int)

/* Error codes in case of timeout*/
#define ESTART_TIMEOUT        120
#define ETRANSIT_TIMEOUT      121
#define EMASTERLOST_TIMEOUT   122
#define ESLAVE_TIMEOUT        123

typedef struct
{
    __u32 fskhz;    /* for fast or standard speed in khz*/
    __u32 hskhz;    /* for high speed in khz*/
    __u8 hs;        /*Boolean- True: High speed is set;
                                 False: otherwise*/
}i2c_ph_xferspeed;

typedef struct i2c_slave
{
    __u8  slave_addr;   /* slave address */
    __u8  general_call; /* True: enables general call */
    __u8 *tx_buf;      /* Transmit buffer */
    __u8 *rx_buf;      /* Receive buffer */
    __u32 tx_data_length; /* length of the slave transmit buffer */
    __u32 rx_data_length; /* length of the slave recieve buffer */
    __u32 rx_tx_bytecount; /* indicates number of bytes read or written.
                            Updated by kernel after Slave transfer */
    __u32 slave_event;    /* Slave Event from the i2c driver
                            I2C_SLAVE_RD_DONE or I2C_SLAVE_WR_DONE OR I2C_SLAVE_BUS_ERROR
                            */
}i2c_slave_data;


#define I2C_SUSPEND		  _IO('G', 0x741)
#define I2C_RESUME		  _IO('G', 0x742)

#ifndef __KERNEL__

/*
 * I2C Message - used for pure i2c transaction, also from /dev interface
 */
struct i2c_msg {
    __u16 addr; /* slave address            */
    __u16 flags;
#define I2C_M_TEN   0x10    /* we have a ten bit chip address   */
#define I2C_M_RD    0x01
#define I2C_M_NOSTART   0x4000
#define I2C_M_REV_DIR_ADDR  0x2000
#define I2C_M_IGNORE_NAK    0x1000
#define I2C_M_NO_RD_ACK     0x0800
    __u16 len;      /* msg length               */
    __u8 *buf;      /* pointer to msg data          */
};

/* To determine what functionality is present */

#define I2C_FUNC_I2C            0x00000001
#define I2C_FUNC_10BIT_ADDR     0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING  0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC   0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL  0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK        0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE    0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE   0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA   0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA  0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA   0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA  0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL    0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA  0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK   0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK  0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2  0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */

#define I2C_FUNC_SMBUS_BYTE (I2C_FUNC_SMBUS_READ_BYTE | \
                             I2C_FUNC_SMBUS_WRITE_BYTE)
#define I2C_FUNC_SMBUS_BYTE_DATA (I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BYTE_DATA)
#define I2C_FUNC_SMBUS_WORD_DATA (I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_WORD_DATA)
#define I2C_FUNC_SMBUS_BLOCK_DATA (I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                   I2C_FUNC_SMBUS_WRITE_BLOCK_DATA)
#define I2C_FUNC_SMBUS_I2C_BLOCK (I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)
#define I2C_FUNC_SMBUS_I2C_BLOCK_2 (I2C_FUNC_SMBUS_READ_I2C_BLOCK_2 | \
                                    I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2)

#define I2C_FUNC_SMBUS_EMUL (I2C_FUNC_SMBUS_QUICK | \
                             I2C_FUNC_SMBUS_BYTE | \
                             I2C_FUNC_SMBUS_BYTE_DATA | \
                             I2C_FUNC_SMBUS_WORD_DATA | \
                             I2C_FUNC_SMBUS_PROC_CALL | \
                             I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                             I2C_FUNC_SMBUS_I2C_BLOCK)

/*
 * Data for SMBus Messages
 */
#define I2C_SMBUS_BLOCK_MAX 32  /* As specified in SMBus standard */
union i2c_smbus_data {
    __u8 byte;
    __u16 word;
    __u8 block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
                           /* and one more for user-space compatibility */
};

/* smbus_access read or write markers */
#define I2C_SMBUS_READ  1
#define I2C_SMBUS_WRITE 0

/* SMBus transaction types (size parameter in the above functions)
   Note: these no longer correspond to the (arbitrary) PIIX4 internal codes! */
#define I2C_SMBUS_QUICK         0
#define I2C_SMBUS_BYTE          1
#define I2C_SMBUS_BYTE_DATA     2
#define I2C_SMBUS_WORD_DATA     3
#define I2C_SMBUS_PROC_CALL     4
#define I2C_SMBUS_BLOCK_DATA        5
#define I2C_SMBUS_I2C_BLOCK_DATA    6
#define I2C_SMBUS_BLOCK_PROC_CALL   7       /* SMBus 2.0 */


/* ----- commands for the ioctl like i2c_command call:
 * note that additional calls are defined in the algorithm and hw
 *  dependent layers - these can be listed here, or see the
 *  corresponding header files.
 */
                /* -> bit-adapter specific ioctls   */
#define I2C_RETRIES 0x0701  /* number of times a device address      */
                /* should be polled when not            */
                                /* acknowledging            */
#define I2C_TIMEOUT 0x0702  /* set timeout - call with int      */


/* this is for i2c-dev.c    */
#define I2C_SLAVE   0x0703  /* Change slave address         */
                /* Attn.: Slave address is 7 or 10 bits */
#define I2C_SLAVE_FORCE 0x0706  /* Change slave address         */
                /* Attn.: Slave address is 7 or 10 bits */
                /* This changes the address, even if it */
                /* is already taken!            */
#define I2C_TENBIT  0x0704  /* 0 for 7 bit addrs, != 0 for 10 bit   */

#define I2C_FUNCS   0x0705  /* Get the adapter functionality */
#define I2C_RDWR    0x0707  /* Combined R/W transfer (one stop only)*/
#define I2C_PEC     0x0708  /* != 0 for SMBus PEC                   */

#define I2C_SMBUS   0x0720  /* SMBus-level access */

#endif /*__KERNEL__*/
#endif /* I2C_PHLM_H*/
