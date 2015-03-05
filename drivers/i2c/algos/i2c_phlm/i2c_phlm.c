/*
 * Copyright (C) 2014, Entropic Communications. All Rights Reserved
 * Author:
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_PNX8XXX_HOTBOOT
#include <linux/notifier.h>
#include <hotboot.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <linux/i2c.h>
#include <linux/i2c_phlm.h>
#include <linux/mm.h>

#include "i2c_phlm_cfg.h"
#include "tmhwI2c.h"

#ifdef CONFIG_I2C_DEBUG
#define dbg_print(str...) printk(str)
#else
#define dbg_print(str...)
#endif

#ifdef CONFIG_I2C_DEBUG
extern void I2c_Error_Trigger( void );
#endif

/*************************************
 *data structures
 **************************************/

typedef struct
{
	spinlock_t      lock;
	__u64 int_pin;                       /* interrupt line for I2c unit*/
	__u32 unit_num;                    /* i2c unit number */
	wait_queue_head_t i2c_wait_master; /* for synchronous master */
	wait_queue_head_t i2c_wait_slave;
	int  mst_status;                   /* status of the  message.
					      For Master: num of messages transferred*/
	int  slv_status;                   /*For Slave :Trasnmit Done or Read Done
					     event */
	__u8 *dma_buf;                      /* Virtual pointer to Dma buffer */
	__u32 dma_addr_phy;            /* dma buffer physical address */
	__u32 num_of_msgs;                /* total number of msgs submitted 1, 2..
					     Valid for Master. Ignore this value for  Slave*/
	__u32 msg_index;                 /* Num of messages transferred
					    Valid for Master. Ignore this value for Slave*/
	__u32 mst_timeout;             /* Timeout for master transfer*/
	__u32 slv_timeout;              /* Timeout for slave transfer*/
	struct i2c_msg *i2c_mst_messages;
	/* chain of Master messages requested for transfer.*/
	i2c_slave_data i2c_slave_message;
	i2c_ph_xferspeed i2c_speedkhz;
	volatile __u8   slave_enabled;    /* True :slave is enabled for this unit*/
	volatile __u8   done;             /* True : master transfer done.*/
	volatile __u8   slvdone;          /* True : slave transfer done */
	volatile __u8   slvstarted;       /* True : slave request event occured*/
	volatile __u8   bus_busy;         /* True : unit/bus is busy */
	volatile __u8   master_active;    /* True : master transfer set by user */
	volatile __u8   master_start;     /* True : master has obtained the bus */
	volatile __u8   master_lost;      /* True : master arbitration lost occured*/
	volatile __u8   set_slave_disable;

}i2c_phlm_bus_t;



/*********************************************************
  Globals
 **********************************************************/

static i2c_phlm_bus_t *i2c_phlm_busobjects;
static struct i2c_algorithm *i2c_phlm_algorithms;

static unsigned int i2c_phlm_units;

static int* i2c_phlmno_adapterno_map;   /*Added for I2C Wrapper Driver interface*/

#ifdef CONFIG_PNX8XXX_HOTBOOT
struct notifier_block hotboot_notify;
#endif
/*********************************************************
  Local functions
 **********************************************************/


static void i2c_mst_request(__u32 i2c_unit,
		i2c_phlm_bus_t * busobject,
		__u32 event);

static void i2c_mst_done(__u32 i2c_unit,
		i2c_phlm_bus_t * busobject,
		__u32 event);

static void i2c_slv_request(__u32 i2c_unit,
		i2c_phlm_bus_t * busobject,
		__u32 event,
		tmhwI2cDirection_t dir);

static void i2c_slv_done(__u32 i2c_unit,
		i2c_phlm_bus_t * busobject,
		__u32 event, tmhwI2cDirection_t dir);


static void i2c_slave_mst_control( __u32 i2c_unit,
		i2c_phlm_bus_t * busobject);

static int i2c_algo_transfer(struct i2c_adapter *i2c_adap,
		struct i2c_msg msgs[],
		int num_msgs);

static int i2c_algo_control(struct i2c_adapter *adapter,
		unsigned int cmd,
		unsigned long arg);

static int  i2c_slave_enable(struct i2c_adapter *adapter,
		unsigned long data_length);


static int i2c_slave_disable(struct i2c_adapter *adapter);

static void i2c_slave_cleanup(i2c_phlm_bus_t *busobject);

static int i2c_add_bus(int device, struct i2c_adapter *adap);
static int i2c_delete_bus(struct i2c_adapter *adap);

#ifdef CONFIG_PNX8XXX_HOTBOOT
static int i2c_hotboot(struct notifier_block * self, unsigned long val, void * data);
#endif

static int i2c_softReset(unsigned int i2c_unit);
static int i2c_master_timeout(i2c_phlm_bus_t *busobject);

/*
 * i2c_isr - interrupt service routine
 * Returns : IRQ_HANDLED
 * Descn.  : handles the interrupts.
 */
static irqreturn_t i2c_isr(int irq, void *dev_id)
{
	// get the BusObject
	i2c_phlm_bus_t * busobject = (i2c_phlm_bus_t *)dev_id;
	__u32  event;
	__u32 i2c_unit = busobject->unit_num;
	tmhwI2cStatusType_t intr_status_type;
	__u8 slave_monitor;

	/* When the current message has been finished, the device returns to
	   addressed slave mode if slave_monitor = True. */
	slave_monitor = False;

	if ( busobject->slave_enabled == True){
		slave_monitor = True;
	}

	tmhwI2cIntGetStatus(i2c_unit, &intr_status_type);

	if (intr_status_type == tmhwI2cNoInterrupt){
		return IRQ_HANDLED;
	}

	busobject->bus_busy = True;

	/* Invoke the tmHwI2cEvent . Interrupt event*/
	event= tmhwI2cEvent(i2c_unit, tmhwI2cEventInterrupt, slave_monitor);
	switch(event)
	{
		case TMHW_ERR_I2C_BUSY:
			/* nointerrupt, probably polling */
			return IRQ_HANDLED;


		case TM_OK:
			break;

		case TMHW_ERR_I2C_EVENT_MST_REQ:
		case TMHW_ERR_I2C_EVENT_MST_TRX_REQ:
		case TMHW_ERR_I2C_EVENT_MST_REC_REQ:

			/* Invoke i2c_mst_request */
			i2c_mst_request(i2c_unit, busobject, event);
			tmhwI2cEvent(i2c_unit,tmhwI2cEventProceed,slave_monitor);
			break;

		case TMHW_ERR_I2C_EVENT_MST_LOST:

			busobject->msg_index = 0; /* retry the whole message */
			busobject->bus_busy = False;
			busobject->master_lost = True;
			busobject->master_start = False;
			break;

		case TMHW_ERR_I2C_EVENT_MST_TRX_DONE:
		case TMHW_ERR_I2C_EVENT_MST_TRX_ABORT :
		case TMHW_ERR_I2C_EVENT_MST_ABORT:
		case TMHW_ERR_I2C_EVENT_MST_REC_DONE:
		case TMHW_ERR_I2C_EVENT_MST_REC_ABORT:
			if ( busobject->master_active == True )
			{/* PR : Workaround
			  * Missing Master states 0x08,0x40.So, master_active is not set
			  * This will cause a NULL ptr in mst_msgs. Hence this workaround */
				i2c_mst_done(i2c_unit, busobject, event);
			}
			/* else do nothing ignore this state */
			break;

		case TMHW_ERR_I2C_EVENT_SLV_REC_REQ_MST_LOST:
		case TMHW_ERR_I2C_EVENT_SLV_GC_REQ_MST_LOST:
			busobject->msg_index = 0; /* retry the whole message */
			busobject->master_start = False;
			busobject->master_lost = True;
			/* ignoremaster lost */
		case TMHW_ERR_I2C_EVENT_SLV_GC_REQ:
		case TMHW_ERR_I2C_EVENT_SLV_REC_REQ:
			i2c_slv_request(i2c_unit, busobject, event, tmhwI2cReceive);
			tmhwI2cEvent(i2c_unit,tmhwI2cEventProceed,slave_monitor);
			break;

		case TMHW_ERR_I2C_EVENT_SLV_REC_DONE:
		case TMHW_ERR_I2C_EVENT_SLV_REC_ABORT:
			i2c_slv_done(i2c_unit, busobject, event, tmhwI2cReceive);
			break;

		case TMHW_ERR_I2C_EVENT_SLV_TRX_REQ_MST_LOST:
			busobject->msg_index = 0; /* retry the whole message */
			busobject->master_start = False;
			busobject->master_lost = True;

		case TMHW_ERR_I2C_EVENT_SLV_TRX_REQ:

			i2c_slv_request(i2c_unit, busobject, event, tmhwI2cTransmit);
			tmhwI2cEvent(i2c_unit,tmhwI2cEventProceed,slave_monitor);
			break;

		case TMHW_ERR_I2C_EVENT_SLV_TRX_DONE:
		case TMHW_ERR_I2C_EVENT_SLV_TRX_ABORT:
			i2c_slv_done(i2c_unit, busobject, event, tmhwI2cTransmit);
			break;

			/* buserror */
		case (TMHW_ERR_I2C_EVENT_BUS_ERROR):
			if ( busobject->master_active == True ){
				i2c_mst_done( i2c_unit, busobject, event);
			}
			else if ( busobject->slave_enabled == True ){
				i2c_slv_done(i2c_unit, busobject, event, tmhwI2cTransmit);
			}
			break;

		default :
			dbg_print("ERROR in state machine\n ");
			dbg_print( "Assertion failed! %s,%s,line=%d\n",\
					__FILE__,__FUNCTION__,__LINE__);
			break;
	}
	i2c_slave_mst_control(i2c_unit, busobject);

	return IRQ_HANDLED;

}

/*
 * i2c_slave_mst_control
 * Returns : None
 * Pre condn : Bus idle
 * Descn.  : Starts Master and Slave if required after the isr completes.
 */

static void i2c_slave_mst_control( __u32 i2c_unit, i2c_phlm_bus_t *
		busobject)
{

	__u32 slave_addr;
	tmhwI2cDirection_t direction;

	if ( busobject->bus_busy == False ){

		/* Check if Slave mode is enabled */
		if  (busobject->slave_enabled == True){
			/* Enable general call only if set to true */
			if ( busobject->i2c_slave_message.general_call == True )
			{
				tmhwI2cEnableGeneralCall(busobject->unit_num);
			}

			tmhwI2cSetSlaveAddr(busobject->unit_num,
					busobject->i2c_slave_message.slave_addr);
			tmhwI2cStartSlave(i2c_unit);
		}
		else{
			tmhwI2cStopSlave( i2c_unit);
		}

		/* Check if the master is active and messages pending */
		if (   busobject->master_active == True ){

			slave_addr = busobject->i2c_mst_messages[0].addr;
			if (busobject->i2c_mst_messages[0].flags & I2C_M_RD ){
				direction =  tmhwI2cReceive;
			}
			else{
				direction = tmhwI2cTransmit;
			}

			/* Start the Master. Transfer starts from first message*/
			tmhwI2cStartMaster( i2c_unit, slave_addr, direction);
		}
	}
}

/*
 * i2c_slv_request
 * Returns : None
 * Pre condn : None.
 * Descn.  : Handles Slave Request interrupt.
 */
static void i2c_slv_request(__u32 i2c_unit,
		i2c_phlm_bus_t *busobject,
		__u32 event, tmhwI2cDirection_t direction)
{
	tmhwI2cData_t data;
	i2c_slave_data *slvmsg = &(busobject->i2c_slave_message);

	data.direction = direction;

	if  (direction ==tmhwI2cTransmit ){
		data.pAddress =  slvmsg->tx_buf;
		data.counter =   slvmsg->tx_data_length;
		data.length =    slvmsg->tx_data_length;

	}
	else{
		data.pAddress =  slvmsg->rx_buf;
		data.counter = slvmsg->rx_data_length;
		data.length =  slvmsg->rx_data_length;
	}
	data.attach = tmhwI2cStop;

	tmhwI2cSetData(i2c_unit, &data);

	busobject->slvstarted = True;
	/* Slave has been addressed. Now put the slave wait
	   with timeout */
	wake_up_interruptible(&(busobject->i2c_wait_slave));

}

/*
 * i2c_slv_done
 * Returns : None
 * Pre condn : None.
 * Descn.  : Handles Slave Done interrupt.
 */
static void i2c_slv_done(__u32 i2c_unit,
		i2c_phlm_bus_t * busobject,
		__u32 event, tmhwI2cDirection_t direction)
{
	tmhwI2cData_t data;
	i2c_slave_data *slvmsg = &(busobject->i2c_slave_message);

	tmhwI2cGetData(i2c_unit, &data);

	if ( event == TMHW_ERR_I2C_EVENT_BUS_ERROR){
		busobject->slv_status = I2C_SLAVE_BUS_ERROR;
	}
	else{
		if ( direction == tmhwI2cReceive ){
			busobject->slv_status = I2C_SLAVE_RD_DONE;
		}
		else{
			busobject->slv_status = I2C_SLAVE_WR_DONE;
		}
	}

	slvmsg->rx_tx_bytecount = data.length - data.counter;

	busobject->slvdone = True;

	/* Disable the unit till the task is serviced */
	tmhwI2cIntDisable(i2c_unit);

	wake_up(&(busobject->i2c_wait_slave));


}

/*
 * i2c_mst_request
 * Returns : None
 * Pre condn : None.
 * Descn.  : Handles Master Request interrupt.
 */
static void i2c_mst_request(__u32 i2c_unit,
		i2c_phlm_bus_t * busobject, __u32 event)
{
	tmhwI2cData_t data;
	struct i2c_msg *mst_msg =
		&(busobject->i2c_mst_messages[busobject->msg_index]);

	/* Set the flag master_start = True */
	busobject->master_start = True;
	busobject->master_lost = False;

	/* Set the data */
	data.length =  mst_msg->len;
	data.counter = mst_msg->len;
	data.pAddress = mst_msg->buf;
	data.slaveAddress = mst_msg->addr;

	if ( mst_msg->flags & I2C_M_RD )
		data.direction = tmhwI2cReceive;
	else
		data.direction = tmhwI2cTransmit;


	if ( busobject->num_of_msgs == (busobject->msg_index+1) ){
		data.attach = tmhwI2cStop;
	}

	else{
		struct i2c_msg *mst_msg_next =
			&(busobject->i2c_mst_messages[busobject->msg_index + 1]);

		if ( mst_msg_next->flags & I2C_M_RD ){ /* Receive */
			if ( data.direction == tmhwI2cReceive ){
				if ((mst_msg->flags  & I2C_M_NOSTART) == I2C_M_NOSTART )
					data.attach = tmhwI2cChain;
				else
					data.attach = tmhwI2cRestart;
			}

			else
				data.attach = tmhwI2cRestart;
		}/*Receive*/
		else{ /* transmit */
			if ( data.direction == tmhwI2cReceive )
				data.attach = tmhwI2cRestart;
			else{
				if (mst_msg->flags   == I2C_M_NOSTART )
					data.attach = tmhwI2cChain;
				else
					data.attach = tmhwI2cRestart;
			}
		}/*transmit*/
	}

	busobject->msg_index += 1;

	if ( busobject->msg_index == 1 )
	{
		data.bHS = busobject->i2c_speedkhz.hs;
		tmhwI2cSetSpeed(i2c_unit, busobject->i2c_speedkhz.fskhz,
				busobject->i2c_speedkhz.hskhz);
	}
	tmhwI2cSetData(i2c_unit, &data);

}

/*
 * i2c_mst_done
 * Returns : None
 * Pre condn : None.
 * Descn.  : Handles Master Done interrupt.
 */
static void i2c_mst_done(__u32 i2c_unit,
		i2c_phlm_bus_t *busobject,
		__u32 event)
{
	tmhwI2cData_t data;
	struct i2c_msg *mst_msg =
		&(busobject->i2c_mst_messages[busobject->msg_index-1]);

	tmhwI2cGetData(i2c_unit, &data);
	mst_msg->len = data.length - data.counter;

	if ( event == TMHW_ERR_I2C_EVENT_BUS_ERROR){
		/* I2C-bus error.
		 * Workaround for STOP bit reset in INTROG register*/
		tmhwI2cDeinit(i2c_unit);
		tmhwI2cInit(i2c_unit);
		busobject->mst_status= -EIO;
	}
	else if (event == TMHW_ERR_I2C_EVENT_MST_ABORT){
		/*slave not available*/
		busobject->mst_status = -ENODEV;
	}
	else if (event == TMHW_ERR_I2C_EVENT_MST_TRX_ABORT ||
			event == TMHW_ERR_I2C_EVENT_MST_REC_ABORT){
		/* preminary abort of I2C communication */
		/* update the status = number of messages transferred */
		busobject->mst_status = busobject->msg_index -1;
	}
	else{
		/* success */
		/* Transfer is over */
		/* update the status = number of messages transferred */
		busobject->mst_status = busobject->msg_index;
	}

	busobject->done = True;

	wake_up( &(busobject->i2c_wait_master));

	/* Reset all the variables here !!*/
	busobject->bus_busy = False;
	busobject->msg_index = 0;
	busobject->num_of_msgs = 0;
	busobject->master_active = False;
	busobject->master_start = False;
	busobject->master_lost = False;
	busobject->i2c_mst_messages = Null;
}

/*
 * i2c_algo_transfer
 * Returns : status of Master Transfer. Num of messages transferred
 * Pre condn : None.
 * Descn.  : Initiates Master transfer.
 */
static int i2c_algo_transfer(struct i2c_adapter *i2c_adap,
		struct i2c_msg msgs[],
		int num_msgs)
{
	i2c_phlm_bus_t *busobject = ( i2c_phlm_bus_t *)i2c_adap->algo_data;
	struct i2c_msg *pmsg;
	int i;
	tmhwI2cDirection_t direction;
	int status;
	unsigned long flags;

	for ( i = 0; i < num_msgs;  i++ ){
		pmsg = &msgs[i];
		/* For 7bit address only for the moment */
		if (pmsg->flags & I2C_M_TEN)    /* addr = 10 bit addr, not supported */
			return -ENODEV;
		else{
			/* addr = 7 bit addr*/
			pmsg->addr &= 0x7f;
		}
	}

	if ( msgs[0].flags & I2C_M_RD )
		direction = tmhwI2cReceive;
	else
		direction = tmhwI2cTransmit;

	/* !!DISABLE PREEMPTION */
	spin_lock_irqsave(&busobject->lock, flags);

	busobject->i2c_mst_messages = msgs;

	busobject->msg_index = 0;
	busobject->num_of_msgs = num_msgs;


	busobject->done = False;

	busobject->master_active = True;

	if ( busobject->bus_busy != True ){
		tmhwI2cStartMaster(busobject->unit_num,
				busobject->i2c_mst_messages[0].addr,
				direction );
	}

	spin_unlock_irqrestore(&busobject->lock, flags);
	/* Pre-emption enabled */

	/* Suspend current task till the event flag is set */
	/* Wait for IIC transfer until the condition becomes true or
	   timeout(in secs) expires.Result of action is written in i2c_phlm_bus_t
	 */
	status = wait_event_timeout( (busobject->i2c_wait_master),
			(busobject->done == True),
			(busobject->mst_timeout));

	if ( busobject->done != True){
		dbg_print("Master transfer timeout on unit %d\n\n\n",busobject->unit_num);
#ifdef CONFIG_I2C_DEBUG
		I2c_Error_Trigger();
#endif
		/* check for the error code and reset the unit */
		status = i2c_master_timeout(busobject);
		return status;
	}

	status = busobject->mst_status;

	/* gives number of msgs succesfully transferred
	 */
	return status;


}

static int  i2c_slave_get_slv_info(i2c_phlm_bus_t *busobject,
		i2c_slave_data *userdata)
{

	int bytecount;
	i2c_slave_data  *slvmsg;
	int status;
	unsigned long flags;

	status = wait_event_interruptible( (busobject->i2c_wait_slave),
			(busobject->slvstarted == True) );

	if ( status != 0 ){
		return -EINTR;
	}
	else if (busobject->slv_status != I2C_SET_SLAVE_DISABLE)
	{/* slave is addressed . Now put the slave with a wait timeout*/
		status = wait_event_timeout( (busobject->i2c_wait_slave),
				(busobject->slvdone == True),
				(busobject->slv_timeout));
		if (busobject->slvdone != True)
		{
			/* Reset the slave unit*/
			dbg_print("Slave transfer timeout on unit %d\n\n\n",busobject->unit_num);
			spin_lock_irqsave(&busobject->lock, flags);

			i2c_softReset(busobject->unit_num);

			spin_unlock_irqrestore(&busobject->lock, flags);
			return -ESLAVE_TIMEOUT;
		}
	}

	busobject->slvdone = False;
	busobject->slvstarted = False;

	slvmsg = &(busobject->i2c_slave_message);
	bytecount = slvmsg->rx_tx_bytecount;
	userdata->rx_tx_bytecount = bytecount;
	userdata->slave_addr = slvmsg->slave_addr;

	userdata->slave_event =  busobject->slv_status;

	/* Disable the slave
	 */
	if (busobject->slv_status == I2C_SET_SLAVE_DISABLE){
		/* Invoke slave cleanup */
		i2c_slave_cleanup(busobject);
		return 0;
	}

	if ( busobject->slv_status == I2C_SLAVE_RD_DONE ){
		/* slave write done !*/;
		if ( (bytecount == 0) && (slvmsg->rx_data_length != 0))
			return -ENODATA;

		if ( bytecount != 0 ){
			if (copy_to_user(userdata->rx_buf,
						slvmsg->rx_buf,
						bytecount) )
				return -EFAULT;
		}

	}
	else if (busobject->slv_status == I2C_SLAVE_WR_DONE){
		/* slave read done !*/;
		if ( (bytecount == 0) && (slvmsg->tx_data_length != 0))
			return -ENODATA;
	}
	return 0;

}
/*
 * i2c_algo_control
 * Returns : 0 : success, Error otherwise.
 * Pre condn : None.
 * Descn.  : Handles ioctl calls for Maser and Slave
 */
static int i2c_algo_control(struct i2c_adapter *adapter, unsigned int cmd,
		unsigned long arg)
{

	i2c_phlm_bus_t *busobject =  ( i2c_phlm_bus_t *)adapter->algo_data;
	int ret=0;

	switch (cmd)
	{
		case I2C_SET_SLAVE_ENABLE:

			ret = i2c_slave_enable( adapter, arg);
			break;

		case I2C_SET_SLAVE_DISABLE:

			ret =  i2c_slave_disable( adapter );
			break;

		case I2C_GET_SLAVE_INFO:
			{
				i2c_slave_data *userdata = (i2c_slave_data*)arg;
				ret = i2c_slave_get_slv_info(busobject, userdata);
			}
			break;
		case I2C_SET_SPEED:
			{
				i2c_ph_xferspeed *speed = (i2c_ph_xferspeed *)arg;
				busobject->i2c_speedkhz.fskhz = speed->fskhz;
				busobject->i2c_speedkhz.hskhz = (speed->hskhz * speed->hs);
				busobject->i2c_speedkhz.hs = (speed->hs & (speed->hskhz !=0));
			}
			break;
		case I2C_SET_MASTER_TIMEOUT:
			{
				unsigned int timeout = (unsigned int)arg;
				if ( timeout >= 500 ) {
					busobject->mst_timeout = (timeout*HZ)/1000;
				}
				else{
					ret = -EINVAL;/* invalid timeout value*/
				}
			}
			break;
		case I2C_SET_SLAVE_TIMEOUT:
			{
				unsigned int timeout = (unsigned int)arg;
				if ( timeout >= 500 ) {
					busobject->slv_timeout = (timeout*HZ)/1000;
				}
				else{
					ret = -EINVAL;/* invalid timeout value*/
				}
			}
			break;
		case I2C_SOFT_RESET:
			{
				unsigned int i2c_unit = (unsigned int)arg;
				i2c_softReset(i2c_unit);
			}
			break;
		case I2C_SUSPEND:
			{
				(void)tmhwI2cSetIntEnable(busobject->unit_num, FALSE);
				(void)tmhwI2cSetPowerState(busobject->unit_num,tmPowerOff);
			}
			break;
		case I2C_RESUME:
			{
				(void)tmhwI2cSetPowerState(busobject->unit_num,tmPowerOn);
				(void)tmhwI2cSetIntEnable(busobject->unit_num, TRUE);
			}
			break;
		default:
			ret = -ENOTTY;/* no such commamd*/
			break;
	}
	return ret;
}

/*
 * i2c_slave_enable
 * Returns : 0 : success, Error otherwise.
 * Pre condn : None.
 * Descn.  : Enables the slave.
 */

int  i2c_slave_enable(struct i2c_adapter *adapter, unsigned long arg)
{
	i2c_phlm_bus_t *busobject
		=  ( i2c_phlm_bus_t *)adapter->algo_data;
	i2c_slave_data *slvmsg;
	i2c_slave_data *userdata = (i2c_slave_data*)arg;

	int rxlen, txlen;
	unsigned long flags;

	slvmsg = &busobject->i2c_slave_message;
	rxlen =  userdata->rx_data_length;
	txlen =  userdata->tx_data_length;

	if (busobject->slave_enabled != True ){

		slvmsg->rx_data_length = rxlen;
		slvmsg->tx_data_length = txlen;

		/* Copy message for slave read */
		if ( (rxlen != 0) && (userdata->rx_buf == NULL) ){
			return -EFAULT;
		}
		if ( rxlen != 0 ){
			slvmsg->rx_buf = (unsigned char*)kmalloc(rxlen, GFP_KERNEL);
			if (slvmsg->rx_buf == NULL)
				return -ENOMEM;
		}

		if ( (txlen != 0) && (userdata->tx_buf == NULL) ){
			if (slvmsg->rx_buf != NULL)
				kfree(slvmsg->rx_buf);
			return -EFAULT;
		}

		if ( txlen != 0 ){
			slvmsg->tx_buf = (unsigned char*)kmalloc(txlen, GFP_KERNEL);
			if (slvmsg->tx_buf == NULL){
				if ( slvmsg->rx_buf != NULL )
					kfree(slvmsg->rx_buf);
				return -ENOMEM;
			}
			if( copy_from_user( slvmsg->tx_buf,
						userdata->tx_buf, txlen ) ){
				if ( slvmsg->rx_buf != NULL )
					kfree(slvmsg->rx_buf);
				kfree(slvmsg->tx_buf);
				return -EFAULT;
			}
		}

		slvmsg->slave_addr = userdata->slave_addr;

		spin_lock_irqsave(&busobject->lock, flags);

		busobject->slave_enabled = True;
		busobject->slvdone = False;
		busobject->slvstarted = False;
		busobject->slv_status = I2C_SET_SLAVE_ENABLE;

		slvmsg->general_call = userdata->general_call;

		if ( busobject->bus_busy == False ){
			/* Enable general call only if set to true */
			if ( userdata->general_call == True )
			{
				tmhwI2cEnableGeneralCall(busobject->unit_num);
			}
			tmhwI2cSetSlaveAddr(busobject->unit_num, slvmsg->slave_addr);
			tmhwI2cStartSlave(busobject->unit_num);
		}
		spin_unlock_irqrestore(&busobject->lock, flags);

	}
	else{/*just renable the interrupt and
	       copy any transmit data from user*/
		/* Make this preempt safe */

		if ( busobject->set_slave_disable == True ){

			spin_lock_irqsave(&busobject->lock, flags);
			busobject->bus_busy = False;

			i2c_slave_disable(adapter);
			spin_unlock_irqrestore(&busobject->lock, flags);
			return 0;

		}


		if( txlen != 0){
			if ( userdata->tx_buf == NULL )
				return -EFAULT;

			if (copy_from_user( slvmsg->tx_buf,
						userdata->tx_buf, txlen )) {
				return -EFAULT;
			}
		}

		spin_lock_irqsave(&busobject->lock, flags);
		busobject->slave_enabled = True;
		busobject->slvdone = False;
		busobject->slvstarted = False;
		busobject->slv_status = I2C_SET_SLAVE_ENABLE;

		tmhwI2cIntEnable(busobject->unit_num);

		busobject->bus_busy = False;

		/* Restart the slave or master on this unit */
		i2c_slave_mst_control(busobject->unit_num, busobject);
		spin_unlock_irqrestore(&busobject->lock, flags);

	}

	return 0;
}

/*
 * i2c_slave_disable
 * Returns : 0 : success, Error otherwise.
 * Pre condn : Slave  enabled
 * Descn.  : Disables the slave.
 */

int i2c_slave_disable(struct i2c_adapter *adapter)
{
	i2c_phlm_bus_t * busobject =  ( i2c_phlm_bus_t *)adapter->algo_data;
	i2c_slave_data  *slvmsg;
	unsigned long flags;

	if (busobject->slave_enabled == False )
		return 0;

	slvmsg = &(busobject->i2c_slave_message);

	busobject->set_slave_disable = True;

	/* Make it preempt safe and check if the bus is not busy
	   before stopping the slave */
	spin_lock_irqsave(&busobject->lock, flags);

	if ( busobject->bus_busy == False ){
		/* reset the slave */
		busobject->set_slave_disable = False;
		busobject->slv_status = I2C_SET_SLAVE_DISABLE;
		busobject->slave_enabled = False;

		if ( busobject->slvstarted == False )
		{/* check if slave hasnot been addressed yet*/
			busobject->slvstarted = True;
			wake_up_interruptible(&(busobject->i2c_wait_slave));
		}
		else
		{/* slave is addressed*/
			busobject->slvdone = True;
			wake_up(&(busobject->i2c_wait_slave));
		}
	}
	spin_unlock_irqrestore(&busobject->lock, flags);
	return 0;
}

/*
 * i2c_slave_cleanup
 * Returns : None
 * Pre condn : None
 * Descn.  : Stops the Slave unit and cleans up the memory
 *          Called when I2C_SLAVE_DISABLE ioctl is invoked
 *          or is called from i2c_exit function.
 */

static void i2c_slave_cleanup(i2c_phlm_bus_t *busobject)
{
	unsigned long flags;
	i2c_slave_data  *slvmsg = &(busobject->i2c_slave_message);

	busobject->slave_enabled = False;

	spin_lock_irqsave(&busobject->lock, flags);
	tmhwI2cIntEnable(busobject->unit_num);
	tmhwI2cStopSlave(busobject->unit_num);
	spin_unlock_irqrestore(&busobject->lock, flags);

	slvmsg->rx_tx_bytecount = 0;
	slvmsg->tx_data_length = slvmsg->rx_data_length = 0;
	slvmsg->slave_addr = 0;

	if (NULL != slvmsg->rx_buf){
		kfree(slvmsg->rx_buf);  // kfree returns void, no check needed
		slvmsg->rx_buf = NULL;
	}

	if (NULL != slvmsg->tx_buf){
		kfree(slvmsg->tx_buf);  // kfree returns void, no check needed
		slvmsg->tx_buf = NULL;
	}

}


/*
 * i2c_bus_init
 * Returns : -EFAULT error
 * Pre condn :
 * Descn.  : Initializes the i2c busses.
 */

static int i2c_bus_init(int device)
{
	i2c_phlm_bus_t *busobject;
	tmhwI2cMmFunc_t     funcStruct;

	busobject              = &(i2c_phlm_busobjects[device]);
	busobject->int_pin     = i2c_phlm_cfg_intpins[device];

	busobject->unit_num     = device;
	busobject->num_of_msgs  = 0;
	busobject->msg_index    = 0;
	busobject->slave_enabled= False;

	busobject->done = False;
	busobject->slvdone = False;
	busobject->slvstarted = False;
	busobject->bus_busy = False;
	busobject->master_active = False;
	busobject->master_start = False;
	busobject->master_lost = False;
	busobject->set_slave_disable = False;
	busobject->slv_status = I2C_SET_SLAVE_DISABLE;

	busobject->i2c_speedkhz.fskhz = 100;
	busobject->i2c_speedkhz.hskhz = 100;
	busobject->i2c_speedkhz.hs = False;

	busobject->mst_timeout = (I2C_PHCFG_TIMEOUT*HZ)/1000;
	busobject->slv_timeout = (I2C_PHCFG_SLV_TIMEOUT*HZ)/1000;

	spin_lock_init(&busobject->lock);

	/* The remaining elements of the struct are initialized when i2c write,read
	   functions are called */

	busobject->dma_buf =
		dma_alloc_coherent(  NULL,
				PAGE_SIZE,
				(dma_addr_t*)(&busobject->dma_addr_phy),
				GFP_KERNEL );

	if (busobject->dma_buf == NULL){
		printk (KERN_ERR "i2c_phlm: Unable allocate DMA memory.\r\n");
		return -ENOMEM;
	}

	init_waitqueue_head( &(busobject->i2c_wait_master) );
	init_waitqueue_head( &(busobject->i2c_wait_slave) );


	/* store the Buffer Pointer in the funcStruct structure */
	funcStruct.pI2cVirtToPhys = Null;
	funcStruct.pI2cCacheFlush = Null;
	funcStruct.pI2cCacheInvalidate = Null;
	funcStruct.pI2cDynamicMemPtr = busobject->dma_buf;
	funcStruct.i2cDynamicMemLength = PAGE_SIZE;
	funcStruct.phyAddr =     busobject->dma_addr_phy;
	tmhwI2cRegisterMmFunc(busobject->unit_num, &funcStruct);

	/* Install interrupt handler. */
	if (0 != request_irq( busobject->int_pin, (i2c_isr),
				IRQF_DISABLED,
				"i2c", (void *)busobject)){
		dbg_print("install interrupt handler error for %d unit\n",busobject->unit_num);
		return -EFAULT;
	}

	return 0;
}

#ifdef CONFIG_PM
static int i2c_phlm_suspend(struct device *dev)
{
	int ret = 0;
	struct i2c_adapter *adap = to_i2c_adapter(dev);

	i2c_algo_control(adap,I2C_SUSPEND,0);
	return ret;
}

static int i2c_phlm_resume(struct device *dev)
{
	int ret = 0;
	struct i2c_adapter *adap = to_i2c_adapter(dev);

	i2c_algo_control(adap,I2C_RESUME,0);
	return ret;
}

/* Power management functions for IP3203 I2C controller */
static struct dev_pm_ops i2c_phlm_dev_pm_ops = {
	.suspend  = i2c_phlm_suspend,
	.resume   = i2c_phlm_resume,
};

static struct device_type i2c_phlm_dev_type = {
	.name     = "i2c_phlm_dev_type",
	.pm       = &i2c_phlm_dev_pm_ops,
};
#endif

/*
 * i2c_add_bus
 * Returns : 0 success
 * Pre condn :
 * Descn.  : registering functions to load algorithms at runtime
 */

static int i2c_add_bus(int device, struct i2c_adapter *adap)
{
	int res;

	dbg_print("i2c-phlm.o: routines for %s registered.\n",
			adap->name);

	dbg_print("Initialise device %d\n", device);

	/* register new adapter to i2c module... */
	adap->algo = &(i2c_phlm_algorithms[device]);
	adap->timeout = I2C_PHCFG_TIMEOUT;
	adap->retries = I2C_PHCFG_NUM_RETIRES;

	res = i2c_bus_init(device);
	if ( res != 0){
		dbg_print("initialization failed for device %d\n",device);
		return -EFAULT;
	}

#ifdef CONFIG_PM
	adap->dev.type = &i2c_phlm_dev_type;
#endif

	/* i2c device drivers may be active on return from add_adapter() */
	res = i2c_add_numbered_adapter(adap);
	if (res) {
		dbg_print( "failure adding adapter:%s\n",adap->name);
		return -EFAULT;
	}


	i2c_phlmno_adapterno_map[device] = adap->nr; /*Added for I2C Wrapper Driver interface*/

	tmhwI2cInit(device);

	return 0;
}

/*
 * i2c_delete_bus
 * Returns : None
 * Pre condn :
 * Descn.  : ungistering functions to load algorithms at runtime
 */

static int i2c_delete_bus(struct i2c_adapter *adap)
{
	i2c_del_adapter(adap);
	dbg_print("i2c-phlm.o: adapter unregistered: %s\n",adap->name);

	return 0;
}

int i2c_get_adapterno(int deviceno)
{
	int adapterno = -ENODEV;

	if (deviceno < i2c_phlm_units)
	{
		adapterno = i2c_phlmno_adapterno_map[deviceno];
	}

	return adapterno;
}

EXPORT_SYMBOL(i2c_get_adapterno);

int i2c_get_timeoutval(int deviceno, unsigned int* timeoutval)
{
	int retVal = -ENODEV;

	i2c_phlm_bus_t *busobject = NULL;

	if (deviceno < i2c_phlm_units)
	{
		busobject = &(i2c_phlm_busobjects[deviceno]);

		*timeoutval = (busobject->mst_timeout * 1000) /HZ;

		retVal = 0;
	}
	return retVal;
}

EXPORT_SYMBOL(i2c_get_timeoutval);


int i2c_set_timeoutval(int deviceno, unsigned int timeoutval)
{
	int retVal = -ENODEV;

	i2c_phlm_bus_t *busobject = NULL;

	if (deviceno < i2c_phlm_units)
	{
		busobject = &(i2c_phlm_busobjects[deviceno]);

		if ( timeoutval >= 500 )
		{
			busobject->mst_timeout = (timeoutval*HZ)/1000;
			retVal = 0;
		}
	}
	return retVal;
}

EXPORT_SYMBOL(i2c_set_timeoutval);


int i2c_get_busspeed(int deviceno, i2c_ph_xferspeed* busspeed)
{
	int retVal = -ENODEV;

	i2c_phlm_bus_t *busobject = NULL;

	if (deviceno < i2c_phlm_units)
	{
		busobject = &(i2c_phlm_busobjects[deviceno]);

		busspeed->fskhz = busobject->i2c_speedkhz.fskhz;
		busspeed->hskhz = busobject->i2c_speedkhz.hskhz;
		busspeed->hs = busobject->i2c_speedkhz.hs;

		retVal = 0;
	}

	return retVal;
}

EXPORT_SYMBOL(i2c_get_busspeed);


int i2c_set_busspeed(int deviceno, i2c_ph_xferspeed* busspeed)
{
	int retVal = -ENODEV;

	i2c_phlm_bus_t *busobject = NULL;

	if (deviceno < i2c_phlm_units)
	{
		busobject = &(i2c_phlm_busobjects[deviceno]);

		busobject->i2c_speedkhz.fskhz = busspeed->fskhz;
		busobject->i2c_speedkhz.hskhz = (busspeed->hskhz * busspeed->hs);
		busobject->i2c_speedkhz.hs = (busspeed->hs & (busspeed->hskhz !=0));

		retVal = 0;
	}

	return retVal;
}

EXPORT_SYMBOL(i2c_set_busspeed);



#ifdef CONFIG_PNX8XXX_HOTBOOT
/*
 *i2c_hotboot
 *Returns : None
 *Pre cond : Hotboot reset event has occured
 *Descn : Terminates the ongoing transfers and performs hw deinit
 */
static int i2c_hotboot(struct notifier_block *self, unsigned long val, void * data)
{
	tmhwI2cCapabilities_t unit_capb;
	int device;

	tmhwI2cGetCapabilities ( 0, &unit_capb );
	i2c_phlm_units = unit_capb.i2cUnitCount;

	for (device=0;device<i2c_phlm_units;device++)
	{
		tmhwI2cHotBoot(device);
	}

	return 0;
}
#endif

/*
 *i2c_softreset
 *Returns : None
 *Pre cond : Fatal error on the bus.
 *Descn : Resets the hardware registers of the i2c unit.
 */
static int i2c_softReset(unsigned int i2c_unit)
{


	/* Do a hard reset of the i2c unit */
	tmhwI2cSoftReset(i2c_unit);

	/* Reset the state of the i2c unit without resetting setup values
	   like speed, timeout*/
	i2c_phlm_busobjects[i2c_unit].bus_busy = False;
	i2c_phlm_busobjects[i2c_unit].msg_index = 0;
	i2c_phlm_busobjects[i2c_unit].num_of_msgs = 0;
	i2c_phlm_busobjects[i2c_unit].master_active = False;
	i2c_phlm_busobjects[i2c_unit].master_lost = False;
	i2c_phlm_busobjects[i2c_unit].master_start = False;
	i2c_phlm_busobjects[i2c_unit].i2c_mst_messages = Null;

	i2c_phlm_busobjects[i2c_unit].slvstarted = False;
	i2c_phlm_busobjects[i2c_unit].slv_status = I2C_SET_SLAVE_ENABLE;
	i2c_phlm_busobjects[i2c_unit].slave_enabled = False;
	i2c_phlm_busobjects[i2c_unit].slvdone = False;

	/* send a wakeupsignal for slave.*/
	wake_up_interruptible(&(i2c_phlm_busobjects[i2c_unit].i2c_wait_slave));

	/* Reset all the hwapi registers */
	tmhwI2cDeinit(i2c_unit);
	tmhwI2cInit(i2c_unit);

	return 0;
}

static int i2c_master_timeout(i2c_phlm_bus_t *busobject)
{
	unsigned long flags;
	int status=0;

	spin_lock_irqsave(&busobject->lock, flags);

	if ( busobject->master_start == False )
	{
		/* Check for START TIMEOUT. No Start was put on the bus*/
		if ( busobject->master_lost == False )
		{
			status = -ESTART_TIMEOUT;
		}
		else
		{   /*  MASTER LOST TIMEOUT. No start on the bus & previously master
			was lost*/
			status = -EMASTERLOST_TIMEOUT;
		}
	}
	else
	{/*   TRANSITT TIMEOUT. Start was put on the bus, but transfer could not complete */
		status = -ETRANSIT_TIMEOUT;
	}

	/* Reset the unit . Does not reset the setup values - timeout & speed*/
	i2c_softReset(busobject->unit_num);

	spin_unlock_irqrestore(&busobject->lock, flags);

	return status;
}



static u32 i2c_algo_func(struct i2c_adapter *adap)
{
	return (I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL); /*FIXME:: Check if I2C_FUNC_SMBUS_EMUL needed*/
}


/*
 * i2c_phlm_init
 * Returns : None
 * Pre condn :
 * Descn.  : Initializes the i2c busses.
 */

int __init i2c_phlm_init (void)
{
	int device;
	tmhwI2cCapabilities_t unit_capb;


	tmhwI2cGetCapabilities ( 0, &unit_capb );

	i2c_phlm_units = unit_capb.i2cUnitCount;

	i2c_phlm_busobjects =
		(i2c_phlm_bus_t*) kzalloc( (i2c_phlm_units *
					sizeof (i2c_phlm_bus_t)),
				GFP_KERNEL);

	if (i2c_phlm_busobjects == NULL)
	{
		dbg_print (KERN_ERR "I2C_PHLM: Memory allocation failed\r\n");
		return -ENOMEM;
	}

	memset(i2c_phlm_busobjects, 0, (i2c_phlm_units *
				sizeof (i2c_phlm_bus_t)));

	i2c_phlm_algorithms =
		(struct i2c_algorithm*) kzalloc( (i2c_phlm_units *
					sizeof (struct i2c_algorithm)),
				GFP_KERNEL
				);
	if (i2c_phlm_algorithms == NULL)
	{
		dbg_print (KERN_ERR "I2C_PHLM: Memory allocation failed\r\n");
		kfree(i2c_phlm_busobjects);
		return -ENOMEM;
	}


	i2c_phlmno_adapterno_map = (int*) kzalloc((i2c_phlm_units *
				sizeof (int)),
			GFP_KERNEL
			);
	if (i2c_phlmno_adapterno_map == NULL)
	{
		dbg_print (KERN_ERR "I2C_PHLM: Memory allocation failed for adapterno"
				" table\r\n");
		kfree(i2c_phlm_algorithms);
		kfree(i2c_phlm_busobjects);
		return -ENOMEM;
	}

#ifdef CONFIG_PNX8XXX_HOTBOOT
	/* register for hotboot*/
	hotboot_notify.notifier_call = i2c_hotboot;
	hotboot_notify.next = NULL;
	pnx8xxx_hotboot_hook_register(&hotboot_notify);
#endif

	for (device = 0; device < i2c_phlm_units; device++){

		tmhwI2cGetCapabilities ( device, &unit_capb );

		strcpy(i2c_phlm_cfg_adapters[device].name, "i2c-adap");
		i2c_phlm_cfg_adapters[device].nr = device;
		i2c_phlm_cfg_adapters[device].algo_data
			= &i2c_phlm_busobjects[device];

		i2c_phlm_cfg_adapters[device].algo = &i2c_phlm_algorithms[device];

		i2c_phlm_algorithms[device].master_xfer = i2c_algo_transfer;
		i2c_phlm_algorithms[device].functionality = i2c_algo_func;

		i2c_phlmno_adapterno_map[device] = -1;

		if (i2c_add_bus(device, &i2c_phlm_cfg_adapters[device]) < 0)
		{
			dbg_print("i2c-device%d: Unable to register with I2C\n",
					device);
			kfree(i2c_phlm_busobjects);
			kfree(i2c_phlm_algorithms);
			kfree(i2c_phlmno_adapterno_map);
			return -ENODEV;
		}
	}

	return 0;
}

/*
 * i2c_phlm_exit
 * Returns : None
 * Pre condn :
 * Descn.  : Deinitializes the i2c busses.
 */

void __exit i2c_phlm_exit(void)
{
	i2c_phlm_bus_t *busobject;
	int device;


#ifdef CONFIG_PNX8XXX_HOTBOOT
	/* register for hotboot*/
	hotboot_notify.notifier_call = i2c_hotboot;
	hotboot_notify.next = NULL;
	pnx8xxx_hotboot_hook_unregister(&hotboot_notify);
#endif

	for( device = 0; device < i2c_phlm_units; device++ )
	{
		dbg_print("exit bus %x\n", device);
		busobject = &i2c_phlm_busobjects[device];

		i2c_slave_disable(&i2c_phlm_cfg_adapters[device]);

		/* disable the i2c_unit using HWAPI */
		tmhwI2cDeinit(device);

		free_irq(busobject->int_pin,(void*)busobject);

		dma_free_coherent (
				NULL,
				PAGE_SIZE,
				(void*)busobject->dma_buf,
				(dma_addr_t)( busobject->dma_addr_phy)
				);

		busobject->dma_buf = NULL;

		i2c_delete_bus(&i2c_phlm_cfg_adapters[device]);
	}

	kfree(i2c_phlm_busobjects);
	kfree(i2c_phlm_algorithms);
	kfree(i2c_phlmno_adapterno_map);
}

module_init(i2c_phlm_init);
module_exit(i2c_phlm_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C-Bus adapter routines");
