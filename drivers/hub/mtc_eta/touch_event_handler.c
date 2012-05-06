/*
 *  Copyright (c) 2010 LGE.
 *
 *  All source code in this file is licensed under the following license
 *  except where indicated.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, you can find it at http://www.fsf.org
 */

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include "lge_mtc_eta.h"

//#define MTC_ETA_DEBUG
#ifdef MTC_ETA_DEBUG
#define PDEBUG(fmt, args...) printk("mtc_eta_touch: " fmt, ## args)
#else
#define PDEBUG(fmt, args...)
#endif

static struct input_handler input_handler;
static struct workqueue_struct *touch_log_workqueue;//hak.lee@lge.com
static struct work_struct event_log_work;
static struct log_data_touch_event touch_event;

void mtc_eta_add_logging_event(struct mtc_eta_log *log);

static int eta_abs_event[] = {
	ABS_MT_TOUCH_MAJOR,
	ABS_MT_WIDTH_MAJOR,
	ABS_MT_POSITION_X,
	ABS_MT_POSITION_Y,
};

/* (MT sync -> sync) report after no major, x or y report means that touch is pen-up */
enum {
	TOUCH_STATE_NONE,
	TOUCH_STATE_DOWN,
	TOUCH_STATE_MOVE,//hak.lee@lge.com
	TOUCH_STATE_SYNC,
	TOUCH_STATE_SYNC_MT,
};
static int touch_state = TOUCH_STATE_NONE;
static int touch_sent = 0;//hak.lee@lge.com

static int mtc_eta_event_log_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;

	if (dev->name == NULL)
		return 0;

#if defined(CONFIG_PRODUCT_LGE_JUSTIN)
	if (strcmp(dev->name, "justin_synaptics_touch") != 0)
		return 0;
#elif defined(CONFIG_PRODUCT_LGE_BLACK)
	if (strcmp(dev->name, "black_synaptics_touch") != 0)
		return 0;
#elif defined(CONFIG_PRODUCT_LGE_HUB)
	if (strcmp(dev->name, "hub_synaptics_touch") != 0)
		return 0;
#else
#error "Not defined CONFIG_PRODUCT_xxx"
#endif


	for (i = 0 ; i < ARRAY_SIZE(eta_abs_event) - 1 ; i++) {
		if (test_bit(eta_abs_event[i], dev->absbit))
			break;
	}
	if (i == ARRAY_SIZE(eta_abs_event))
		return -ENODEV;

	PDEBUG("connect () %s\n",dev->name);

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if(!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "mtc_eta_touch_event_log";
	handle->private = NULL;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void mtc_eta_event_log_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id mtc_eta_event_log_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, mtc_eta_event_log_ids);

static void event_log_work_func(struct work_struct *work)
{
	struct mtc_eta_log *new_log_item;

	new_log_item = kmalloc(sizeof(struct mtc_eta_log), GFP_ATOMIC);
	if (!new_log_item) {
		printk("mtc_eta_key: cannot allocate new_item buffer\n");
		return;
	}

	new_log_item->id = MTC_ETA_LOG_ID_TOUCH;
	new_log_item->data.touch.action = touch_event.action;
	new_log_item->data.touch.x = touch_event.x;
	new_log_item->data.touch.y = touch_event.y;
	PDEBUG("NEW: id %d\n", new_log_item->id);
	PDEBUG("NEW: touch action 0x%x\n", new_log_item->data.touch.action);
	PDEBUG("NEW: touch x %d\n", new_log_item->data.touch.x);
	PDEBUG("NEW: touch y %d\n", new_log_item->data.touch.y);

	mtc_eta_add_logging_event(new_log_item);
}
//hak.lee@lge.com start

static void make_new_event_log(void)
{
	struct mtc_eta_log *new_log_item;

	new_log_item = kmalloc(sizeof(struct mtc_eta_log), GFP_ATOMIC);
	if (!new_log_item) {
		printk("mtc_eta_key: cannot allocate new_item buffer\n");
		return;
	}

	new_log_item->id = MTC_ETA_LOG_ID_TOUCH;
	new_log_item->data.touch.action = touch_event.action;
	new_log_item->data.touch.x = touch_event.x;
	new_log_item->data.touch.y = touch_event.y;
	PDEBUG("NEW: id %d\n", new_log_item->id);
	PDEBUG("NEW: touch action 0x%x\n", new_log_item->data.touch.action);
	PDEBUG("NEW: touch x %d\n", new_log_item->data.touch.x);
	PDEBUG("NEW: touch y %d\n", new_log_item->data.touch.y);

	mtc_eta_add_logging_event(new_log_item);
}

static void mtc_eta_event_log_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	static int finger = 0;

	PDEBUG("[%d] type: %u, code: 0x%x, value: %d\n", finger, type, code, value);
	//printk("##finger :%d###type:%u####code 0x%x t_state : %d####\n",finger,type,code,touch_state);
	switch (type) {
		case EV_SYN: /* 0x00 */
			switch (code) {
				case SYN_REPORT: /* 0x00 */
					//printk("++++uch_state : %d+++action : %d++++\n",touch_state,touch_event.action);
					if (touch_state == TOUCH_STATE_SYNC_MT) {

						
						if(touch_event.action == ETA_TOUCH_DOWN)
						{
							touch_state = TOUCH_STATE_DOWN;//keep move to 
						}
						else if(touch_event.action == ETA_TOUCH_MOVETO )
						{						
								touch_state = TOUCH_STATE_MOVE;
						}
						else
						{
							touch_state = TOUCH_STATE_NONE;
							touch_event.action = ETA_TOUCH_MOVETO	;
						}
					} else {
						PDEBUG("Wrong touch state in SYN_REPORT\n");
						return;
					}
					finger = 0;
					break;
				case SYN_MT_REPORT: /* 0x02 */
					//printk("----uch_state : %d---action : %d---\n",touch_state,touch_event.action);

					if (!finger) {

						if(touch_sent ==1)
						{
							touch_sent= 0;
							queue_work(touch_log_workqueue, &event_log_work);
							//printk("&&&&&handler: touch action 0x%x&&&&&\n",touch_event.action);
							PDEBUG("handler: touch x %u\n",
									touch_event.x);
							PDEBUG("handler: touch y %u\n",
									touch_event.y);
						}
						else
						{
							touch_event.action = ETA_TOUCH_UP;
							queue_work(touch_log_workqueue, &event_log_work);
							//printk("UUUUUUU: touch action 0x%x&&&&&\n",touch_event.action);

						}
					} else {
						PDEBUG("Ignore multi touch in SYN_REPORT\n");
						return;
					}
					finger++;
					touch_state = TOUCH_STATE_SYNC_MT;
					break;
			}
			break;
		case EV_ABS: /* 0x03 */
			switch (code) {
				case ABS_MT_TOUCH_MAJOR: /* 0x30 */
					if (!finger) {
						//printk("@@@@@touch_state : %d@@action : %d @@\n",touch_state,touch_event.action);
						if (value > 0) {
							if (touch_state == TOUCH_STATE_NONE)
							{
								 if(touch_event.action == ETA_TOUCH_MOVETO )//1. start touch press
								 {
								 	touch_state = TOUCH_STATE_DOWN;
									touch_event.action = ETA_TOUCH_DOWN;
								 }

							}
							else if(touch_state == TOUCH_STATE_DOWN)
							{
								if(touch_event.action == ETA_TOUCH_DOWN)
								{
								 	touch_state = TOUCH_STATE_MOVE;
									touch_event.action = ETA_TOUCH_MOVETO;							 	
								}
							}
							else if(touch_state == TOUCH_STATE_MOVE)
							{
								if(touch_event.action == ETA_TOUCH_MOVETO)
								{
								 	touch_state = TOUCH_STATE_MOVE;
									touch_event.action = ETA_TOUCH_MOVETO;							 	
								}
							}	

							touch_sent = 1;
							#if 0
							else if (touch_event.action == ETA_TOUCH_DOWN)
							{
								touch_event.action = ETA_TOUCH_MOVETO;
							}
							else if(touch_event.action == ETA_TOUCH_MOVETO )
							{
								touch_state = TOUCH_STATE_DOWN;
								touch_event.action = ETA_TOUCH_MOVETO;
							}
							else
							{
								touch_event.action = ETA_TOUCH_DOWN;
								touch_state = TOUCH_STATE_DOWN;
							}
							#endif
						} else {
							touch_event.action = ETA_TOUCH_UP;
							touch_state = TOUCH_STATE_NONE;
						}
					}
					break;
				case ABS_MT_POSITION_X: /* 0x35 */
					if (!finger) {
						touch_event.x = value * 1000 / 2055; // 2.055 = 986(max X)/480(width)
					}
					break;
				case ABS_MT_POSITION_Y: /* 0x36 */
					if (!finger) {
						touch_event.y = value * 1000 / 2055; // 2.055 = 1644(max Y)/800(height)
					}
					break;
			}
			break;
	}

		//printk("#####touch state : %d### touch action : %d####\n",touch_state,touch_event.action);

}
//hak.lee@lge.com end

int hub_start_touch_logging(void)
{
	int ret = 0;

	input_handler.name = "mtc_eta_log_touch";
	input_handler.connect = mtc_eta_event_log_connect;
	input_handler.disconnect = mtc_eta_event_log_disconnect;
	input_handler.event = mtc_eta_event_log_event;
	input_handler.id_table = mtc_eta_event_log_ids;
	ret = input_register_handler(&input_handler);
	if (ret != 0)
		printk("%s:fail to registers input handler\n", __func__);
//hak.lee@lge.com start

	touch_log_workqueue = create_rt_workqueue("touch ETA event wq");
	if (!touch_log_workqueue) {
		printk("%s:fail to create event workqueue\n", __func__);
		return -ENOMEM;
	}
//hak.lee@lge.com end
	INIT_WORK(&event_log_work, event_log_work_func);

	return ret;
}

void hub_stop_touch_logging(void)
{
	input_unregister_handler(&input_handler);
}

