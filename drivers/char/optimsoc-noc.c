/**
 * Copyright (c) 2012-2017 by the author(s)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * =================================================================
 *
 * Linux driver for OpTiMSoC network-on-chip adapter.
 *
 * Author(s):
 *   Pedro H. Penna <pedrohenriquepenna@gmail.com>
 *   Stefan Wallentowitz <stefan.wallentowitz@tum.de>
 */

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>

/**
 * Maximum number of endpoints in a tile.
 */
#define NR_ENDPOINTS 2

/*
 * Device name.
 */
#define OPTIMSOC_NA_NAME "optimsoc-noc"

/*
 * NoC adapter IRQ number.
 */
#define OPTIMSOC_NA_IRQ 4

/*
 * Base hardware address for NoC adapter.
 */
#define OPTIMSOC_NA_BASE_HWADDR 0xe0000000

/**
 * Base virtual address for NoC adapter
 */
static uint32_t OPTIMSOC_NA_BASE_VADDR = 0;

/*
 * NoC adapter memory size bytes.
 */
#define OPTIMSOC_NA_MEM_SIZE (8*1024*1024)

/* 
 * Address of memory maped registers.
 */
#define REG_NUM_CT (OPTIMSOC_NA_BASE_VADDR + 0x28)
#define BASE       (OPTIMSOC_NA_BASE_VADDR + 0x100000)
#define REG_NUMEP  BASE
#define EP_BASE    BASE + 0x2000
#define EP_OFFSET  0x2000
#define REG_SEND   0x0
#define REG_RECV   0x0
#define REG_ENABLE 0x4

/**
 * Retrieves a word at a target address.
 */
#define REG32(x) (*((uint32_t *)(x)))

/*
 * Buffer size (in bytes).
 */
#define OPTIMSOC_NA_BUFFER_SIZE 32

/*
 * Success.
 */
#define SUCCESS 0

/*
 * Failure
 */
#define FAILURE -1

/*
 * Major device number.
 */
static int major;

/*
 * Number of end points.
 */
static uint32_t nr_endpoints;

static struct
{
	int nopen;                /* Number of processes using this device. */
	int head;                 /* Buffer head.                           */
	int tail;                 /* Buffer tail.                           */
	int data_received;        /* Was any data received?                 */
	uint32_t *buffer;         /* Input buffer.                          */
	wait_queue_head_t wqueue; /* Wait queue of processes.               */
} adapters[NR_ENDPOINTS];

/*
 * enable() - Enables an endpoint.
 */
static void enable(unsigned ep)
{
	uint32_t *addr;

#ifdef DEBUG
	printk(KERN_INFO "enable: %x", ep);
#endif

	addr = (uint32_t *)(EP_BASE + ep*EP_OFFSET + REG_ENABLE);

	*addr = 1;
}

/*
 * send() - Sends a word.
 */
static void send(unsigned ep, uint32_t word)
{
	uint32_t *addr;

#ifdef DEBUG
	printk(KERN_INFO "send: %x", word);
#endif

	addr = (uint32_t *)(EP_BASE + ep*EP_OFFSET + REG_SEND);

	*addr = word;
}

/*
 * receive() - Receives a word.
 */
static uint32_t receive(unsigned ep)
{
	uint32_t *addr;

	addr = (uint32_t *)(EP_BASE + ep*EP_OFFSET + REG_RECV);

#ifdef DEBUG
	printk(KERN_INFO "send: %x", *addr);
#endif

	return (*addr);
}


/*
 * irq_handler() - Handles a NoC IRQ.
 */
static irqreturn_t irq_handler(int irq, void *opaque)
{
	int i;

	((void) opaque);

	for (i = 0; i < nr_endpoints; i++)
	{
		uint32_t word;

		if (adapters[i].nopen <= 0)
			continue;

		word = receive(i);

		printk(KERN_INFO "receive: %x", word);

		/* Drop packet. */
		if ((adapters[i].tail + 1)%OPTIMSOC_NA_BUFFER_SIZE == adapters[i].head)
			break;

		adapters[i].buffer[adapters[i].tail] = word;

		adapters[i].tail = (adapters[i].tail + 1)%OPTIMSOC_NA_BUFFER_SIZE;

		adapters[i].data_received = 1;

		/* Wakeup sleeping processes. */
		if (adapters[i].data_received)
			wake_up(&adapters[i].wqueue);
	}

	return (IRQ_HANDLED);
}

/*
 * optimsoc_noc_open() - Opens the NoC device.
 */
static int optimsoc_noc_open(struct inode *inode, struct file *file)
{
	unsigned minor;

	minor = iminor(inode);

	printk(KERN_INFO "%s: open device %d", OPTIMSOC_NA_NAME, minor);

	/* Device already in use. */
	if (adapters[minor].nopen > 0)
		return (-EBUSY);

	/* Grab resources. */
	adapters[minor].buffer = kmalloc(OPTIMSOC_NA_BUFFER_SIZE, GFP_KERNEL);
	if (adapters[minor].buffer == NULL)
		return (-ENOMEM);

	adapters[minor].nopen++;

	enable(minor);

	try_module_get(THIS_MODULE);

	return (SUCCESS);
}

/*
 * optimsoc_noc_release() - Closes the NoC device.
 */
static int optimsoc_noc_release(struct inode *inode, struct file *file)
{
	unsigned minor;

	minor = iminor(inode);

	printk(KERN_INFO "%s: close device %d", OPTIMSOC_NA_NAME, minor);

	/* Release resources. */
	adapters[minor].nopen--;
	kfree(adapters[minor].buffer);
	adapters[minor].buffer = NULL;

	module_put(THIS_MODULE);

	return (SUCCESS);
}

/*
 * optimsoc_noc_read() - Reads bytes from the NoC device.
 */
static ssize_t optimsoc_noc_read(struct file *filp, char *buffer, size_t length, loff_t * offset)
{
	size_t i;
	unsigned minor;

	((void) offset);

	minor = iminor(filp->f_inode);
	if (minor >= nr_endpoints)
		return (-EINVAL);

	printk(KERN_INFO "%s: read from device %d", OPTIMSOC_NA_NAME, minor);

	/* Read bytes. */
	for (i = 0; i < length; /* noop*/)
	{
		size_t n;
		uint32_t word;
		size_t count;

		/* Wait for data, */
		if (adapters[minor].head == adapters[minor].tail)
		{
			wait_event_interruptible(adapters[minor].wqueue, adapters[minor].data_received);
			adapters[minor].data_received = !adapters[minor].data_received;
		}

		word = adapters[minor].buffer[adapters[minor].head];
		adapters[minor].head = (adapters[minor].head + 1)%OPTIMSOC_NA_BUFFER_SIZE;

		n = ((i + 4) <= length) ? 4 : (length  - i);

		count = copy_to_user(&buffer[i], &word, n);

		printk(KERN_INFO "receive: %x", word);

		if (count != 0)
			return (i + (n - count));

		i += 4;
	}

	return (length);
}

/*
 * optimsoc_noc_write() - Writes bytes from the NoC device.
 */
static ssize_t optimsoc_noc_write(struct file *filp, const char *buff, size_t len, loff_t * off)
{
	int i;
	unsigned minor;

	((void) off);

	/* Get minor device. */
	minor = iminor(filp->f_inode);
	if (minor >= nr_endpoints)
		return (-EINVAL);

	printk(KERN_INFO "%s: write to device %d", OPTIMSOC_NA_NAME, minor);

	/* Copy data to temporary buffer. */
	for (i = 0; i < len; /* noop. */)
	{
		size_t n;
		size_t count;
		uint32_t word;

		n = ((i + 4) <= len) ? 4 : (len  - i);

		count = copy_from_user(&word, &buff[i], n);
		send(minor, word);

		if (count != 0)
			return (i + (n - count));

		i += 4;
	}

	return (len);
}

static struct file_operations fops = {
	.read = optimsoc_noc_read,
	.write = optimsoc_noc_write,
	.open = optimsoc_noc_open,
	.release = optimsoc_noc_release
};

/*
 * optimsoc_module_init() - Initializes the device driver module.
 */
static int __init optimsoc_module_init(void)
{
	int i;
	int ret;

	printk(KERN_INFO "%s: loading driver", OPTIMSOC_NA_NAME);

	/* Register interrupt device driver. */
	major = register_chrdev(0, OPTIMSOC_NA_NAME, &fops);
	if (major < 0)
	{
		printk(KERN_ALERT "%s: failed to register driver", OPTIMSOC_NA_NAME);
		goto error0;
	}

	printk(KERN_INFO "%s: got major number %d", OPTIMSOC_NA_NAME, major);


	/* Register interrupt handler. */
	ret = request_irq(OPTIMSOC_NA_IRQ, irq_handler, 0, OPTIMSOC_NA_NAME "-handler", (void *)(irq_handler));
	if (ret != SUCCESS)
	{
		printk(KERN_ALERT "%s: failed to register interrupt handler", OPTIMSOC_NA_NAME);
		goto error1;
	}
	printk(KERN_INFO "%s: interrupt handler registered at IRQ %d", OPTIMSOC_NA_NAME, OPTIMSOC_NA_IRQ);


	OPTIMSOC_NA_BASE_VADDR = (uint32_t) ioremap_nocache(OPTIMSOC_NA_BASE_HWADDR, OPTIMSOC_NA_MEM_SIZE);
	if (OPTIMSOC_NA_BASE_VADDR == (uint32_t)NULL)
	{
		printk(KERN_ALERT "%s: failed to allocate buffers", OPTIMSOC_NA_NAME);
		goto error1;
	}

    nr_endpoints = REG32(REG_NUMEP);

	printk(KERN_INFO "%s: %d endpoints detected", OPTIMSOC_NA_NAME, nr_endpoints);

	/* Initialize devices. */
	for (i = 0; i < NR_ENDPOINTS; i++)
	{
		adapters[i].nopen = 0;
		adapters[i].data_received = 0;
		adapters[i].head = 0;
		adapters[i].tail = 0;
		adapters[i].buffer = NULL;
		init_waitqueue_head(&adapters[i].wqueue);
	}

	return (SUCCESS);

error1:
	unregister_chrdev(major, OPTIMSOC_NA_NAME);
error0:
	return (FAILURE);
}

/*
 * optimsoc_module_cleanup() - Unloads the device driver module.
 */
static void __exit optimsoc_module_cleanup(void)
{
	int i;

	printk(KERN_INFO "%s: unloading driver", OPTIMSOC_NA_NAME);

	/* Release resources. */
	for (i = 0; i < NR_ENDPOINTS; i++)
	{
		if (adapters[i].buffer != NULL)
			kfree(adapters[i].buffer);
	}
	
	unregister_chrdev(major, OPTIMSOC_NA_NAME);
}

module_init(optimsoc_module_init);
module_exit(optimsoc_module_cleanup);

MODULE_INFO(intree, "Y");
MODULE_LICENSE("GPLv2");
MODULE_AUTHOR("Pedro H. Penna");
