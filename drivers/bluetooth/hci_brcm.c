/******************************************************************************
**
**  This software is contributed or developed by KYOCERA Corporation.
**  (C) 2012 KYOCERA Corporation
**
*******************************************************************************/
/******************************************************************************
**  
**  Broadcom Bluetooth Low Power UART protocol
**
** Copyright (C) 2009 Broadcom Corporation.
** 
** This software is licensed under the terms of the GNU General Public License,
** version 2, as published by the Free Software Foundation (the "GPL"), and may
** be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
** FITNESS FOR A PARTICULAR PURPOSE. See the GPL for more details.
** 
** A copy of the GPL is available at http://www.broadcom.com/licenses/GPLv2.php
** or by writing to the Free Software Foundation, Inc.,
** 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA
**
*******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>
#include <linux/timer.h>
#include <asm/gpio.h>

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>
#include <linux/serial_core.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"

#if 0
#ifndef CONFIG_BT_HCIUART_DEBUG
#undef  BT_DBG
#define BT_DBG( A... )
#endif
#endif

/* HCIBRCM commands */
#define HCIBRCM_GO_TO_SLEEP_IND	0x30
#define HCIBRCM_GO_TO_SLEEP_ACK	0x31
#define HCIBRCM_WAKE_UP_IND	0x32
#define HCIBRCM_WAKE_UP_ACK	0x33

/* HCIBRCM receiver States */
#define HCIBRCM_W4_PACKET_TYPE	0
#define HCIBRCM_W4_EVENT_HDR	1
#define HCIBRCM_W4_ACL_HDR	2
#define HCIBRCM_W4_SCO_HDR	3
#define HCIBRCM_W4_DATA		4

#define TIMER_PERIOD 100    /* 100 ms */
#define HOST_CONTROLLER_IDLE_TSH 4000  /* 4 s */
#define HOST_CONTROLLER_IDLE_TSH_SERIAL 5000  /* 5 s */

#define GPIO_BT_HOST_WAKE	(145)
#define INT_BT_HOST_WAKE	(MSM_GPIO_TO_INT(GPIO_BT_HOST_WAKE))

//#define BT_WAKE  22
#define BT_WAKE  93
 
void assert_bt_wake(void);
void deassert_bt_wake(void);
  
void assert_bt_wake()
{
#if 0
    gpio_set_value(BT_WAKE, 1);
#else
    int ret = 0;
/*  BT_ERR("assert_bt_wake()"); */
    BT_DBG("assert_bt_wake()");
    ret = gpio_request(BT_WAKE, NULL);
    if(ret < 0)
    {
        BT_ERR("Error: Failed to request BT_WAKE");
    }
    gpio_direction_output(BT_WAKE, 1);
    gpio_set_value(BT_WAKE, 1);
    gpio_free(BT_WAKE);
#endif
}
 
void deassert_bt_wake()
{
#if 0
   gpio_set_value(BT_WAKE, 0);
#else
    int ret = 0;
/*  BT_ERR("deassert_bt_wake()"); */
    BT_DBG("deassert_bt_wake()");
    ret = gpio_request(BT_WAKE, NULL);
    if(ret < 0)
    {
        BT_ERR("Error: Failed to request BT_WAKE");
    }
    gpio_direction_output(BT_WAKE, 0);
    gpio_set_value(BT_WAKE, 0);
    gpio_free(BT_WAKE);
#endif
}
 
/* HCIBRCM states */
enum hcibrcm_states_e {
	HCIBRCM_ASLEEP,
	HCIBRCM_ASLEEP_TO_AWAKE,
	HCIBRCM_AWAKE,
	HCIBRCM_AWAKE_TO_ASLEEP
};

struct hcibrcm_cmd {
	u8 cmd;
} __attribute__((packed));

struct brcm_struct {
	unsigned long rx_state;
	unsigned long rx_count;
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	spinlock_t hcibrcm_lock;		/* HCIBRCM state lock	*/
	unsigned long hcibrcm_state;	/* HCIBRCM power state	*/
    unsigned short is_there_activity;
    unsigned short inactive_period;
	struct sk_buff_head tx_wait_q;	/* HCIBRCM wait queue	*/
	unsigned long hcibrcm_state_serial;
	unsigned long inactive_period_serial;
};

static struct timer_list sleep_timer;

void msm_hs_set_mctrl(struct uart_port *uport, unsigned int mctrl);
static void serial_asleep(struct hci_uart *hu);
static void serial_awake(struct hci_uart *hu);

//
// timeout handler
//
void sleep_timer_function(unsigned long data)
{
//    struct brcm_struct *lbrcm = (struct brcm_struct *)data;
    struct hci_uart *hu = (struct hci_uart *)data;
    struct brcm_struct *lbrcm = (struct brcm_struct *)hu->priv;

    if (lbrcm->is_there_activity)
    {
        lbrcm->inactive_period = 0;
        lbrcm->is_there_activity = 0;
    }
    else
    {
        if (lbrcm->hcibrcm_state != HCIBRCM_ASLEEP)
        {
            lbrcm->inactive_period += TIMER_PERIOD;
            if (lbrcm->inactive_period >= HOST_CONTROLLER_IDLE_TSH)
            {
                // deassert BT_WAKE signal
                BT_DBG("Deassert wake signal, moves to ASLEEP");
                lbrcm->hcibrcm_state = HCIBRCM_ASLEEP;
                lbrcm->inactive_period = 0;
                deassert_bt_wake();
            }
        }
    }

    serial_asleep(hu);

    mod_timer(&sleep_timer, jiffies + TIMER_PERIOD*HZ/1000);
}

#ifdef CONFIG_SERIAL_MSM_HS
void msm_hs_request_clock_off(struct uart_port *uport);
void msm_hs_request_clock_on(struct uart_port *uport);
void sleep_timer_function(unsigned long data);

static void __brcm_msm_serial_clock_on(struct tty_struct *tty) {
	struct uart_state *state = tty->driver_data;
/*	struct uart_port *port = state->port;*/
	struct uart_port *port = state->uart_port;
	msm_hs_request_clock_on(port);
	msm_hs_set_mctrl(port, TIOCM_RTS);
}

static void __brcm_msm_serial_clock_request_off(struct tty_struct *tty) {
	struct uart_state *state = tty->driver_data;
/*	struct uart_port *port = state->port;*/
	struct uart_port *port = state->uart_port;

	msm_hs_set_mctrl(port, 0);
	msm_hs_request_clock_off(port);
}
#else
static inline void __brcm_msm_serial_clock_on(struct tty_struct *tty) {}
static inline void __brcm_msm_serial_clock_request_off(struct tty_struct *tty) {}
#endif

static inline int can_sleep(struct hci_uart *hu)
{
    /* check if GPIO_BT_HOST_WAKE are deasserted */
    return gpio_get_value(GPIO_BT_HOST_WAKE) ? 0 : 1;
}

static inline int can_wakeup(struct hci_uart *hu)
{
    /* check if rx buffer empty */
    return 1;   /* always true */
}

static void serial_asleep(struct hci_uart *hu)
{
    struct brcm_struct *lbrcm = (struct brcm_struct *)hu->priv;
    unsigned long flags;

    if (lbrcm->hcibrcm_state_serial == HCIBRCM_ASLEEP)
        return;

    lbrcm->inactive_period_serial += TIMER_PERIOD;
    if (lbrcm->inactive_period_serial < HOST_CONTROLLER_IDLE_TSH_SERIAL)
        return;

    if (!can_sleep(hu))
        return;

    spin_lock_irqsave(&lbrcm->hcibrcm_lock, flags);

    __brcm_msm_serial_clock_request_off(hu->tty);

    lbrcm->hcibrcm_state_serial = HCIBRCM_ASLEEP;

    spin_unlock_irqrestore(&lbrcm->hcibrcm_lock, flags);

    BT_DBG("bluetooth serial clock off");
}

static void serial_awake(struct hci_uart *hu)
{
    struct brcm_struct *lbrcm = (struct brcm_struct *)hu->priv;
    unsigned long flags;

    lbrcm->inactive_period_serial = 0;

    if (lbrcm->hcibrcm_state_serial != HCIBRCM_ASLEEP)
        return;

    if (!can_wakeup(hu))
        return;

    spin_lock_irqsave(&lbrcm->hcibrcm_lock, flags);

    __brcm_msm_serial_clock_on(hu->tty);

    lbrcm->hcibrcm_state_serial = HCIBRCM_AWAKE;

    spin_unlock_irqrestore(&lbrcm->hcibrcm_lock, flags);

    BT_DBG("bluetooth serial clock on");
}

static irqreturn_t bt_host_wake_isr(int irq, void *p)
{
    BT_DBG("IRQ(%d)", irq);

    serial_awake((struct hci_uart*)p);

    return IRQ_HANDLED;
}

static void enable_bt_host_wake(struct hci_uart *hu)
{
    int ret;

    ret = request_irq(INT_BT_HOST_WAKE, bt_host_wake_isr,
                      IRQF_TRIGGER_RISING, "bt_host_wake", hu);
    if (unlikely(ret)) {
        BT_ERR("GPIO(%d) request_irq error. ret = %d", GPIO_BT_HOST_WAKE, ret);
    }

    ret = enable_irq_wake(INT_BT_HOST_WAKE);
    if (unlikely(ret)) {
        BT_ERR("enable_irq_wake() failed for GPIO (%d)", GPIO_BT_HOST_WAKE);
    }
}

static void disable_bt_host_wake(struct hci_uart *hu)
{
    int ret;

    ret = disable_irq_wake(INT_BT_HOST_WAKE);
    if (unlikely(ret)) {
        BT_ERR("disable_irq_wake() failed for GPIO (%d)", GPIO_BT_HOST_WAKE);
    }

    free_irq(INT_BT_HOST_WAKE, hu);
}

/*
 * Builds and sends an HCIBRCM command packet.
 * These are very simple packets with only 1 cmd byte
 */
static int send_hcibrcm_cmd(u8 cmd, struct hci_uart *hu)
{
	int err = 0;
	struct sk_buff *skb = NULL;
	struct brcm_struct *brcm = hu->priv;
	struct hcibrcm_cmd *hcibrcm_packet;

	BT_DBG("hu %p cmd 0x%x", hu, cmd);

	/* allocate packet */
	skb = bt_skb_alloc(1, GFP_ATOMIC);
	if (!skb) {
		BT_ERR("cannot allocate memory for HCIBRCM packet");
		err = -ENOMEM;
		goto out;
	}

	/* prepare packet */
	hcibrcm_packet = (struct hcibrcm_cmd *) skb_put(skb, 1);
	hcibrcm_packet->cmd = cmd;
	skb->dev = (void *) hu->hdev;

	/* send packet */
	skb_queue_tail(&brcm->txq, skb);
out:
	return err;
}

/* Initialize protocol */
static int brcm_open(struct hci_uart *hu)
{
	struct brcm_struct *brcm;

	BT_DBG("hu %p", hu);

	brcm = kzalloc(sizeof(*brcm), GFP_ATOMIC);
	if (!brcm)
		return -ENOMEM;

	skb_queue_head_init(&brcm->txq);
	skb_queue_head_init(&brcm->tx_wait_q);
	spin_lock_init(&brcm->hcibrcm_lock);

	brcm->hcibrcm_state = HCIBRCM_AWAKE;
	brcm->hcibrcm_state_serial = HCIBRCM_AWAKE;

    init_timer(&sleep_timer);
    
    sleep_timer.expires = jiffies + TIMER_PERIOD*HZ/1000;
//    sleep_timer.data = (unsigned long)brcm;
    sleep_timer.data = (unsigned long)hu;
    sleep_timer.function = sleep_timer_function;
    
    add_timer(&sleep_timer);
  
    brcm->is_there_activity = 0;
	hu->priv = brcm;

    enable_bt_host_wake(hu);

	return 0;
}

/* Flush protocol data */
static int brcm_flush(struct hci_uart *hu)
{
	struct brcm_struct *brcm = hu->priv;

	BT_DBG("hu %p", hu);

    /* del_timer_sync(&sleep_timer); */
	skb_queue_purge(&brcm->tx_wait_q);
	skb_queue_purge(&brcm->txq);

	return 0;
}

/* Close protocol */
static int brcm_close(struct hci_uart *hu)
{
	struct brcm_struct *brcm = hu->priv;

	BT_DBG("hu %p", hu);

    disable_bt_host_wake(hu);

    del_timer_sync(&sleep_timer);
	skb_queue_purge(&brcm->tx_wait_q);
	skb_queue_purge(&brcm->txq);

	if (brcm->rx_skb)
		kfree_skb(brcm->rx_skb);

	hu->priv = NULL;

	kfree(brcm);

	return 0;
}

/*
 * internal function, which does common work of the device wake up process:
 * 1. places abrcm pending packets (waiting in tx_wait_q list) in txq list.
 * 2. changes internal state to HCIBRCM_AWAKE.
 * Note: assumes that hcibrcm_lock spinlock is taken,
 * shouldn't be cabrcmed otherwise!
 */
static void __brcm_do_awake(struct brcm_struct *brcm)
{
	struct sk_buff *skb = NULL;

	while ((skb = skb_dequeue(&brcm->tx_wait_q)))
		skb_queue_tail(&brcm->txq, skb);

	brcm->hcibrcm_state = HCIBRCM_AWAKE;
}

/*
 * Cabrcmed upon a wake-up-indication from the device
 */
static void brcm_device_want_to_wakeup(struct hci_uart *hu)
{
	unsigned long flags;
	struct brcm_struct *brcm = hu->priv;

	BT_DBG("hu %p", hu);

	/* lock hcibrcm state */
	spin_lock_irqsave(&brcm->hcibrcm_lock, flags);

	switch (brcm->hcibrcm_state) {
	case HCIBRCM_ASLEEP_TO_AWAKE:
		/*
		 * This state means that both the host and the BRF chip
		 * have simultaneously sent a wake-up-indication packet.
		 * Traditionaly, in this case, receiving a wake-up-indication
		 * was enough and an additional wake-up-ack wasn't needed.
		 * This has changed with the BRF6350, which does require an
		 * explicit wake-up-ack. Other BRF versions, which do not
		 * require an explicit ack here, do accept it, thus it is
		 * perfectly safe to always send one.
		 */
		BT_DBG("dual wake-up-indication");
		/* deliberate fabrcm-through - do not add break */
	case HCIBRCM_ASLEEP:
		/* Make sure clock is on - we may have turned clock off since
		 * receiving the wake up indicator
		 */
		__brcm_msm_serial_clock_on(hu->tty);
		/* acknowledge device wake up */
		if (send_hcibrcm_cmd(HCIBRCM_WAKE_UP_ACK, hu) < 0) {
			BT_ERR("cannot acknowledge device wake up");
			goto out;
		}
		break;
	default:
		/* any other state is ibrcmegal */
		BT_ERR("received HCIBRCM_WAKE_UP_IND in state %ld", brcm->hcibrcm_state);
		break;
	}

	/* send pending packets and change state to HCIBRCM_AWAKE */
	__brcm_do_awake(brcm);

out:
	spin_unlock_irqrestore(&brcm->hcibrcm_lock, flags);

	/* actuabrcmy send the packets */
	hci_uart_tx_wakeup(hu);
}

/*
 * Cabrcmed upon a sleep-indication from the device
 */
static void brcm_device_want_to_sleep(struct hci_uart *hu)
{
	unsigned long flags;
	struct brcm_struct *brcm = hu->priv;

	BT_DBG("hu %p", hu);

	/* lock hcibrcm state */
	spin_lock_irqsave(&brcm->hcibrcm_lock, flags);

	/* sanity check */
	if (brcm->hcibrcm_state != HCIBRCM_AWAKE)
		BT_ERR("ERR: HCIBRCM_GO_TO_SLEEP_IND in state %ld", brcm->hcibrcm_state);

	/* acknowledge device sleep */
	if (send_hcibrcm_cmd(HCIBRCM_GO_TO_SLEEP_ACK, hu) < 0) {
		BT_ERR("cannot acknowledge device sleep");
		goto out;
	}

	/* update state */
	brcm->hcibrcm_state = HCIBRCM_ASLEEP;

out:
	spin_unlock_irqrestore(&brcm->hcibrcm_lock, flags);

	/* actuabrcmy send the sleep ack packet */
	hci_uart_tx_wakeup(hu);

	spin_lock_irqsave(&brcm->hcibrcm_lock, flags);
	if (brcm->hcibrcm_state == HCIBRCM_ASLEEP)
		__brcm_msm_serial_clock_request_off(hu->tty);
	spin_unlock_irqrestore(&brcm->hcibrcm_lock, flags);
}

/*
 * Cabrcmed upon wake-up-acknowledgement from the device
 */
static void brcm_device_woke_up(struct hci_uart *hu)
{
	unsigned long flags;
	struct brcm_struct *brcm = hu->priv;

	BT_DBG("hu %p", hu);

	/* lock hcibrcm state */
	spin_lock_irqsave(&brcm->hcibrcm_lock, flags);

	/* sanity check */
	if (brcm->hcibrcm_state != HCIBRCM_ASLEEP_TO_AWAKE)
		BT_ERR("received HCIBRCM_WAKE_UP_ACK in state %ld", brcm->hcibrcm_state);

	/* send pending packets and change state to HCIBRCM_AWAKE */
	__brcm_do_awake(brcm);

	spin_unlock_irqrestore(&brcm->hcibrcm_lock, flags);

	/* actuabrcmy send the packets */
	hci_uart_tx_wakeup(hu);
}

/* Enqueue frame for transmittion (padding, crc, etc) */
/* may be cabrcmed from two simultaneous tasklets */
static int brcm_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	//unsigned long flags = 0;
	struct brcm_struct *brcm = hu->priv;

	BT_DBG("hu %p skb %p", hu, skb);

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);
#if 0
	/* lock hcibrcm state */
	spin_lock_irqsave(&brcm->hcibrcm_lock, flags);

	/* act according to current state */
	switch (brcm->hcibrcm_state) {
	case HCIBRCM_AWAKE:
		BT_DBG("device awake, sending normabrcmy");
		skb_queue_tail(&brcm->txq, skb);
		break;
	case HCIBRCM_ASLEEP:
		BT_DBG("device asleep, waking up and queueing packet");
		__brcm_msm_serial_clock_on(hu->tty);
		/* save packet for later */
		skb_queue_tail(&brcm->tx_wait_q, skb);
		/* awake device */
		if (send_hcibrcm_cmd(HCIBRCM_WAKE_UP_IND, hu) < 0) {
			BT_ERR("cannot wake up device");
			break;
		}
		brcm->hcibrcm_state = HCIBRCM_ASLEEP_TO_AWAKE;
		break;
	case HCIBRCM_ASLEEP_TO_AWAKE:
		BT_DBG("device waking up, queueing packet");
		/* transient state; just keep packet for later */
		skb_queue_tail(&brcm->tx_wait_q, skb);
		break;
	default:
		BT_ERR("ibrcmegal hcibrcm state: %ld (losing packet)", brcm->hcibrcm_state);
		kfree_skb(skb);
		break;
	}

	spin_unlock_irqrestore(&brcm->hcibrcm_lock, flags);
#endif

    serial_awake(hu);

	skb_queue_tail(&brcm->txq, skb);

        brcm->is_there_activity = 1;
        if (brcm->hcibrcm_state == HCIBRCM_ASLEEP)
        {
            BT_DBG("Asserting wake signal, moves to AWAKE");
            /* assert BT_WAKE signal */
            assert_bt_wake();
            brcm->hcibrcm_state = HCIBRCM_AWAKE;
        }

	return 0;
}

static inline int brcm_check_data_len(struct brcm_struct *brcm, int len)
{
	register int room = skb_tailroom(brcm->rx_skb);

	BT_DBG("len %d room %d", len, room);

	if (!len) {
		hci_recv_frame(brcm->rx_skb);
	} else if (len > room) {
		BT_ERR("Data length is too large");
		kfree_skb(brcm->rx_skb);
	} else {
		brcm->rx_state = HCIBRCM_W4_DATA;
		brcm->rx_count = len;
		return len;
	}

	brcm->rx_state = HCIBRCM_W4_PACKET_TYPE;
	brcm->rx_skb   = NULL;
	brcm->rx_count = 0;

	return 0;
}

/* Recv data */
static int brcm_recv(struct hci_uart *hu, void *data, int count)
{
	struct brcm_struct *brcm = hu->priv;
	register char *ptr;
	struct hci_event_hdr *eh;
	struct hci_acl_hdr   *ah;
	struct hci_sco_hdr   *sh;
	register int len, type, dlen;

	BT_DBG("hu %p count %d rx_state %ld rx_count %ld", hu, count, brcm->rx_state, brcm->rx_count);

    serial_awake(hu);

    brcm->is_there_activity = 1;
    if (brcm->hcibrcm_state == HCIBRCM_ASLEEP)
    {
        BT_DBG("Assert wake signal, moves to AWAKE");
        /* assert BT_WAKE signal */
        assert_bt_wake();
        brcm->hcibrcm_state = HCIBRCM_AWAKE;
    }
	ptr = data;
	while (count) {
		if (brcm->rx_count) {
			len = min_t(unsigned int, brcm->rx_count, count);
			memcpy(skb_put(brcm->rx_skb, len), ptr, len);
			brcm->rx_count -= len; count -= len; ptr += len;

			if (brcm->rx_count)
				continue;

			switch (brcm->rx_state) {
			case HCIBRCM_W4_DATA:
				BT_DBG("Complete data");
				hci_recv_frame(brcm->rx_skb);

				brcm->rx_state = HCIBRCM_W4_PACKET_TYPE;
				brcm->rx_skb = NULL;
				continue;

			case HCIBRCM_W4_EVENT_HDR:
				eh = (struct hci_event_hdr *) brcm->rx_skb->data;

				BT_DBG("Event header: evt 0x%2.2x plen %d", eh->evt, eh->plen);

				brcm_check_data_len(brcm, eh->plen);
				continue;

			case HCIBRCM_W4_ACL_HDR:
				ah = (struct hci_acl_hdr *) brcm->rx_skb->data;
				dlen = __le16_to_cpu(ah->dlen);

				BT_DBG("ACL header: dlen %d", dlen);

				brcm_check_data_len(brcm, dlen);
				continue;

			case HCIBRCM_W4_SCO_HDR:
				sh = (struct hci_sco_hdr *) brcm->rx_skb->data;

				BT_DBG("SCO header: dlen %d", sh->dlen);

				brcm_check_data_len(brcm, sh->dlen);
				continue;
			}
		}

		/* HCIBRCM_W4_PACKET_TYPE */
		switch (*ptr) {
		case HCI_EVENT_PKT:
			BT_DBG("Event packet");
			brcm->rx_state = HCIBRCM_W4_EVENT_HDR;
			brcm->rx_count = HCI_EVENT_HDR_SIZE;
			type = HCI_EVENT_PKT;
			break;

		case HCI_ACLDATA_PKT:
			BT_DBG("ACL packet");
			brcm->rx_state = HCIBRCM_W4_ACL_HDR;
			brcm->rx_count = HCI_ACL_HDR_SIZE;
			type = HCI_ACLDATA_PKT;
			break;

		case HCI_SCODATA_PKT:
			BT_DBG("SCO packet");
			brcm->rx_state = HCIBRCM_W4_SCO_HDR;
			brcm->rx_count = HCI_SCO_HDR_SIZE;
			type = HCI_SCODATA_PKT;
			break;

		/* HCIBRCM signals */
		case HCIBRCM_GO_TO_SLEEP_IND:
			BT_DBG("HCIBRCM_GO_TO_SLEEP_IND packet");
			brcm_device_want_to_sleep(hu);
			ptr++; count--;
			continue;

		case HCIBRCM_GO_TO_SLEEP_ACK:
			/* shouldn't happen */
			BT_ERR("received HCIBRCM_GO_TO_SLEEP_ACK (in state %ld)", brcm->hcibrcm_state);
			ptr++; count--;
			continue;

		case HCIBRCM_WAKE_UP_IND:
			BT_DBG("HCIBRCM_WAKE_UP_IND packet");
			brcm_device_want_to_wakeup(hu);
			ptr++; count--;
			continue;

		case HCIBRCM_WAKE_UP_ACK:
			BT_DBG("HCIBRCM_WAKE_UP_ACK packet");
			brcm_device_woke_up(hu);
			ptr++; count--;
			continue;

		default:
			BT_ERR("Unknown HCI packet type %2.2x", (__u8)*ptr);
			hu->hdev->stat.err_rx++;
			ptr++; count--;
			continue;
		};

		ptr++; count--;

		/* Abrcmocate packet */
		brcm->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
		if (!brcm->rx_skb) {
			BT_ERR("Can't allocate mem for new packet");
			brcm->rx_state = HCIBRCM_W4_PACKET_TYPE;
			brcm->rx_count = 0;
			return 0;
		}

		brcm->rx_skb->dev = (void *) hu->hdev;
		bt_cb(brcm->rx_skb)->pkt_type = type;
	}

	return count;
}

static struct sk_buff *brcm_dequeue(struct hci_uart *hu)
{
	struct brcm_struct *brcm = hu->priv;
	return skb_dequeue(&brcm->txq);
}

static struct hci_uart_proto brcmp = {
	.id		= HCI_UART_BRCM,
	.open		= brcm_open,
	.close		= brcm_close,
	.recv		= brcm_recv,
	.enqueue	= brcm_enqueue,
	.dequeue	= brcm_dequeue,
	.flush		= brcm_flush,
};

int brcm_init(void)
{
	int err = hci_uart_register_proto(&brcmp);

	if (!err)
		BT_INFO("HCIBRCM protocol initialized");
	else
		BT_ERR("HCIBRCM protocol registration failed");

	return err;
}

int brcm_deinit(void)
{
	return hci_uart_unregister_proto(&brcmp);
}

