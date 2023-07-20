/*
 * Copyright (c) 2022, smartmx - smartmx@qq.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef _LWIP_TASK_H_
#define _LWIP_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "ch32v30x_eth.h"
//#include "main.h"
#include "string.h"
#include <lwip/opt.h>
#include <lwip/arch.h>
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "ethernetif.h"
#include "netif/ethernet.h"
#include "ethernetif.h"
#include "lwip/def.h"
#include "lwip/timeouts.h"
#include "ch32v30x_rng.h"
#include "list.h"
//#include "memb.h"

#define USE10BASE_T        1
#define PHY_ADDRESS     0x01

#define ETH_RXBUFNB        4
#define ETH_TXBUFNB        4

#if 0
#define USE_LOOP_STRUCT    1
#else
#define USE_CHAIN_STRUCT   1
#endif

typedef struct
{
    void *next;
    u32 length;
    u32 buffer;
    ETH_DMADESCTypeDef *descriptor;
}FrameTypeDef;

extern ETH_DMADESCTypeDef  *DMATxDescToSet;
extern ETH_DMADESCTypeDef  *DMARxDescToGet;

extern ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];/* 鎺ユ敹鎻忚堪绗﹁〃 */
extern ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];/* 鍙戦�佹弿杩扮琛� */

//!!LIST_EXTERN(ch307_mac_rec);
//!!MEMB_EXTERN(ch307_mac_rec_frame_mem);

extern uint32_t ETH_TxPkt_ChainMode(u16 FrameLength);
void mac_send(uint8_t * content_ptr, uint16_t content_len);
extern void* ETH_RxPkt_ChainMode(void);
extern void PHY_control_pin_init(void);
extern void GETH_pin_init(void);
extern void FETH_pin_init(void);

extern void lwip_init_success_callback();

extern volatile uint8_t net_data_led_require; /*闇�瑕佷寒鐏渶姹�*/
#define NET_LED_PERIOD_MSECS 100
extern void net_led_tmr(void);

void ch307_init_phy();
//!!extern OS_TASK(os_lwip, void);

//!!extern OS_TASK(os_lwip_timeouts, void);

#endif /* _LWIP_TASK_H_ */
