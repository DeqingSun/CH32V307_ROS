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

#include "lwip_task.h"

/* Globe variable */
//!!LIST(ch307_mac_rec);
//!!MEMB(ch307_mac_rec_frame_mem, FrameTypeDef, ETH_RXBUFNB);

__attribute__ ((aligned(4))) ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];/* 鎺ユ敹鎻忚堪绗﹁〃 */
__attribute__ ((aligned(4))) ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];/* 鍙戦�佹弿杩扮琛� */
__attribute__ ((aligned(4))) uint8_t Rx_Buff[ETH_RXBUFNB][ETH_MAX_PACKET_SIZE];/* 鎺ユ敹闃熷垪 */
__attribute__ ((aligned(4))) uint8_t Tx_Buff[ETH_TXBUFNB][ETH_MAX_PACKET_SIZE];/* 鍙戦�侀槦鍒� */

/* extern variable */

/* Macro */
#ifndef ETH_ERROR
#define  ETH_ERROR              ((uint32_t)0)
#endif

#ifndef ETH_SUCCESS
#define  ETH_SUCCESS            ((uint32_t)1)
#endif

#define  ETH_DMARxDesc_FrameLengthShift           16

#define define_O(a,b) \
GPIO_InitStructure.GPIO_Pin = b;\
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;\
GPIO_Init(a, &GPIO_InitStructure)

#define define_I(a,b) \
GPIO_InitStructure.GPIO_Pin = b;\
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;\
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;\
GPIO_Init(a, &GPIO_InitStructure)


/*********************************************************************
 * @fn      Ethernet_LED_LINKSET
 *
 * @brief   set eth link led,setbit 0 or 1,the link led turn on or turn off
 *
 * @return  none
 */
void Ethernet_LED_LINKSET(uint8_t setbit)
{
     if(setbit){
         GPIO_SetBits(GPIOC, GPIO_Pin_0);
     }
     else {
         GPIO_ResetBits(GPIOC, GPIO_Pin_0);
    }
}


/*********************************************************************
 * @fn      Ethernet_LED_DATASET
 *
 * @brief   set eth data led,setbit 0 or 1,the data led turn on or turn off
 *
 * @return  none
 */
void Ethernet_LED_DATASET(uint8_t setbit)
{
     if(setbit){
         GPIO_SetBits(GPIOC, GPIO_Pin_1);
     }
     else {
         GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    }
}


/*********************************************************************
 * @fn      CH307_INIT_PHY
 *
 * @brief   init ch307 on chip 10M-phy
 *
 * @return  none
 */
void ch307_init_phy()
{
    // SET_MCO
    GPIO_InitTypeDef GPIO={0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

    GPIO.GPIO_Pin = GPIO_Pin_8; //not sure why A8!!
    GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA,&GPIO);

    RCC_PLL3Cmd(DISABLE);
    RCC_PREDIV2Config(RCC_PREDIV2_Div2);
    RCC_PLL3Config(RCC_PLL3Mul_15);
    RCC_MCOConfig(RCC_MCO_PLL3CLK);
    RCC_PLL3Cmd(ENABLE);

    vTaskDelay(100);

    if(RESET == RCC_GetFlagStatus(RCC_FLAG_PLL3RDY))
    {
        printf("Wait for PLL3 ready.\n");
        vTaskDelay(500);
    }
    printf("PLL3 is ready.\n");
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

    // Ethernet LED Configuration
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    GPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO);
    Ethernet_LED_LINKSET(1);// turn off link led.
    Ethernet_LED_DATASET(1);// turn off data led.

    // Ethernet_Configuration
    ETH_InitTypeDef ETH_InitStructure;
    uint32_t timeout;

    memset(&ETH_InitStructure, 0, sizeof(ETH_InitTypeDef));
    // Enable Ethernet MAC clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ETH_MAC|RCC_AHBPeriph_ETH_MAC_Tx|RCC_AHBPeriph_ETH_MAC_Rx,ENABLE);

#ifdef USE10BASE_T
    // Enable internal 10BASE-T PHY
    EXTEN->EXTEN_CTR |=EXTEN_ETH_10M_EN;// enable 10M ethernet phy
#endif

#ifdef USE_GIGA_MAC
    // Enable 1G MAC*/
    EXTEN->EXTEN_CTR |= EXTEN_ETH_RGMII_SEL;// enable 1G ethernet phy
    RCC_ETH1GCLKConfig(RCC_ETH1GCLKSource_PB1_IN);// use external 125Mhz input
    RCC_ETH1G_125Mcmd(ENABLE);// enable 125M clock

    //  Enable RGMII GPIO
    GETH_pin_init();
#endif

#ifdef USE_FAST_MAC
    //  Enable MII or RMII GPIO
    FETH_pin_init();
#endif

    // Reset ETHERNET on AHB Bus
    ETH_DeInit();

    /* Software reset */
    ETH_SoftwareReset();

    /* Wait for software reset */
    timeout=10;

    while(ETH->DMABMR & ETH_DMABMR_SR)
    {
        timeout--;
        if(timeout==0)
        {
            printf("Error:Eth soft-reset timeout!\nPlease check RGMII TX & RX clock line.\n");
        }
        vTaskDelay(10);
    }

    // ETHERNET Configuration ------------------------------------------------------
    // Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter
    ETH_StructInit(&ETH_InitStructure);
    // Fill ETH_InitStructure parametrs
    //------------------------   MAC   -----------------------------------
    ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
    ETH_InitStructure.ETH_Speed = ETH_Speed_1000M;
    ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable  ;
    ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
    ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
    ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
    ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Enable;
    ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
    ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Enable;
    ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
    ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
#ifdef CHECKSUM_BY_HARDWARE
    ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif
    //------------------------   DMA   -----------------------------------
    // When we use the Checksum offload feature, we need to enable the Store and Forward mode:
    //the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum,
    //if the checksum is OK the DMA can handle the frame otherwise the frame is dropped
    ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable;
    ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
    ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;
    ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Enable;
    ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Enable;
    ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Disable;
    ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
    ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
    ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
    ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

    // Configure Ethernet
    uint32_t tmpreg = 0;
    static uint16_t RegValue = 0;

    //---------------------- phy config -------------------
    // set SMI interface clk, main clk div by 42
    tmpreg = ETH->MACMIIAR;
    tmpreg &= MACMIIAR_CR_MASK;
    tmpreg |= (uint32_t)ETH_MACMIIAR_CR_Div42;
    ETH->MACMIIAR = (uint32_t)tmpreg;

    //reset phy
    ETH_WritePHYRegister(PHY_ADDRESS, PHY_BCR, PHY_Reset);//reset phy

    vTaskDelay(100); // delay of rest

    timeout=10000;//max timeout 10S
    RegValue = 0;

    while((ETH_ReadPHYRegister(PHY_ADDRESS, PHY_BCR) & (PHY_Reset)))
    {
        timeout--;
        if(timeout<=0)
        {
          printf("Error:Wait phy software timeout!\nPlease cheak PHY/MID.\nProgram has been blocked!\n");
          while(1);
        }
        vTaskDelay(1);
    }

    // wait till phy establish link with other side
    timeout=10000;//max timeout 10S
    RegValue = 0;

    while((ETH_ReadPHYRegister(PHY_ADDRESS, PHY_BSR) & (PHY_Linked_Status)) == 0)
    {
        timeout--;
        if(timeout<=0)
        {
          printf("Error:Wait phy linking timeout!\nPlease cheak MID.\nProgram has been blocked!\n");
          while(1);
        }
        vTaskDelay(1);
    }

    // wait till phy to negotiate automatically
    timeout=10000;//max timeout 10S
    RegValue = 0;

    while( (ETH_ReadPHYRegister(PHY_ADDRESS, PHY_BSR) & PHY_AutoNego_Complete) == 0 )
    {
        timeout--;
        if(timeout<=0)
        {
          printf("Error:Wait phy auto-negotiation complete timeout!\nPlease cheak MID.\nProgram has been blocked!\n");
          while(1);
        }
        vTaskDelay(1);
    }

    RegValue = ETH_ReadPHYRegister( PHY_ADDRESS, 0x10 );
    printf("PHY_SR value:%04x.\n", RegValue);

    if( RegValue & (1<<2) )
    {
        ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
      printf("Full Duplex.\n");
    }
    else
    {
        ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
      printf("Half Duplex.\n");
    }
    ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
    if(RegValue & (1<<3))
    {
      printf("Loopback_10M \n");
    }
    else
    {
    }

    vTaskDelay(100);

    //turn on Link LED
    Ethernet_LED_LINKSET(0);

    //------------------------ MAC reg config ----------------------- --------------------
    // MACCCR in RGMII mode has the part to adjust RGMII timing, please note
    tmpreg = ETH->MACCR;
    tmpreg &= MACCR_CLEAR_MASK;
    tmpreg |= (uint32_t)(ETH_InitStructure.ETH_Watchdog |
                    ETH_InitStructure.ETH_Jabber |
                    ETH_InitStructure.ETH_InterFrameGap |
                    ETH_InitStructure.ETH_CarrierSense |
                    ETH_InitStructure.ETH_Speed |
                    ETH_InitStructure.ETH_ReceiveOwn |
                    ETH_InitStructure.ETH_LoopbackMode |
                    ETH_InitStructure.ETH_Mode |
                    ETH_InitStructure.ETH_ChecksumOffload |
                    ETH_InitStructure.ETH_RetryTransmission |
                    ETH_InitStructure.ETH_AutomaticPadCRCStrip |
                    ETH_InitStructure.ETH_BackOffLimit |
                    ETH_InitStructure.ETH_DeferralCheck);
    // write MAC control reg
    ETH->MACCR = (uint32_t)tmpreg;
  #ifdef USE10BASE_T
    ETH->MACCR|=ETH_Internal_Pull_Up_Res_Enable;/* 鍚敤鍐呴儴涓婃媺  */
  #endif
    ETH->MACFFR = (uint32_t)(ETH_InitStructure.ETH_ReceiveAll |
                            ETH_InitStructure.ETH_SourceAddrFilter |
                            ETH_InitStructure.ETH_PassControlFrames |
                            ETH_InitStructure.ETH_BroadcastFramesReception |
                            ETH_InitStructure.ETH_DestinationAddrFilter |
                            ETH_InitStructure.ETH_PromiscuousMode |
                            ETH_InitStructure.ETH_MulticastFramesFilter |
                            ETH_InitStructure.ETH_UnicastFramesFilter);
    //--------------- ETHERNET MACHTHR and MACHTLR Configuration ---------------
    // Write to ETHERNET MACHTHR
    ETH->MACHTHR = (uint32_t)ETH_InitStructure.ETH_HashTableHigh;
    // Write to ETHERNET MACHTLR
    ETH->MACHTLR = (uint32_t)ETH_InitStructure.ETH_HashTableLow;
    //----------------------- ETHERNET MACFCR Configuration --------------------
    // Get the ETHERNET MACFCR value
    tmpreg = ETH->MACFCR;
    // Clear xx bits
    tmpreg &= MACFCR_CLEAR_MASK;

    tmpreg |= (uint32_t)((ETH_InitStructure.ETH_PauseTime << 16) |
                     ETH_InitStructure.ETH_ZeroQuantaPause |
                     ETH_InitStructure.ETH_PauseLowThreshold |
                     ETH_InitStructure.ETH_UnicastPauseFrameDetect |
                     ETH_InitStructure.ETH_ReceiveFlowControl |
                     ETH_InitStructure.ETH_TransmitFlowControl);
    ETH->MACFCR = (uint32_t)tmpreg;

    ETH->MACVLANTR = (uint32_t)(ETH_InitStructure.ETH_VLANTagComparison |
                               ETH_InitStructure.ETH_VLANTagIdentifier);

    tmpreg = ETH->DMAOMR;
    // Clear xx bits
    tmpreg &= DMAOMR_CLEAR_MASK;

    tmpreg |= (uint32_t)(ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame |
                    ETH_InitStructure.ETH_ReceiveStoreForward |
                    ETH_InitStructure.ETH_FlushReceivedFrame |
                    ETH_InitStructure.ETH_TransmitStoreForward |
                    ETH_InitStructure.ETH_TransmitThresholdControl |
                    ETH_InitStructure.ETH_ForwardErrorFrames |
                    ETH_InitStructure.ETH_ForwardUndersizedGoodFrames |
                    ETH_InitStructure.ETH_ReceiveThresholdControl |
                    ETH_InitStructure.ETH_SecondFrameOperate);
    ETH->DMAOMR = (uint32_t)tmpreg;

    ETH->DMABMR = (uint32_t)(ETH_InitStructure.ETH_AddressAlignedBeats |
                            ETH_InitStructure.ETH_FixedBurst |
                            ETH_InitStructure.ETH_RxDMABurstLength | // !! if 4xPBL is selected for Tx or Rx it is applied for the other
                            ETH_InitStructure.ETH_TxDMABurstLength |
                           (ETH_InitStructure.ETH_DescriptorSkipLength << 2) |
                            ETH_InitStructure.ETH_DMAArbitration |
                            ETH_DMABMR_USP);

    // Enable the Ethernet Rx Interrupt
    ETH_DMAITConfig(ETH_DMA_IT_NIS
                  | ETH_DMA_IT_R
                  | ETH_DMA_IT_T
                  ,ENABLE);

    NVIC_EnableIRQ(ETH_IRQn);
    ETH_DMATxDescChainInit(DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
    ETH_DMARxDescChainInit(DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
    ETH_Start();

}


struct netif WCH_NetIf;

ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

static void wait_dhcp(void *arg);

static void wait_dhcp(void *arg)
{
    if(ip_addr_cmp(&(WCH_NetIf.ip_addr),&ipaddr))   //绛夊緟dhcp鍒嗛厤鐨刬p鏈夋晥
    {
        sys_timeout(50, wait_dhcp, NULL);
    }
    else
    {
        lwip_init_success_callback(&(WCH_NetIf.ip_addr)); /* ip鍒嗛厤鎴愬姛鍥炶皟锛岀敤鎴峰湪姝ゅ鍔犲叧浜庣綉缁滆繘绋嬬殑鍒濆鍖栧嚱鏁� */
    }
}

//OS_TASK(os_lwip, void)
//{
//    OS_TASK_START(os_lwip);
//
//    /* mem_init of lwip, init outside the lwip_init for user to using outside. */
//    mem_init();
//
//    /* call sub task ch307 init phy */
//    OS_CALL_SUBNT(CH307_INIT_PHY);
//    printf("CH307_INIT_PHY ok\n");
//
//    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_RNG, ENABLE);
//    RNG_Cmd(ENABLE);
//    printf("enable rng ok\n");
//#if LWIP_DHCP
//    ip_addr_set_zero_ip4(&(WCH_NetIf.ip_addr));
//    ip_addr_set_zero_ip4(&ipaddr);
//    ip_addr_set_zero_ip4(&netmask);
//    ip_addr_set_zero_ip4(&gw);
//#else
//    /* IP addresses initialization */
//    IP4_ADDR(&ipaddr,192,168,123,105);
//    IP4_ADDR(&netmask,255,255,255,0);
//    IP4_ADDR(&gw,192,168,123,1);
//#endif
//    /* Initilialize the LwIP stack without RTOS */
//    lwip_init();
//
//    /* add the network interface (IPv4/IPv6) without RTOS */
//    netif_add(&WCH_NetIf, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
//
//    /* Registers the default network interface */
//    netif_set_default(&WCH_NetIf);
//
//    if (netif_is_link_up(&WCH_NetIf))
//    {
//        /* When the netif is fully configured this function must be called */
//        netif_set_up(&WCH_NetIf);
//#if LWIP_DHCP
//        int err;
//        /*  Creates a new DHCP client for this interface on the first call.
//        Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
//        the predefined regular intervals after starting the client.
//        You can peek in the netif->dhcp struct for the actual DHCP status.*/
//
//        printf("鏈緥绋嬪皢浣跨敤DHCP鍔ㄦ�佸垎閰岻P鍦板潃,濡傛灉涓嶉渶瑕佸垯鍦╨wipopts.h涓皢LWIP_DHCP瀹氫箟涓�0\n\n");
//
//        err = dhcp_start(&WCH_NetIf);      //寮�鍚痙hcp
//        if(err == ERR_OK)
//        {
//            printf("lwip dhcp start success...\n\n");
//        }
//        else
//        {
//            printf("lwip dhcp start fail...\n\n");
//        }
//        sys_timeout(50, wait_dhcp, NULL);
//
//#else
//        lwip_init_success_callback(&(WCH_NetIf.ip_addr)); /*ip鍒嗛厤鎴愬姛鍥炶皟锛岀敤鎴峰湪姝ゅ鍔犲叧浜庣綉缁滆繘绋嬬殑鍒濆鍖栧嚱鏁�*/
//
//#endif
//        OS_TASK_RESTART_ANOTHER(os_lwip_timeouts, 5);  /* 寮�濮嬭秴鏃朵换鍔″鐞� */
//
//        {
//            OS_TASK_SET_STATE();
//            if(list_head(ch307_mac_rec) != NULL)
//            {
//                /* received a packet */
//                ethernetif_input(&WCH_NetIf);
//            }
//            OS_TASK_CWAITX(0);      /* 涓嶆柇杞鏈夋病鏈夋暟鎹寘闇�瑕佸鐞� */
//        }
//    }
//    else
//    {
//        /* When the netif link is down this function must be called */
//        netif_set_down(&WCH_NetIf);
//    }
//    OS_TASK_END(os_lwip);
//}

//OS_TASK(os_lwip_timeouts, void)
//{
//    OS_TASK_START(os_lwip_timeouts);
//
//    {
//        OS_TASK_SET_STATE();
//        sys_check_timeouts();
//        OS_TASK_CWAITX(5);      /* 5ms妫�鏌ヤ竴娆imeouts瓒呮椂 */
//    }
//
//    OS_TASK_END(os_lwip_timeouts);
//}

void ETH_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      ETH_IRQHandler
 *
 * @brief   This function handles ETH exception.
 *
 * @return  none
 */
void ETH_IRQHandler(void)
{
    void *p;
    if(ETH->DMASR&ETH_DMA_IT_R)
    {
        ETH_DMAClearITPendingBit(ETH_DMA_IT_R);

        p = ETH_RxPkt_ChainMode();
        if(p != NULL)
        {
            //!!list_add(ch307_mac_rec, p); /* add to rec list. */
        }
    }
    if(ETH->DMASR&ETH_DMA_IT_T)
    {
        ETH_DMAClearITPendingBit(ETH_DMA_IT_T);
    }

    ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
}

/*********************************************************************
 * @fn      CH30x_RNG_GENERATE
 *
 * @brief   CH30x_RNG_GENERATE api function for lwip.
 *
 * @param   None.
 *
 * @return  None.
 */
uint32_t CH30x_RNG_GENERATE()
{
    while(1)
    {
        if(RNG_GetFlagStatus(RNG_FLAG_DRDY) == SET)
        {
            break;
        }
        if(RNG_GetFlagStatus(RNG_FLAG_CECS) == SET)
        {
            /* 鏃堕挓閿欒 */
            RNG_ClearFlag(RNG_FLAG_CECS);
            Delay_Us(100);
        }
        if(RNG_GetFlagStatus(RNG_FLAG_SECS) == SET)
        {
            /* 绉嶅瓙閿欒 */
            RNG_ClearFlag(RNG_FLAG_SECS);
            RNG_Cmd(DISABLE);
            Delay_Us(100);
            RNG_Cmd(ENABLE);
            Delay_Us(100);
        }
    }
    return RNG_GetRandomNumber();
}

uint32_t sysTicks = 0;

/*********************************************************************
 * @fn      sys_now
 *
 * @brief   sys_now api function for lwip.
 *
 * @param   None.
 *
 * @return  None.
 */
u32_t sys_now(void)
{
    return sysTicks;
}

//void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
///*********************************************************************
// * @fn      SysTick_Handler
// *
// * @brief   SysTick涓柇鍑芥暟
// *
// * @param   None.
// *
// * @return  None.
// */
//void SysTick_Handler(void)
//{
//    sysTicks++;
//    //!!OS_UPDATE_TIMERS();
//    SysTick->SR=0;
//}

volatile uint8_t net_data_led_require = 0;
/*********************************************************************
 * @fn      net_led_tmr
 *
 * @brief   lwip timeouts.c涓璸eriod timer涓鍔犵殑鑷畾涔夎秴鏃跺嚱鏁帮紝姣廚ET_LED_PERIOD_MSECS鍛ㄦ湡璋冪敤
 *
 * @param   None.
 *
 * @return  None.
 */
void net_led_tmr(void)
{
    static uint8_t net_data_led = 0;

    if(net_data_led)
    {
        /* 褰撳墠澶勫湪浜伅鐘舵�� */
        net_data_led = 0;
        Ethernet_LED_DATASET(1);/* turn off data led. */
    }
    else
    {
        /* 褰撳墠澶勫湪鐏伅鐘舵�侊紝濡傛灉鏈懆鏈熷唴鍙堟敹鍒颁簡鏁版嵁鍖咃紝浜伅 */
        if(net_data_led_require != 0)
        {
            net_data_led = 1;
            Ethernet_LED_DATASET(0);/* turn on data led. */
            net_data_led_require = 0;
        }
    }
}

/*********************************************************************
 * @fn      ETH_RxPkt_ChainMode
 *
 * @brief   MAC receive a ethernet frame in chain mode.
 *
 * @return  Frame information.
 */
void* ETH_RxPkt_ChainMode(void)
{
    uint32_t framelength = 0;
  FrameTypeDef *rec_frame;

  //!!rec_frame = memb_alloc(&ch307_mac_rec_frame_mem);

  /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
  if((DMARxDescToGet->Status & ETH_DMARxDesc_OWN) != (u32)RESET)
  {

    if ((ETH->DMASR & ETH_DMASR_RBUS) != (u32)RESET)
    {
      /* Clear RBUS ETHERNET DMA flag */
      ETH->DMASR = ETH_DMASR_RBUS;
      /* Resume DMA reception */
      ETH->DMARPDR = 0;
    }
    printf("Error:ETH_DMARxDesc_OWN.\n");

    if(rec_frame != NULL)
    {
        rec_frame->length = ETH_ERROR;
        //!!memb_free(&ch307_mac_rec_frame_mem, rec_frame);
    }
    /* Return error: OWN bit set */
    return NULL;
  }

  if(rec_frame == NULL)
  {
      return NULL;
  }

  if(
     ((DMARxDescToGet->Status & ETH_DMARxDesc_ES) == (u32)RESET) &&
     ((DMARxDescToGet->Status & ETH_DMARxDesc_LS) != (u32)RESET) &&
     ((DMARxDescToGet->Status & ETH_DMARxDesc_FS) != (u32)RESET))
  {
    /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
    framelength = ((DMARxDescToGet->Status & ETH_DMARxDesc_FL) >> ETH_DMARxDesc_FrameLengthShift) - 4;

    /* Get the addrees of the actual buffer */
    rec_frame->buffer = DMARxDescToGet->Buffer1Addr;
  }
  else
  {
    /* Return ERROR */
    framelength = ETH_ERROR;
    printf("Error:recv error frame,status锛�0x%08x.\n",DMARxDescToGet->Status);
  }
  //DMARxDescToGet->Status |= ETH_DMARxDesc_OWN;//浣滀负鏈搷浣滄暟鎹寘鐨勬爣蹇楋紝鍦╨wip low level input涓嚜琛屽鐞�
  rec_frame->length = framelength;
  rec_frame->descriptor = DMARxDescToGet;

  /* Update the ETHERNET DMA global Rx descriptor with next Rx decriptor */
  /* Chained Mode */
  /* Selects the next DMA Rx descriptor list for next buffer to read */
  DMARxDescToGet = (ETH_DMADESCTypeDef*) (DMARxDescToGet->Buffer2NextDescAddr);

  return rec_frame;

}

/*********************************************************************
 * @fn      ETH_TxPkt_ChainMode
 *
 * @brief   MAC send a ethernet frame in chain mode.
 *
 * @param   Send length
 *
 * @return  Send status.
 */
uint32_t ETH_TxPkt_ChainMode(u16 FrameLength)
{
    /* Check if the descriptor is owned by the ETHERNET DMA (when set) or CPU (when reset) */
    if((DMATxDescToSet->Status & ETH_DMATxDesc_OWN) != (u32)RESET)
    {
        /* Return ERROR: OWN bit set */
        return ETH_ERROR;
    }

    /* Setting the Frame Length: bits[12:0] */
    DMATxDescToSet->ControlBufferSize = (FrameLength & ETH_DMATxDesc_TBS1);
#ifdef CHECKSUM_BY_HARDWARE
    /* Setting the last segment and first segment bits (in this case a frame is transmitted in one descriptor) */
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS | ETH_DMATxDesc_CIC_TCPUDPICMP_Full;
#else
    DMATxDescToSet->Status |= ETH_DMATxDesc_LS | ETH_DMATxDesc_FS;
#endif
    /* Set Own bit of the Tx descriptor Status: gives the buffer back to ETHERNET DMA */
    DMATxDescToSet->Status |= ETH_DMATxDesc_OWN;

    /* When Tx Buffer unavailable flag is set: clear it and resume transmission */
    if ((ETH->DMASR & ETH_DMASR_TBUS) != (u32)RESET)
    {
        /* Clear TBUS ETHERNET DMA flag */
        ETH->DMASR = ETH_DMASR_TBUS;
        /* Resume DMA transmission*/

        ETH->DMATPDR = 0;
    }

    /* Update the ETHERNET DMA global Tx descriptor with next Tx decriptor */
    /* Chained Mode */
    /* Selects the next DMA Tx descriptor list for next buffer to send */
    DMATxDescToSet = (ETH_DMADESCTypeDef*) (DMATxDescToSet->Buffer2NextDescAddr);


    /* Return SUCCESS */
    return ETH_SUCCESS;
}

/*********************************************************************
 * @fn      mac_send
 *
 * @brief   MAC send a ethernet frame in chain mode.
 *
 * @param   Send length and send pointer.
 *
 * @return  none
 */
void mac_send(uint8_t * content_ptr, uint16_t content_len)
{
    u8 *buffer =  (u8 *)ETH_GetCurrentTxBufferAddress();

    memcpy(buffer, content_ptr, content_len);
    if( ! ETH_TxPkt_ChainMode(content_len))
    {
        printf("Send failed.\n");
    }
}

/*********************************************************************
 * @fn      PHY_control_pin_init
 *
 * @brief   PHY interrupt GPIO Initialization.
 *
 * @return  none
 */
void PHY_control_pin_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource15);
    EXTI_InitStructure.EXTI_Line=EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*********************************************************************
 * @fn      GETH_pin_init
 *
 * @brief   PHY RGMII interface GPIO initialization.
 *
 * @return  none
 */
void GETH_pin_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PB12/13缃负鎺ㄦ尳澶嶇敤杈撳嚭 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
    GPIOB->CFGHR&=~(0xff<<16);
    GPIOB->CFGHR|= (0xbb<<16);
    GPIOB->CFGLR&=~(0xff<<4);

    define_O(GPIOA,GPIO_Pin_2);
    define_O(GPIOA,GPIO_Pin_3);
    define_O(GPIOA,GPIO_Pin_7);
    define_O(GPIOC,GPIO_Pin_4);
    define_O(GPIOC,GPIO_Pin_5);
    define_O(GPIOB,GPIO_Pin_0);

    define_I(GPIOC,GPIO_Pin_0);
    define_I(GPIOC,GPIO_Pin_1);
    define_I(GPIOC,GPIO_Pin_2);
    define_I(GPIOC,GPIO_Pin_3);
    define_I(GPIOA,GPIO_Pin_0);
    define_I(GPIOA,GPIO_Pin_1);

    define_I(GPIOB,GPIO_Pin_1);/* 125m in */
}

/*********************************************************************
 * @fn      FETH_pin_init
 *
 * @brief   PHY MII/RMII interface GPIO initialization.
 *
 * @return  none
 */
void FETH_pin_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USE_RMII
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
    GPIO_ETH_MediaInterfaceConfig(GPIO_ETH_MediaInterface_RMII);
    define_O(GPIOA,GPIO_Pin_2);/* MDC */
    define_O(GPIOC,GPIO_Pin_1);/* MDIO */

    define_O(GPIOB,GPIO_Pin_11);//txen
    define_O(GPIOB,GPIO_Pin_12);//txd0
    define_O(GPIOB,GPIO_Pin_13);//txd1

    define_I(GPIOA,GPIO_Pin_1);/* PA1 REFCLK */
    define_I(GPIOA,GPIO_Pin_7);/* PA7 CRSDV */
    define_I(GPIOC,GPIO_Pin_4);/* RXD0 */
    define_I(GPIOC,GPIO_Pin_5);/* RXD1 */

#else
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);

    define_O(GPIOA,GPIO_Pin_2);/* MDC */
    define_O(GPIOC,GPIO_Pin_1);/* MDIO */

    define_I(GPIOC,GPIO_Pin_3);//txclk
    define_O(GPIOB,GPIO_Pin_11);//txen
    define_O(GPIOB,GPIO_Pin_12);//txd0
    define_O(GPIOB,GPIO_Pin_13);//txd1
    define_O(GPIOC,GPIO_Pin_2); //txd2
    define_O(GPIOB,GPIO_Pin_8);//txd3
/* RX缁� */
    define_I(GPIOA,GPIO_Pin_1);/* PA1 RXC */
    define_I(GPIOA,GPIO_Pin_7);/* PA7 RXDV */
    define_I(GPIOC,GPIO_Pin_4);/* RXD0 */
    define_I(GPIOC,GPIO_Pin_5);/* RXD1 */
    define_I(GPIOB,GPIO_Pin_0);/* RXD2 */
    define_I(GPIOB,GPIO_Pin_1);/* RXD3 */
    define_I(GPIOB,GPIO_Pin_10);/* RXER */

    define_O(GPIOA,GPIO_Pin_0);/* PA0 */
    define_O(GPIOA,GPIO_Pin_3);/* PA3 */
#endif
}


