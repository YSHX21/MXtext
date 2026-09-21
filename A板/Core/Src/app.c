
#include "app.h"
#include "hx711.h"   /*HX711称重驱动(PA5=DOUT数据, PA4=SCK时钟, CubeMX已初始化)*/

/*================= HX711 称重(1kg量程) 配置 =================
 * 采用与你验证通过的参考代码一致的两点标定：
 *  上电空载时自动读一次原始码作为零点 reset；
 *  放上 HX711_STD_G 克标准砝码时的原始读数为 HX711_STD_RAW。
 * 重量(克) = Δ原始码 × HX711_STD_G ÷ (HX711_STD_RAW - reset)
 * 例：100g砝码读数8768602、空载约8.5M → HX711_STD_RAW=8768602，
 *     HX711_STD_G=100.0f；请按你的实测修改两个宏 */
#define HX711_SAMPLE_N     5U         /*每轮采样次数(取平均滤波)*/
#define HX711_STD_G        100.0f     /*标准砝码质量(克)*/
#define HX711_STD_RAW      8768602UL  /*放该砝码时的原始读数(实测标定)*/

static unsigned long g_reset_raw = 0; /*空载零点(上电自动读，对应参考里的reset)*/
static uint8_t       g_reset_ok  = 0; /*空载零点已获取标志*/
static volatile int32_t g_weight_gram = 0; /*当前重量(克)，供OLED显示*/
static volatile uint8_t g_hx_ready = 0U;   /*HX711是否已成功读到数据(0时屏幕显示----，便于判断通信)*/

/*button队列发送*/
static void App_ButtonSend(void)
{
    uint8_t key =0;
    key =Button4_4_Scan();
    if(key!=0)
    {
        (void)xQueueSend(xQueue1, &key,portMAX_DELAY);
    }

}
/*button队列接收：返回1表示收到有效按键，0表示队列为空
  (❗必须判断返回值，否则会把"没收到"当按键0写入目标数组，填满后新按键失效)*/
static uint8_t App_ButtonRecv(uint8_t *pkey)
{
    uint8_t key = 0U;
    if(xQueueReceive(xQueue1, &key, 0) != pdTRUE)
        return 0U;
    if(key == 0U)
        return 0U;
    *pkey = key;
    return 1U;
}


/**
 * @brief 接收一条数据，自动去重：出现重复则清除两个相同数据
 * @param new_val 新接收到的数据
 */
static void ReceiveAndDeduplicate(uint8_t new_val)
{
    uint8_t i;
    taskENTER_CRITICAL();
    for(i = 0; i < g_data_cnt; i++)
    {
        if(g_data_buf[i] == new_val)
        {
            //发现重复，删除旧数据，丢弃新数据
            for(uint8_t j = i; j < g_data_cnt - 1; j++)
            {
                g_data_buf[j] = g_data_buf[j+1];
            }
            g_data_cnt --;
            oled_refresh_flag = 1; //❗数据修改完毕，通知OLED立刻刷新
            taskEXIT_CRITICAL();
            return;
        }
    }
    //没有重复，存入数组
    if(g_data_cnt < ARR_MAX_LEN)
    {
        g_data_buf[g_data_cnt] = new_val;
        g_data_cnt ++;
        oled_refresh_flag = 1; //❗存入新数据，标记刷新
    }
    taskEXIT_CRITICAL();
}
/*按照数组的内容，显示在OLED上*/

//局部清除8个点位，输出空格覆盖旧数字
static void ClearAllShowPos(void)
{
    for(uint8_t i=0;i<12;i++)
    {
        uint8_t r = pos_table[i].row;
        uint8_t c = pos_table[i].col;
        OLED_ShowString(r, c, "  ");
    }
}

static void ShowDataOnOLED(void)
{
    //拷贝数组，临界区保护
    taskENTER_CRITICAL();
    uint8_t cnt = g_data_cnt;
    uint16_t temp_buf[ARR_MAX_LEN];
    for(uint8_t k=0;k<cnt;k++)
    {
        temp_buf[k] = g_data_buf[k];
    }
    taskEXIT_CRITICAL();

    ClearAllShowPos(); //清除12个显示位置

    //绘制当前有效数据
    for(uint8_t i = 0; i < cnt; i++)
    {
        uint16_t val = temp_buf[i];
        if(val >= 1 && val <= 12)
        {
            uint8_t idx = val - 1;
            uint8_t r = pos_table[idx].row;
            uint8_t c = pos_table[idx].col;
            OLED_ShowNum(r, c, val, 2);
        }
    }
    /*第0行：门状态 + 电梯当前楼层
      (数据由门控任务更新，这里统一绘制，避免两个任务并发操作OLED)*/
    switch(g_door_state)
    {
    case DOOR_OPENED:  OLED_ShowString(4, 1, "Open "); break;
    case DOOR_CLOSING: OLED_ShowString(4, 1, "Clos "); break;
    case DOOR_CLOSED:  OLED_ShowString(4, 1, "Close"); break;
    case DOOR_RUNNING: OLED_ShowString(4, 1, "Run  "); break;
    default:           OLED_ShowString(4, 1, "Err  "); break;
    }
    OLED_ShowString(4, 6, "F:");
    OLED_ShowNum(4, 8, g_floor_cur, 2);

    /*第4行右侧：实时重量(克)。电梯运行时不更新，由App_WeightTask维护*/
    OLED_ShowString(4, 10, "W:");
    if(g_hx_ready == 0U)          /*HX711从未读到数据：显示----，提示通信/接线问题*/
    {
        OLED_ShowString(4, 12, "----");
        OLED_ShowChar(4, 16, 'g');
    }
    else
    {
        int32_t  w    = (g_weight_gram < 0) ? 0 : g_weight_gram;
        uint32_t div  = 1000U;
        uint8_t  st   = 0U;
        for(uint8_t k = 0; k < 4; k++)       /*高位补空格，最多显示4位克数*/
        {
            uint8_t d = (uint8_t)((w / div) % 10);
            div /= 10;
            if((st == 1U) || (d != 0U) || (k == 3U))
            {
                OLED_ShowChar(4, 12 + k, d + '0');
                st = 1U;
            }
            else
            {
                OLED_ShowChar(4, 12 + k, ' ');   /*覆盖掉旧的高位数字*/
            }
        }
        OLED_ShowChar(4, 16, 'g');
    }
}
/*DAM转运数据处理*/
static uint16_t filter_buf[150];
static void App_DMA_value(void)
{
    uint16_t i = 0;  //❗改成uint16_t，防止i溢出
    memcpy(filter_buf, ADC_Value, sizeof(ADC_Value));
    ad1 = 0;
    ad2 = 0;
    ad3 = 0;
    for(i = 0; i < 150; ){
        ad1 += filter_buf[i++];
        ad2 += filter_buf[i++];
        ad3 += filter_buf[i++];
    }
    ad1 /= 50;
    ad2 /= 50;
    ad3 /= 50;
}
/*电梯开门 关门控制*/
void xuanzhun_SetAngle(float Angle)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,Angle/180*2000+500);
}
/*任务1楼层输入和显示*/
static void App_InputTask( void * pvParameters )
{
    (void)pvParameters;

    
    //xTimerStart(xTimer1,portMAX_DELAY);//打开软件定时器
    while(1)
    {        
        uint8_t new_key = 0U;
        App_ButtonSend();
        if(App_ButtonRecv(&new_key) != 0U)   /*只有真的收到按键才处理*/
        {
            ReceiveAndDeduplicate(new_key);
        }
        //检测刷新标志
        if(oled_refresh_flag == 1)
        {
            ShowDataOnOLED();
            taskENTER_CRITICAL();
            oled_refresh_flag = 0U; //清除刷新标记
            taskEXIT_CRITICAL();
        }
        vTaskDelay(pdMS_TO_TICKS(20)); //20ms轮询，响应很快       
    }
}
/*任务2传感器数据采集*/
static void App_SensorTask( void * pvParameters )
{
    (void)pvParameters;
    /*启动DMA传感器数据采集*/
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Value,150);
    while(1)
    {
       jb=xSemaphoreTake(xSemaphore1,portMAX_DELAY);
       if(jb==pdTRUE)
       {
        App_DMA_value();
       }
       vTaskDelay(pdMS_TO_TICKS(20));   //✅20ms刷新一次ad，关门防夹检测更及时
    }

}

/*==================================================
 * 电梯自动开关门控制(任务3)
 * 流程：开门等待 →(无阻挡持续5s)→ 关门(防夹窗口)
 *      →(有选层目标)→ 模拟升降 → 到站开门 → 循环
 *==================================================*/
/*电梯运行方向：1=上行，-1=下行(用于"顺路就近"调度)*/
static int8_t g_dir = 1;

/*挑选下一个服务楼层：不是先来先到，而是按当前运行方向"顺路就近"。
 *  dir > 0(上行)：优先选还在上方的、离当前层最近的；上方没请求才选下方最高的(掉头后依次服务)
 *  dir < 0(下行)：优先选还在下方的、离当前层最近的；下方没请求才选上方最低的
 *  dir == 0(静止)：选离当前层最近的，同距离优先上行
 *  本层(==cur)的请求优先级最高，立即开门服务
 * 无效请求(0或>12)会自动从数组剔除，避免占满槽位把电梯卡住*/
static uint8_t App_PickTarget(uint8_t cur, int8_t dir)
{
    uint8_t best = 0;

    taskENTER_CRITICAL();
    for(uint8_t i = 0; i < g_data_cnt; i++)
    {
        uint8_t f = (uint8_t)g_data_buf[i];

        if((f < 1U) || (f > 12U))               /*无效请求：直接剔除*/
        {
            for(uint8_t j = i; j < (g_data_cnt - 1U); j++)
            {
                g_data_buf[j] = g_data_buf[j+1];
            }
            g_data_cnt --;
            oled_refresh_flag = 1;
            i --;
            continue;
        }

        if(f == cur)                             /*本层请求最优先：就地开门*/
        {
            best = f;
            break;
        }

        if(best == 0U)
        {
            best = f;
            continue;
        }

        if(dir > 0)
        {
            uint8_t f_up    = (f > cur)    ? 1U : 0U;
            uint8_t best_up = (best > cur) ? 1U : 0U;
            if(f_up && (!best_up))                          best = f;   /*上方请求优先于下方*/
            else if(f_up && best_up && (f < best))          best = f;   /*都在上方：取最近*/
            else if((!f_up) && (!best_up) && (f > best))    best = f;   /*都在下方：取最高*/
        }
        else if(dir < 0)
        {
            uint8_t f_dn    = (f < cur)    ? 1U : 0U;
            uint8_t best_dn = (best < cur) ? 1U : 0U;
            if(f_dn && (!best_dn))                          best = f;   /*下方请求优先于上方*/
            else if(f_dn && best_dn && (f > best))          best = f;   /*都在下方：取最近*/
            else if((!f_dn) && (!best_dn) && (f < best))    best = f;   /*都在上方：取最低*/
        }
        else                                                /*静止：取距离最近，同距优先上行*/
        {
            uint8_t d_new  = (f > cur)    ? (uint8_t)(f - cur)    : (uint8_t)(cur - f);
            uint8_t d_best = (best > cur) ? (uint8_t)(best - cur) : (uint8_t)(cur - best);
            if(d_new < d_best)                       best = f;
            else if((d_new == d_best) && (f > cur))  best = f;
        }
    }
    taskEXIT_CRITICAL();

    return best;
}
/*==================================================================
 * CAN 2.0A 双节点通信(主控A ↔ 驱动B)
 *------------------------------------------------------------------
 * 节点A(主控)：按键 + 调度 + 门控 + 称重 + 显示
 * 节点B(驱动)：只负责升降(电机未到之前用延时模拟)
 *
 * ID 位段(11bit)： [10:8]消息类型   [7:4]源节点   [3:0]目标节点
 * CAN 仲裁规则：ID 数值越小优先级越高 → 按实时性倒排，急停排最小
 *==================================================================*/
#define CAN_ID_TYPE_SHIFT   8U
#define CAN_ID_SRC_SHIFT    4U
#define CAN_ID_TYPE_MASK    0x7U
#define CAN_ID_NODE_MASK    0xFU

typedef enum
{
    MSG_EMERGENCY = 0U,   /*急停/故障：优先级最高(ID最小，能抢占总线)*/
    MSG_MOTION_STA,       /*B→A  运行状态帧*/
    MSG_WEIGHT,           /*B→A  称重*/
    MSG_DOOR_OK,          /*A→B  门已关好，允许运行*/
    MSG_CMD_GOTO          /*A→B  目标楼层*/
}CanMsgType_e;

typedef enum
{
    NODE_MAIN   = 1U,     /*主控板A*/
    NODE_DRIVER = 2U      /*驱动板B*/
}CanNode_e;

/*运行状态*/
#define MOTOR_STA_IDLE     0U
#define MOTOR_STA_RUN      1U
#define MOTOR_STA_ARRIVED  2U
#define MOTOR_STA_FAULT    3U

/*运行方向(用 1/2 表示，不用 -1，避免uint8符号问题)*/
#define MOTOR_DIR_STOP     0U
#define MOTOR_DIR_UP       1U
#define MOTOR_DIR_DOWN     2U

/*拼ID / 拆ID*/
#define CAN_MAKE_ID(type,src,dst) ((uint32_t)(((uint32_t)(type) << CAN_ID_TYPE_SHIFT) | \
                                              ((uint32_t)(src)  << CAN_ID_SRC_SHIFT)  | \
                                              ((uint32_t)(dst))))
#define CAN_GET_TYPE(id)  ((uint8_t)(((id) >> CAN_ID_TYPE_SHIFT) & CAN_ID_TYPE_MASK))
#define CAN_GET_SRC(id)   ((uint8_t)(((id) >> CAN_ID_SRC_SHIFT)  & CAN_ID_NODE_MASK))
#define CAN_GET_DST(id)   ((uint8_t)( (id)                       & CAN_ID_NODE_MASK))

/*数据域约定(每帧只用前几个字节)
 * MSG_CMD_GOTO  : [0]=目标楼层
 * MSG_DOOR_OK   : [0]=1允许运行
 * MSG_MOTION_STA: [0]=当前楼层 [1]=运行状态 [2]=方向
 * MSG_EMERGENCY : [0]=故障码
 * MSG_WEIGHT    : [0]=重量低字节 [1]=重量高字节 [2]=超载标志 */

/*一帧CAN报文(注意：结构体可以直接通过队列整体传递，方便又安全)*/
typedef struct
{
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
}CanFrame_t;

static QueueHandle_t g_can_rx_queue    = NULL;  /*CAN中断   → App_CanTask(解析)*/
static QueueHandle_t g_motor_cmd_queue = NULL;  /*App_CanTask → App_TractionTask(曳引执行)*/

/**
 * @brief 发送一帧CAN报文(非阻塞：总线异常时最多等100ms就放弃，绝不卡死任务)
 * @param type 消息类型  @param src 源节点  @param dst 目标节点
 * @retval pdTRUE=发送成功  pdFALSE=失败(调用方可以据此重发)
 */
static BaseType_t Can_Send(uint8_t type, uint8_t src, uint8_t dst, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t mailbox;

    if(len > 8U) len = 8U;

    txHeader.StdId  = CAN_MAKE_ID(type, src, dst);
    txHeader.IDE    = CAN_ID_STD;
    txHeader.RTR    = CAN_RTR_DATA;
    txHeader.DLC    = len;
    txHeader.TransmitGlobalTime = DISABLE;

    /*放入发送邮箱。失败(邮箱满/总线关闭)返回 pdFALSE 告诉调用方，绝不在这里死等*/
    if(HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &mailbox) != HAL_OK)
        return pdFALSE;

    for(uint16_t i = 0U; i < 100U; i++)
    {
        if(HAL_CAN_IsTxMessagePending(&hcan, mailbox) == 0U) return pdTRUE;
        vTaskDelay(pdMS_TO_TICKS(1));   /*让出CPU，别空转*/
    }
    return pdFALSE;   /*100ms 还没发出去 = 超时*/
}

/*主控下发"去N楼"*/
static void Can_SendCmdGoto(uint8_t floor)
{
    uint8_t tx[1] = { floor };
    Can_Send(MSG_CMD_GOTO, NODE_MAIN, NODE_DRIVER, tx, 1);
}

/**
 * @brief CAN接收中断回调：只负责"读报文 + 入队"，解析交给任务
 *        (中断里干活越少越好，这是RTOS的基本原则)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rxHeader;
    CanFrame_t frame;
    BaseType_t hpw = pdFALSE;

    if(hcan_ptr->Instance != CAN1) return;
    if(g_can_rx_queue == NULL)     return;      /*队列还没建好，直接丢弃*/

    if(HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO0, &rxHeader, frame.data) != HAL_OK)
        return;

    frame.id  = rxHeader.StdId;
    frame.len = rxHeader.DLC;

    xQueueSendFromISR(g_can_rx_queue, &frame, &hpw);
    portYIELD_FROM_ISR(hpw);                    /*有高优先级任务被唤醒就立刻切换*/
}

/*任务5：CAN 收发与协议分派
 * 链路：CAN中断 → g_can_rx_queue → 本任务 → 按消息类型分派
 *
 * ❗这个工程是【主控A】。CMD_GOTO 只在单板模拟模式下才由本机消费
 *   (真双节点时它应该被驱动板B消费，A 自己不处理)*/
static void App_CanTask( void * pvParameters )
{
    (void)pvParameters;
    CanFrame_t frame;

    while(1)
    {
        if(xQueueReceive(g_can_rx_queue, &frame, portMAX_DELAY) != pdTRUE)
            continue;

        switch(CAN_GET_TYPE(frame.id))
        {
        case MSG_CMD_GOTO:            /*A→曳引侧：去N楼(单板模拟模式下本机代收)*/
#if TRACTION_LOCAL_SIM
            (void)xQueueSend(g_motor_cmd_queue, &frame.data[0], 0);
#endif
            break;

        case MSG_MOTION_STA:          /*曳引侧→A：运行状态(A 的核心输入)*/
            g_motor_cur_floor = frame.data[0];
            g_motor_state     = frame.data[1];
            g_floor_cur       = frame.data[0];  /*楼层权威在曳引侧(它有编码器)，这里直接镜像*/
            g_run_wait_tick   = 0U;             /*收到状态帧 = 曳引侧还活着 → 喂狗*/
            g_run_retry       = 0U;
            oled_refresh_flag = 1;
            break;

        case MSG_EMERGENCY:           /*急停：ID最小，能抢占总线*/
            g_motor_state = MOTOR_STA_FAULT;
            oled_refresh_flag = 1;
            break;

        default:
            break;
        }
    }
}

/*任务6：曳引执行(电机未到 → 本机延时模拟)
 * 收到"去N楼" → 逐层走，每走一层上报一次运行状态 → 到站上报 ARRIVED
 *
 * ❗关键改动(修"冲过头的bug")：每 50ms 都去看一眼队列里有没有新命令，
 *   所以运行中途改终点能立刻生效，不用等跑完旧目标。
 *
 * ❗电机到位后：这个任务整体搬到驱动板B的独立工程里，把"走一层"换成
 *   编码器闭环。收命令/上报状态/协议/A侧代码 —— 全都不用动*/
static void App_TractionTask( void * pvParameters )
{
    (void)pvParameters;

    uint8_t  cur    = 1U;   /*当前楼层(电机到位后改成编码器脉冲换算)*/
    uint8_t  target = 0U;
    uint8_t  have   = 0U;   /*是否有有效目标*/
    uint16_t step_wait = 0U;/*距离走下一层还剩几拍*/

    while(1)
    {
        /*① 每拍都看一眼有没有新命令(超时0，不阻塞) → 支持运行中途改终点*/
        if(xQueueReceive(g_motor_cmd_queue, &target, 0) == pdTRUE)
        {
            have      = 1U;
            step_wait = 0U;         /*新命令：立刻开始走*/
        }

        /*② 没有目标 → 歇着*/
        if(have == 0U)
        {
            vTaskDelay(pdMS_TO_TICKS(TRACTION_PERIOD));
            continue;
        }

        /*③ 目标无效 / 已在本层 → 直接回到站*/
        if((target < 1U) || (target > 12U) || (target == cur))
        {
            have = 0U;
            uint8_t tx[3] = { cur, MOTOR_STA_ARRIVED, MOTOR_DIR_STOP };
            Can_Send(MSG_MOTION_STA, NODE_DRIVER, NODE_MAIN, tx, 3);
            continue;
        }

        /*④ 还没到走下一层的时刻*/
        if(step_wait > 0U)
        {
            step_wait --;
            vTaskDelay(pdMS_TO_TICKS(TRACTION_PERIOD));
            continue;
        }

        /*⑤ 走一层并上报状态*/
        uint8_t dir = (target > cur) ? MOTOR_DIR_UP : MOTOR_DIR_DOWN;
        cur = (dir == MOTOR_DIR_UP) ? (uint8_t)(cur + 1U) : (uint8_t)(cur - 1U);

        uint8_t tx[3];
        tx[0] = cur;
        tx[1] = (cur == target) ? MOTOR_STA_ARRIVED : MOTOR_STA_RUN;
        tx[2] = dir;
        Can_Send(MSG_MOTION_STA, NODE_DRIVER, NODE_MAIN, tx, 3);

        if(cur == target) have = 0U;                                  /*到站*/
        else              step_wait = (uint16_t)(FLOOR_RUN_MS / TRACTION_PERIOD);

        vTaskDelay(pdMS_TO_TICKS(TRACTION_PERIOD));
    }
}

/*   CAN初始化补充    */ 
void CAN_Init()
{
	CAN_FilterTypeDef filter;

#if CAN_LOOPBACK_TEST
	/*单机自测：切回环模式(自发自收)。
	  正常模式下如果总线上没有其他节点给ACK应答，报文会一直重发失败，
	  所以只有一块板时必须开这个。两块板接好后把宏改成 0。*/
	HAL_CAN_Stop(&hcan);
	hcan.Init.Mode = CAN_MODE_LOOPBACK;
	if(HAL_CAN_Init(&hcan) != HAL_OK) Error_Handler();
#endif

	filter.FilterActivation = ENABLE;
	filter.FilterMode = CAN_FILTERMODE_IDMASK;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterIdHigh = 0x0000;
	filter.FilterIdLow = 0x0000;
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.FilterBank = 0;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan, &filter);
	HAL_CAN_Start(&hcan);

	/*❗CubeMX 没有开 CAN 的 NVIC，必须手动开，否则中断进不来。
	  优先级 5 不能改小：FreeRTOSConfig.h 里
	  configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5，
	  比它更高(数值更小)的中断里不能调用 xQueueSendFromISR。*/
	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  CAN发送函数（兼容原标准库接口）
 * @param  ID: 报文ID
 * @param  Length: DLC数据长度0~8
 * @param  Data: 发送数据缓冲区指针
 */
void MyCAN_Transmit(uint32_t ID, uint8_t Length, uint8_t *Data)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;   // HAL库返回的邮箱编号

    // 配置报文头：标准帧、数据帧
    txHeader.StdId = ID;
    txHeader.ExtId = ID;
    txHeader.IDE = CAN_ID_STD;         // 标准ID帧，对应原 CAN_Id_Standard
    txHeader.RTR = CAN_RTR_DATA;       // 数据帧，对应原 CAN_RTR_Data
    txHeader.DLC = Length;
    txHeader.TransmitGlobalTime = DISABLE;

    // 将数据放入发送邮箱
    if(HAL_CAN_AddTxMessage(&hcan, &txHeader, Data, &txMailbox) != HAL_OK)
    {
        return; // 添加发送失败，直接退出
    }

    uint32_t Timeout = 0;
    // 等待发送完成，判断邮箱状态
    while(HAL_CAN_IsTxMessagePending(&hcan, txMailbox) != 0)
    {
        Timeout++;
        if(Timeout > 100000)
        {
            break;  // 发送超时退出
        }
    }
}

/**
 * @brief  查询FIFO0是否有收到报文
 * @retval 1：有报文  0：无报文
 */
uint8_t MyCAN_ReceiveFlag(void)
{
    // 查询FIFO0挂起报文数量
    if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 从FIFO0读取一帧CAN报文（查询方式）
 * @param ID: 输出，报文ID
 * @param Length:输出，DLC长度
 * @param Data:输出，接收数据缓存
 */
void MyCAN_Receive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{
    CAN_RxHeaderTypeDef rxHeader;

    // HAL库读取FIFO0报文
    if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, Data) != HAL_OK)
    {
        return;
    }

    if(rxHeader.IDE == CAN_ID_STD)
    {
        *ID = rxHeader.StdId;
    }
    else
    {
        *ID = rxHeader.ExtId;
    }

    if(rxHeader.RTR == CAN_RTR_DATA)
    {
        *Length = rxHeader.DLC;
        // Data已经由HAL_CAN_GetRxMessage直接填充完成，不需要循环拷贝
    }
    else
    {
        // 远程帧RTR处理，这里留空
    }
}
/*服务完成：移除数组中第一个等于 floor 的目标，并通知OLED刷新*/
static void RemoveFloor(uint8_t floor)
{
    taskENTER_CRITICAL();
    for(uint8_t i = 0; i < g_data_cnt; i++)
    {
        if(g_data_buf[i] == floor)
        {
            for(uint8_t j = i; j < g_data_cnt - 1; j++)
            {
                g_data_buf[j] = g_data_buf[j+1];
            }
            g_data_cnt --;
            oled_refresh_flag = 1;
            break;
        }
    }
    taskEXIT_CRITICAL();
}

static void App_DoorTask( void * pvParameters )
{
    (void)pvParameters;
    /*起始状态：开门*/
    xuanzhun_SetAngle(DOOR_OPEN_ANGLE);
    g_door_state = DOOR_OPENED;
    oled_refresh_flag = 1;

    while(1)
    {
        switch(g_door_state)
        {
        case DOOR_OPENED:
            /*开门等待：检测到人/阻挡(ad2<DOOR_BLOCK_TH)就一直保持开门并清零计时*/
            if(ad2 < DOOR_BLOCK_TH)
            {
                door_tick = 0;
            }
            else
            {
                door_tick ++;
                if(door_tick >= (DOOR_OPEN_MS / DOOR_TASK_PERIOD)) /*无人满5s→关门(渐进式，见DOOR_CLOSING)*/
                {
                    g_door_state = DOOR_CLOSING;
                    door_tick = 0;
                    oled_refresh_flag = 1;
                }
            }
            break;

        case DOOR_CLOSING:
        {
            /*关门：从开门角按 DOOR_CLOSE_STEP 逐步推进到关门角。
              每推进一步停下 DOOR_STEP_MS 并持续防夹检测；
              关门全程任何时刻检测到阻挡(ad2<DOOR_BLOCK_TH)→立即弹回开门角并重新计时，
              从而保证关门过程随时可被打断(防夹生效)*/
            float   cur     = DOOR_OPEN_ANGLE; /*当前门角度，从全开开始*/
            uint8_t blocked = 0;

            while((cur < DOOR_CLOSE_ANGLE) && (blocked == 0))
            {
                /*① 向前推进一小步并输出到舵机*/
                cur += DOOR_CLOSE_STEP;
                if(cur > DOOR_CLOSE_ANGLE)
                    cur = DOOR_CLOSE_ANGLE;
                xuanzhun_SetAngle(cur);

                /*② 本步等待期内反复检测阻挡，随时打断关门*/
                for(uint16_t s = 0; s < (DOOR_STEP_MS / DOOR_TASK_PERIOD); s++)
                {
                    vTaskDelay(pdMS_TO_TICKS(DOOR_TASK_PERIOD));
                    if(ad2 < DOOR_BLOCK_TH)
                    {
                        blocked = 1;
                        break;              /*防夹触发，立即退出等待循环*/
                    }
                }
            }

            if(blocked)                     /*防夹：立即弹开并重新开门计时*/
            {
                xuanzhun_SetAngle(DOOR_OPEN_ANGLE);
                g_door_state = DOOR_OPENED;
                door_tick = 0;
                oled_refresh_flag = 1;
            }
            else                            /*全程无阻挡，关门到位*/
            {
                g_door_state = DOOR_CLOSED;
                door_tick = 0;
                oled_refresh_flag = 1;
            }
            break;
        }

        case DOOR_CLOSED:
        {
            /*门已关好：按"顺路就近"挑目标(不再先来先到)*/
            uint8_t target = App_PickTarget(g_floor_cur, g_dir);
            if(target == 0U) break;           /*暂无选层请求，停靠等待*/

            if(target == g_floor_cur)         /*请求的正好是本层→原地开门服务*/
            {
                RemoveFloor(target);
                xuanzhun_SetAngle(DOOR_OPEN_ANGLE);
                g_door_state = DOOR_OPENED;
                door_tick = 0;
                oled_refresh_flag = 1;
                break;
            }

            run_target = target;                        /*锁定本次运行目标*/
            g_dir = (target > g_floor_cur) ? 1 : -1;    /*确定运行方向*/
            oled_refresh_flag = 1;

            /*❗关好门后不能马上运行：先停留 DOOR_START_DELAY_MS(模拟松闸/发车准备)*/
            for(uint16_t i = 0; i < (DOOR_START_DELAY_MS / DOOR_TASK_PERIOD); i++)
            {
                vTaskDelay(pdMS_TO_TICKS(DOOR_TASK_PERIOD));
            }

            /*发车前再挑一次：把等待期间新按下的楼层也纳入调度*/
            target = App_PickTarget(g_floor_cur, g_dir);
            if(target == 0U) break;                     /*请求已取消，继续停靠等待*/

            if(target == g_floor_cur)                   /*等待期间按了本层→直接开门*/
            {
                RemoveFloor(target);
                xuanzhun_SetAngle(DOOR_OPEN_ANGLE);
                g_door_state = DOOR_OPENED;
                door_tick = 0;
                oled_refresh_flag = 1;
                break;
            }

            run_target = target;
            g_dir = (target > g_floor_cur) ? 1 : -1;
            g_door_state = DOOR_RUNNING;
#if CAN_ENABLE
            /*门已关好 → 把目标层交给曳引侧(门关好才允许运行 = 安全联锁)*/
            uint8_t door_ok = 1U;
            Can_Send(MSG_DOOR_OK, NODE_MAIN, NODE_DRIVER, &door_ok, 1);
            Can_SendCmdGoto(run_target);
            g_can_sent_target = run_target;
            g_motor_state     = MOTOR_STA_RUN;
            g_run_wait_tick   = 0U;
            g_run_retry       = 0U;
#endif
            oled_refresh_flag = 1;
            break;
        }
        case DOOR_RUNNING:
        {
            /*运行中动态改终点：同方向上出现"更近且还没经过"的新请求，就把终点改成它。
              例：上行目标10楼，走到3楼时按下5楼 → 5楼还没经过，终点改为5楼先停；
                  若按的是12楼(更远)或2楼(已过)，则保持原终点，到站后再重新调度*/
            uint8_t t = App_PickTarget(g_floor_cur, g_dir);
            if(t != 0U)
            {
                if((g_dir > 0) && (t > g_floor_cur) && (t < run_target))
                {
                    run_target = t;
                }
                else if((g_dir < 0) && (t < g_floor_cur) && (t > run_target))
                {
                    run_target = t;
                }
            }

#if CAN_ENABLE
            /*===== 升降交给曳引侧：本任务只"等到站"，不再自己算时间 =====*/

            /*① 终点被改了 → 重新下发目标(曳引侧支持中途改终点)*/
            if(run_target != g_can_sent_target)
            {
                Can_SendCmdGoto(run_target);
                g_can_sent_target = run_target;
                g_run_wait_tick   = 0U;
            }

            /*② 看门狗：太久没收到任何状态帧 → 重发命令(防止丢帧导致永久等待)
                正常运行每层(1s)会来一帧，所以 2s 不触发；真丢帧才会走到这里*/
            if(g_motor_state == MOTOR_STA_RUN)
            {
                g_run_wait_tick ++;
                if(g_run_wait_tick >= RUN_WARN_TICKS)
                {
                    g_run_wait_tick = 0U;
                    Can_SendCmdGoto(run_target);
                    g_run_retry ++;
                    if(g_run_retry >= 3U)               /*连续3次没响应 → 报故障*/
                    {
                        g_motor_state = MOTOR_STA_FAULT;
                        g_run_retry   = 0U;
                    }
                }
            }
            else
            {
                g_run_wait_tick = 0U;
                g_run_retry     = 0U;
            }

            /*③ 楼层显示：已经在 App_CanTask 里跟随曳引侧镜像了，这里不再重复处理*/

            if(g_motor_state == MOTOR_STA_ARRIVED)      /*到站 → 开门*/
            {
                RemoveFloor(g_motor_cur_floor);
                xuanzhun_SetAngle(DOOR_OPEN_ANGLE);
                g_door_state = DOOR_OPENED;
                door_tick = 0;
                oled_refresh_flag = 1;
            }
            else if(g_motor_state == MOTOR_STA_FAULT)   /*故障 → 回停靠态等处理*/
            {
                g_door_state = DOOR_CLOSED;
                oled_refresh_flag = 1;
            }
#else
            /*===== 纯本地延时模拟(WORK_MODE = MODE_PURE_LOCAL 时才走这里) =====*/
            if(g_floor_cur < run_target)
            {
                g_floor_cur ++;
            }
            else if(g_floor_cur > run_target)
            {
                g_floor_cur --;
            }
            oled_refresh_flag = 1;
            for(uint16_t i = 0; i < (FLOOR_RUN_MS / DOOR_TASK_PERIOD); i++)
            {
                vTaskDelay(pdMS_TO_TICKS(DOOR_TASK_PERIOD)); /*模拟运行1层耗时*/
            }
            if(g_floor_cur == run_target)     /*到达目标层→开门*/
            {
                RemoveFloor(run_target);
                xuanzhun_SetAngle(DOOR_OPEN_ANGLE);
                g_door_state = DOOR_OPENED;
                door_tick = 0;
                oled_refresh_flag = 1;
            }
#endif
            break;
        }

        default:
            g_door_state = DOOR_OPENED;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(DOOR_TASK_PERIOD));
    }
}

/*任务4：HX711 称重(量程1kg)
 * 读取方式与你验证通过的 HX711_GetData() 一致(通道A、增益128)；
 * 电梯升降(DOOR_RUNNING)期间不称重，静止时采样平均、换算克数刷新OLED*/
static void App_WeightTask(void * pvParameters)
{
    (void)pvParameters;

    /*上电唤醒：拉低SCK让HX711退出掉电模式并等第一次转换完成(否则永远读不到数据)*/
    HX711_Init();

    while(1)
    {
        /*电梯运行中：跳过本轮称重，保留上次读数*/
        if(g_door_state == DOOR_RUNNING)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        unsigned long sum = 0;
        uint8_t       cnt = 0;

        for(uint8_t k = 0; k < HX711_SAMPLE_N; k++)
        {
            /*HX711@10Hz每次转换约100ms：先轮询DOUT就绪再读，避免长忙等饿死其他任务
              超时放宽到500ms，覆盖上电复位后的第一次转换*/
            uint8_t ready = 0U;
            for(uint16_t tr = 0U; (tr < 100U) && (ready == 0U); tr++)
            {
                if(HX711_IsReady())
                {
                    ready = 1U;
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(5));   /*每5ms查询一次就绪*/
                }
            }
            if(ready != 0U)
            {
                sum += HX711_GetData();
                cnt++;
            }
            if(k < (HX711_SAMPLE_N - 1U))
                vTaskDelay(pdMS_TO_TICKS(20));   /*留出下次转换时间，实际以DOUT就绪为准*/
        }

        if(cnt == 0)        /*本轮一次都没读到：可能断线/未就绪，保留上次读数并稍等重试*/
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        {
            unsigned long avg = sum / cnt;
            g_hx_ready = 1U;            /*已成功通信一次*/

            /*上电后首次成功读数作为空载零点(与参考代码 reset 一致：请空载开机)*/
            if(g_reset_ok == 0U)
            {
                g_reset_raw = avg;
                g_reset_ok  = 1U;
            }

            /*Δ原始码取绝对值：传感器A+/A-接反时加压读数反而减小，取绝对值保证仍能出数*/
            long delta = (long)avg - (long)g_reset_raw;
            if(delta < 0L)
                delta = -delta;

            long scale = (long)HX711_STD_RAW - (long)g_reset_raw;   /*100g对应增量*/
            if(scale < 0L)
                scale = -scale;
            if(scale == 0L)
                scale = 1L;

            /*重量(克) = |Δ原始码| × 标准砝码克数 ÷ 100g增量*/
            int32_t w = (int32_t)((float)delta * HX711_STD_G / (float)scale + 0.5f);

            if(w != g_weight_gram)      /*读数变化才触发刷新*/
            {
                g_weight_gram = w;
                oled_refresh_flag = 1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(DOOR_TASK_PERIOD));
    }
}



void App_CreateTasks(void)
{
    /*创建一个队列*/
    xQueue1 = xQueueCreate(16, sizeof(uint8_t));
    /*创建一个二值型信号量*/
    xSemaphore1 = xSemaphoreCreateBinary();

    /*CAN：❗必须先建队列再初始化，因为中断回调里要用 g_can_rx_queue*/
#if CAN_ENABLE
    g_can_rx_queue    = xQueueCreate(16, sizeof(CanFrame_t));
    g_motor_cmd_queue = xQueueCreate(8,  sizeof(uint8_t));
    CAN_Init();
#endif
    /*创建任务一 按键输入 可换LVGL触摸屏通信*/
  xTaskCreate(App_InputTask, "input", 256, NULL, tskIDLE_PRIORITY + 3U, NULL);
  /*创建任务二 DMA传感器数据采集*/
  xTaskCreate(App_SensorTask, "sensor",256, NULL, tskIDLE_PRIORITY + 2U, NULL);
  /*创建任务三 电梯自动开关门控制*/
  xTaskCreate(App_DoorTask, "door", 256, NULL, tskIDLE_PRIORITY + 4U, NULL);
  /*创建任务四 HX711称重(电梯非运行时实时显示克数)*/
  xTaskCreate(App_WeightTask, "weight", 256, NULL, tskIDLE_PRIORITY + 1U, NULL);

#if CAN_ENABLE
  /*创建任务五 曳引执行(电机未到：本机延时模拟。真双节点时这个任务不编译，
    由驱动板B的独立工程承担)*/
#if TRACTION_LOCAL_SIM
  xTaskCreate(App_TractionTask, "traction", 192, NULL, tskIDLE_PRIORITY + 3U, NULL);
#endif
  /*创建任务六 CAN收发与协议分派(优先级最高：通信帧不能丢)*/
  xTaskCreate(App_CanTask, "can", 192, NULL, tskIDLE_PRIORITY + 5U, NULL);
#endif
}



//DMA回调函数只释放信号量
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance != ADC1)
        return;
    //防御：信号量还没初始化就直接返回，防止NULL指针崩溃
    if(xSemaphore1 == NULL)
        return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
