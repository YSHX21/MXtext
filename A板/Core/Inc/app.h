/*
 * @Author: yao 3547780037@qq.com
 * @Date: 2026-08-22 13:11:51
 * @LastEditors: yao 3547780037@qq.com
 * @LastEditTime: 2026-09-09 11:44:36
 * @FilePath: \MDK-ARMd:\CudeMxTest\MXtext\Core\Inc\app.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef APP_H
#define APP_H
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "gpio.h"
#include "OLED.h"
#include "button4_4.h"
#include "timers.h"
#include "semphr.h"
#include "adc.h"
#include <string.h>
#include "can.h"
typedef struct
{
    uint8_t row;
    uint8_t col;
}OledPos_t;

static const OledPos_t pos_table[] =
{
    {1, 2},   //val=1
    {1, 6},   //val=2
    {1, 10},  //val=3
    {1, 14},  //val=4
    {2, 2},   //val=5
    {2, 6},   //val=6
    {2, 10},  //val=7
    {2, 14},  //val=8
	{3, 2},   //val=9
    {3, 6},   //val=10
    {3, 10},  //val=11
    {3, 14},  //val=12
};

extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim3;
static QueueHandle_t xQueue1;
static TimerHandle_t xTimer1;
static QueueHandle_t xSemaphore1;
static BaseType_t jb;

/*从队列中接收数据的有效值判断*/
#define ARR_MAX_LEN  16U      //数组最大容量，根据需要改
static uint16_t g_data_buf[ARR_MAX_LEN]; //存放去重后有效数据
static uint8_t  g_data_cnt = 0U;         //数组当前有效元素个数
static uint8_t oled_refresh_flag=0;          //OLED刷新标志
static uint16_t ADC_Value[150];
static uint32_t ad1,ad2,ad3;

/*================= 电梯自动开关门 参数配置 =================*/
/*SG90 舵机角度宏：按实际安装方向填写(把两个值对调即可反转开关方向)*/
#define DOOR_OPEN_ANGLE     0.0f    /*门全开角度(原描述起始开门=0°)*/
#define DOOR_CLOSE_ANGLE  180.0f    /*门全关角度*/

#define DOOR_TASK_PERIOD    50U     /*门控任务节拍(ms)*/
#define DOOR_OPEN_MS       5000U    /*无阻挡保持开门等待时间(ms)*/
#define DOOR_START_DELAY_MS 800U    /*关好门后到启动运行的间隔(ms)：模拟松闸/发车，不能关完门就走*/
#define FLOOR_RUN_MS       1000U    /*模拟升降：每经过一层耗时(ms)*/

/*关门防夹：ad2 < DOOR_BLOCK_TH 判定为有阻挡/有人(阈值按实际标定调整)*/
#define DOOR_BLOCK_TH    1000U

/*关门采用“渐进步进”方式，保证关门过程可被随时打断：
  从开门角开始每次推进 DOOR_CLOSE_STEP(°)，每推进一步等待 DOOR_STEP_MS 并持续检测阻挡，
  发现阻挡立即弹回开门角。总关门耗时 ≈ |开角-关角|/STEP * STEP_MS*/
#define DOOR_CLOSE_STEP  12.0f   /*每步推进角度(°)，越小关门越慢、检测越密*/
#define DOOR_STEP_MS     150U    /*每步等待/防夹检测窗口(ms)，须为 DOOR_TASK_PERIOD 整数倍*/

typedef enum
{
    DOOR_OPENED = 0,   /*门开着，等待/检测阻挡*/
    DOOR_CLOSING,      /*正在关门(防夹窗口内)*/
    DOOR_CLOSED,       /*已关好，等待选层目标*/
    DOOR_RUNNING       /*升降运行中(延时模拟)*/
}DoorState_e;

static volatile DoorState_e g_door_state = DOOR_OPENED; /*门状态(供OLED显示)*/
static volatile uint8_t     g_floor_cur  = 1U;          /*电梯当前所在楼层(起始1F)*/
static uint16_t door_tick  = 0;   /*门控计时器(拍数)*/
static uint8_t  run_target = 0;   /*本次运行目标层*/

/*================= 运行模式总开关 =================*/
/* MODE_PURE_LOCAL = 完全不用CAN，走最初的本地延时逻辑(保险丝/对比用)
 *
 * MODE_LOCAL_SIM  =【当前默认】单板自测。CAN 回环自发自收 + 本机延时模拟曳引。
 *                   电机和第二块板都没到也能跑，而且 CAN 链路和协议是全程真实走的：
 *      A发CMD_GOTO →(回环)→ 本机曳引模拟 →(回环)→ A收到MOTION_STA → 到站开门
 *                   等驱动板B做好，把宏改成 MODE_DUAL_NODE，A 的业务代码一行都不用改
 *
 * MODE_DUAL_NODE  = 真双节点：A 只管调度，升降完全由驱动板B执行 */
#define MODE_PURE_LOCAL  0
#define MODE_LOCAL_SIM   1
#define MODE_DUAL_NODE   2
#ifndef WORK_MODE
#define WORK_MODE       MODE_DUAL_NODE   /*真双节点：A只管调度，升降由驱动板B执行*/
#endif

/*下面三个由 WORK_MODE 自动推导，不要手改*/
#if   (WORK_MODE == MODE_PURE_LOCAL)
#define CAN_ENABLE         0
#define CAN_LOOPBACK_TEST  0
#define TRACTION_LOCAL_SIM 0
#elif (WORK_MODE == MODE_LOCAL_SIM)
#define CAN_ENABLE         1
#define CAN_LOOPBACK_TEST  1   /*回环：总线上没有其他节点时，不发这个会因收不到ACK而一直重发失败*/
#define TRACTION_LOCAL_SIM 1   /*本机模拟曳引(电机没到)*/
#else
#define CAN_ENABLE         1
#define CAN_LOOPBACK_TEST  0   /*真双节点：两块板都要接上，且两端各接一个120Ω终端电阻*/
#define TRACTION_LOCAL_SIM 0
#endif

/*曳引模拟任务节拍(ms)*/
#define TRACTION_PERIOD   50U
/*看门狗：DOOR_RUNNING 里超过这么久没收到任何状态帧就重发命令
  (正常运行时每层1s会来一帧，所以 2s 只在真丢帧时才触发)*/
#define RUN_WARN_TICKS    ((uint16_t)(2000U / DOOR_TASK_PERIOD))

/*曳引侧上报的状态(由CAN接收任务更新，主控侧只读)*/
static volatile uint8_t g_motor_cur_floor = 1U;  /*当前楼层(权威在曳引侧，因为它有编码器)*/
static volatile uint8_t g_motor_state     = 0U;  /*见 MOTOR_STA_xxx*/
static uint8_t          g_can_sent_target = 0U;  /*已下发的目标层(避免重复发)*/
static uint16_t         g_run_wait_tick   = 0U;  /*看门狗计数：多久没收到状态帧了*/
static uint8_t          g_run_retry       = 0U;  /*命令重发次数*/

void App_CreateTasks(void);

#endif 
