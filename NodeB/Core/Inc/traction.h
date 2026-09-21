/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    traction.h
  * @brief   驱动板 B：曳引状态机 + 编码器位置闭环
  *
  * 收到 A 板的 MSG_CMD_GOTO → 闭环把轿厢开到目标层 → 用 MSG_MOTION_STA 上报
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __TRACTION_H__
#define __TRACTION_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can_proto.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/**
  * @brief 控制节拍(ms)
  * @note  必须与 TIM3 的中断周期一致：tim.c 里 PSC=7199 ARR=99
  *        → 72MHz/7200/(99+1) = 100Hz = 10ms
  */
#define TRACTION_TICK_MS    10U

/**
  * @brief 单板台架自测开关
  * @note  1 = 上电 1s 后自动给自己塞一个"去 3 楼"的命令，不需要 A 板在线
  *        就能验证整条闭环链路。要配合 can.h 里的 CAN_LOOPBACK_TEST=1 一起用。
  */
#ifndef BENCH_AUTO_GOTO
#define BENCH_AUTO_GOTO     0
#endif

/**
  * @brief 编码器独立自检模式（不需要 A 板、不需要电机、不需要万用表）
  * @note  1 = 上电后立刻关掉电机驱动器（TB6612 输出高阻，输出轴变自由，
  *            手能拧动），并且忽略所有 CAN 命令，LED 只做一件事：
  *            每累计 500 个编码器脉冲闪一下。
  *
  *        用手慢慢转输出轴（从轴端看随便哪个方向都行）：
  *          看到 LED 在闪 → 编码器有输出，问题不在编码器
  *          一直不闪      → 编码器确实没有脉冲输出（查线序 / 霍尔已损坏）
  *
  *        ❗测完记得改回 0，否则 B 板不会响应 A 板。
  */
#ifndef BENCH_ENCODER_TEST
#define BENCH_ENCODER_TEST  0
#endif

/* USER CODE END EC */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/**
  * @brief 调试观测点（不参与控制）
  * @note  本板没有串口也没有屏，出故障时靠这个结构定位。
  *        Keil：View → Watch Windows → Watch 1，输入 g_dbg，
  *        勾上 View → Periodic Window Update 就能实时看而不停机。
  *        关键两个字段：
  *          g_dbg.delta —— 恒为 0 就是编码器没信号（→ 查编码器供电/接线）
  *          g_dbg.fault_code —— 具体故障码，见 can_proto.h 的 FAULT_xxx
  */
typedef struct
{
  uint32_t ticks;           /*!< ❗心跳：每个 10ms 中断 +1。看到它在涨才说明 Watch 是活的；
                                 不涨的话，你看到的任何 0 都不可信（窗口冻结/程序没在跑） */
  uint8_t  state;           /*!< 0=IDLE 1=MOVE 2=FAULT */
  uint8_t  fault_code;      /*!< FAULT_xxx；0=无故障。注意它会随恢复清零，只存活几十 ms */
  uint8_t  last_fault_code; /*!< ❗最近一次故障码，锁存不清零 —— 看这个最靠谱 */
  uint8_t  fault_cnt;       /*!< 累计故障次数：一直在涨 = 正在故障-重试循环里 */
  uint8_t  cur_floor;     /*!< 当前楼层 */
  uint8_t  target_floor;  /*!< 目标楼层 */
  uint8_t  dir;           /*!< MOTOR_DIR_xxx */
  uint16_t door_ok_ticks; /*!< DOOR_OK 联锁剩余有效拍数(每拍10ms) */
  uint16_t duty;          /*!< 当前实际占空比(%) */
  int32_t  pos;           /*!< 当前位置(脉冲)：电机转起来时这个数必须变化 */
  int32_t  err;           /*!< 目标位置 - 当前位置(脉冲) */
  int32_t  last_arrival_err;/*!< 上次到站瞬间的残余误差(脉冲，带符号)。
                                 在"重挂楼层网格"之前记录。正常应在 ±100 以内；
                                 若达几千 → 是控制环没收敛，不是标定问题 */
  int16_t  delta;         /*!< 本拍编码器增量：恒为 0 = 编码器没信号 */
} TractionDbg_t;

extern volatile TractionDbg_t g_dbg;

/*==================================================================
 * Watch 窗口专用的独立标量
 *------------------------------------------------------------------
 * 有些 Keil 版本展开 volatile 结构体有问题（显示 cannot evaluate、
 * 也不给展开用的加号）。这里额外导出几个独立标量，在 Watch 窗口里
 * 直接输入名字即可，不依赖 Keil 的结构体展开能力。
 *   看编码器：g_dbg_pos / g_dbg_delta
 *   看心跳  ：g_dbg_ticks（必须一直在涨，否则你看到的 0 不可信）
 *==================================================================*/
extern volatile int32_t  g_dbg_pos;      /*!< 当前位置(脉冲)：手转轴时应该变化 */
extern volatile int16_t  g_dbg_delta;    /*!< 本拍编码器增量：恒为 0 = 没脉冲 */
extern volatile int32_t  g_dbg_ticks;    /*!< 心跳：每个 10ms +1 */
extern volatile uint16_t g_dbg_duty;     /*!< 当前占空比(%) */
extern volatile uint8_t  g_dbg_fault;    /*!< 最近一次故障码 */
extern volatile int32_t  g_dbg_arr_err;  /*!< 上次到站残余误差(脉冲) */

/* USER CODE END ET */

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化：位置归零（本板没有平层传感器，上电默认认作在 1F） */
void Traction_Init(void);

/**
  * @brief  控制节拍任务，在 TIM3 的 10 ms 中断里调用
  * @note   只做状态机 + 位置闭环 + 置"需要上报"标志，不发 CAN
  */
void Traction_Tick(void);

/**
  * @brief  CAN 接收回调，在接收中断里调用
  * @note   只改状态标志，不做任何会产生等待的操作
  */
void Traction_OnCanFrame(const CanFrame_t *frame);

/**
  * @brief  主循环调用：把 Traction_Tick 置好的上报请求真正发到总线上
  * @note   所有 CAN 发送都集中在这里，避免在中断里等待发送完成
  */
void Traction_Poll(void);

/**
  * @brief  刷新调试观测点，在 TIM3 中断里 Traction_Tick() 之后调用
  * @note   单独放在 Tick 外面，保证任何提前 return 的分支都能刷到
  */
void Traction_DbgSync(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TRACTION_H__ */
