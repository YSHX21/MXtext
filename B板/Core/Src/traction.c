/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    traction.c
  * @brief   驱动板 B：曳引状态机 + 编码器位置闭环
  *
  * 职责（与 A 板约定的分工）：
  *   收 MSG_CMD_GOTO   → 用编码器闭环把轿厢开到目标层
  *   发 MSG_MOTION_STA → 上报 [当前楼层, 运行状态, 方向]
  *
  * ❗楼层权威在 B 板（编码器在这边），A 板只是镜像本板报的楼层。
  * ❗A 板在 DOOR_RUNNING 下只有看到 MOTOR_STA_RUN 才会启动看门狗；
  *   如果本板在 A 等待期间发 IDLE，A 既不会超时也不会到站 → 永久卡死。
  *   所以本板"只在有消息时上报"，绝不主动发 IDLE。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "traction.h"
#include "motor.h"
#include "encoder.h"
#include "can.h"

/* USER CODE BEGIN 0 */

/*==================================================================
 * 一、标定参数（按实际机械调整）
 *==================================================================*/

/**
  * @brief 一层楼对应多少编码器脉冲（❗实测标定值，不要改回理论值）
  * @note  理论值 = 11PPR × 4倍频 × 377减速比 = 16588，
  *        但实测输出轴转整整一圈只有 15916 个脉冲。
  */
#define PULSES_PER_FLOOR      15916

#define FLOOR_MIN             1U     /*!< 最低层，与 A 板一致 */
#define FLOOR_MAX             12U    /*!< 最高层，与 A 板一致(A 板 dispatch 范围 1~12F) */

/**
  * @brief "上行"对应编码器计数是正还是负
  * @note  取决于 A/B 相接线。上行时 CNT 递减就把它改成 0。
  *        （可用 encoder.c 的自检 Encoder_CalStart() 判定后再改这里）
  */
#define UP_IS_POSITIVE        1

/*==================================================================
 * 二、位置闭环参数
 *==================================================================*/
/**
  * @brief 到站窗口(脉冲)
  * @note  必须大于"一个控制拍最多能走多远"，否则高速时会整拍跨过目标而漏判。
  *        一拍(10ms)行程 ≈ 输出轴转速(rpm)/60 × 16588 / 100，取 150 留了
  *        约 54rpm 输出轴转速的余量；另外还有"误差变号"判据兜底。
  */
#define STOP_WINDOW           150
#define SLOWDOWN_ERR          ((int32_t)(PULSES_PER_FLOOR / 2))  /*!< 剩余半层开始减速 */
#define DUTY_CRUISE           55U    /*!< 远距离巡航占空比(%) */
#define DUTY_MIN_MOVE         28U    /*!< 低速段占空比下限：太小克服不了静摩擦 */

/*==================================================================
 * 三、安全与看门狗
 *==================================================================*/
/**
  * @brief MSG_DOOR_OK 的有效窗口
  * @note  A 板在发 CMD_GOTO 之前会先发 DOOR_OK（门关好才允许运行 = 安全联锁）。
  *        A 的看门狗 2s 会重发一次 CMD_GOTO（但不再重发 DOOR_OK），
  *        所以窗口取 3s，重发时联锁依然有效。
  */
#define DOOR_OK_VALID_MS      3000U
#define DOOR_OK_VALID_TICKS   (DOOR_OK_VALID_MS / TRACTION_TICK_MS)

#define STALL_MS              600U   /*!< 占空比已到位却连续这么久没脉冲 → 堵转/编码器断线 */
#define STALL_TICKS           (STALL_MS / TRACTION_TICK_MS)

/**
  * @brief 单次运行总超时（最后一道兜底）
  * @note  取得很宽松：1 层 ≈ 输出轴 1 圈 ≈ 2.5s，跑到 12F 也就 40s 左右。
  *        真正的保护是下面的堵转检测和"反向检测"，这个只防"跑飞了还没被
  *        前面两道抓住"的极端情况。100000/10 = 10000 拍，uint16 装得下。
  */
#define MOVE_TIMEOUT_MS       100000U
#define MOVE_TIMEOUT_TICKS    (MOVE_TIMEOUT_MS / TRACTION_TICK_MS)

/**
  * @brief 反向检测：误差连续这么多拍都在变大，就说明在往反方向跑
  * @note  这是 A/B 相接反或 UP_IS_POSITIVE 配错时的唯一保护 ——
  *        这种情况下脉冲是有的（堵转检测抓不到），电机会一路冲到机械限位。
  *        ramp 期间轿厢还没动，误差不变大，所以不会误报。
  */
#define AWAY_TICKS            100U   /*!< 100 × 10ms = 1s */

/**
  * @brief 运行中状态帧周期
  * @note  必须远小于 A 板看门狗(RUN_WARN_TICKS = 2000ms/50ms = 40 拍 = 2s)，
  *        否则 A 会误判丢帧并重发命令
  */
#define STA_PERIOD_MS         200U
#define STA_PERIOD_TICKS      (STA_PERIOD_MS / TRACTION_TICK_MS)

/*==================================================================
 * 四、内部状态
 *==================================================================*/
typedef enum
{
  TR_IDLE = 0U,   /*!< 停靠中，等命令 */
  TR_MOVE,        /*!< 闭环运行中 */
  TR_FAULT        /*!< 故障，等 A 重新下发命令才恢复 */
} TractionState_t;

static TractionState_t   s_state        = TR_IDLE;
static uint8_t           s_cur_floor    = FLOOR_MIN;   /*!< 当前楼层 */
static uint8_t           s_target_floor = FLOOR_MIN;   /*!< 本次目标楼层 */
static int32_t           s_pos_base     = 0;           /*!< 位置零点：pos = base + Encoder_Read() */
static uint8_t           s_dir          = MOTOR_DIR_STOP;
static uint8_t           s_fault_code   = FAULT_NONE;
static int8_t            s_last_err_sign = 0;   /*!< 上一拍的位置误差符号，用于"跨过目标"判据 */

/* 中断与主循环之间传递（CAN 接收中断 → Traction_Tick） */
static volatile int16_t  s_pending_goto    = -1;   /*!< -1 = 无新命令 */
static volatile uint16_t s_door_ok_ticks   = 0U;   /*!< DOOR_OK 剩余有效拍数 */
static volatile uint8_t  s_stop_req        = 0U;   /*!< 收到急停 */

/* 上报请求（Traction_Tick → 主循环 Traction_Poll） */
static volatile uint8_t  s_rep_req   = 0U;
static uint8_t           s_rep_floor = FLOOR_MIN;
static uint8_t           s_rep_state = MOTOR_STA_IDLE;
static uint8_t           s_rep_dir   = MOTOR_DIR_STOP;

static uint16_t          s_stall_ticks = 0U;
static uint16_t          s_move_ticks  = 0U;
static uint16_t          s_sta_div     = 0U;
static uint16_t          s_away_ticks  = 0U;
static int32_t           s_last_abs_err = 0;

/* 调试观测点（见 traction.h），不参与控制 */
volatile TractionDbg_t   g_dbg;

/* Watch 窗口专用的独立标量（免结构体展开），见 traction.h 的说明 */
volatile int32_t  g_dbg_pos     = 0;
volatile int16_t  g_dbg_delta   = 0;
volatile int32_t  g_dbg_ticks   = 0;
volatile uint16_t g_dbg_duty    = 0;
volatile uint8_t  g_dbg_fault   = 0;
volatile int32_t  g_dbg_arr_err = 0;

/*==================================================================
 * 五、板载 LED 状态指示（调试用，与 CAN 协议无关）
 *------------------------------------------------------------------
 * 本板没有串口也没有屏，用最小系统板自带的 LED 把状态报出来。
 * 最小系统板的板载 LED 一般挂在 PC13（低电平点亮）。
 * ❗如果你的板子 LED 不在 PC13，改下面的 LED_GPIO_PORT / LED_PIN 即可。
 *
 * 规则（故障优先）：
 *   ① 出过故障 → 持续循环播报故障码：闪 N 次 → 停约 1s → 再闪 N 次
 *        闪烁次数就是故障码：3=ENCODER_LOST  6=WRONG_DIR  1=DOOR_NOT_OK ...
 *        成功到站一次后自动清除，恢复正常显示
 *   ② 没出过故障 → 电机驱动期间每累计 500 个编码器脉冲闪一下
 *        闪 = 编码器有信号（兼作心跳灯）；一直不闪 = 编码器没信号
 *==================================================================*/
#define LED_GPIO_PORT      GPIOC
#define LED_PIN            GPIO_PIN_13
#define LED_ON()           HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()          HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET)

#define LED_SLOT_TICKS     15U      /* 一个亮/灭步 = 150ms */
#define LED_GAP_SLOTS      7U       /* 组间隔 ≈ 1s */
#define LED_PULSE_STEP     500      /* 每 500 个编码器脉冲闪一下 */
#define LED_PULSE_HOLD     3U       /* 每次闪 30ms */

static uint8_t  s_led_code       = 0U;   /* 锁存的故障码，0 = 没出过故障 */
static uint8_t  s_led_phase      = 0U;
static uint8_t  s_led_count      = 0U;
static uint8_t  s_led_gap        = 0U;
static uint16_t s_led_timer      = 0U;
static int32_t  s_led_pulse_acc  = 0;
static uint8_t  s_led_pulse_hold = 0U;
static uint8_t  s_force_pulse_led = 0U;  /*!< 编码器自检模式下强制 LED 显示脉冲 */

/**
  * @brief 上电自检灯计时：LED 常亮 60 拍 = 600ms
  * @note  必须先证明 LED 本身是好的。否则"编码器不闪"和"LED 坏 / LED_PIN
  *        假设错了"这两种情况看起来一模一样，会得出完全错误的结论。
  *        用"常亮 600ms"而不是闪几次，是为了和任何故障码闪烁模式都区分得开。
  */
static uint16_t s_led_boot_ticks = 60U;

#if BENCH_AUTO_GOTO
static uint16_t          s_bench_ticks = 0U;
#endif

/* Private function prototypes -----------------------------------------------*/

static void    Traction_LedInit(void);
static void    Traction_LedTick(void);
static int32_t Traction_GetPos(void);
static int32_t Traction_TargetPos(uint8_t floor);
static void    Traction_ReqReport(uint8_t state, uint8_t dir);
static void    Traction_Fault(uint8_t code);
static void    Traction_StartMove(uint8_t target);

/* Private functions ---------------------------------------------------------*/

/** @brief 当前位置（脉冲），以 1F 为 0 点 */
static int32_t Traction_GetPos(void)
{
  return s_pos_base + Encoder_Read();
}

/** @brief 目标楼层对应的绝对位置（脉冲） */
static int32_t Traction_TargetPos(uint8_t floor)
{
  return ((int32_t)(floor - FLOOR_MIN)) * PULSES_PER_FLOOR;
}

/** @brief 置上报请求，真正的发送由主循环做 */
static void Traction_ReqReport(uint8_t state, uint8_t dir)
{
  s_rep_floor = s_cur_floor;
  s_rep_state = state;
  s_rep_dir   = dir;
  s_rep_req   = 1U;
}

/** @brief 进故障态：立刻刹停 + 上报 FAULT */
static void Traction_Fault(uint8_t code)
{
  Motor_Brake();

  s_state      = TR_FAULT;
  s_dir        = MOTOR_DIR_STOP;
  s_fault_code = code;

  /* 只累加不清零：一直在涨就说明在故障-重试循环里。
     last_fault_code 之所以要单独锁存一份，是因为 A 板几十毫秒后就会重发命令，
     clear 掉故障态，实时的 fault_code 根本来不及被看到。 */
  g_dbg.fault_cnt++;
  g_dbg.last_fault_code = code;

  /* 让板载 LED 也开始播报这个故障码（取最近一次，不用锁首个：
     首次故障可能只是 B 板 CAN 刚起来还没就绪之类的偶发） */
  s_led_code = code;

  Traction_ReqReport(MOTOR_STA_FAULT, MOTOR_DIR_STOP);
}

/**
  * @brief 接受一个目标层并开始运行
  * @note  判据用"目标位置的误差"，而不是"楼层号是否相同"：故障半途停下后
  *        轿厢可能停在两层之间，按楼层号比会误判成"已经在本层"直接报到站，
  *        机械上其实偏了。用位置误差就不会。
  */
static void Traction_StartMove(uint8_t target)
{
  int32_t err;

  /* 收到合法命令即从故障态恢复（A 会重新发 DOOR_OK + CMD_GOTO） */
  s_fault_code    = FAULT_NONE;
  s_target_floor  = target;
  s_last_err_sign = 0;

  err = Traction_TargetPos(target) - Traction_GetPos();

  if ((err >= -STOP_WINDOW) && (err <= STOP_WINDOW))
  {
    /* 已经在目标位置：直接报到站，行为与 A 板的本地模拟一致 */
    g_dbg.last_arrival_err = err;
    Encoder_Reset();
    s_pos_base  = Traction_TargetPos(target);
    s_cur_floor = target;
    s_state     = TR_IDLE;
    s_dir       = MOTOR_DIR_STOP;

    Traction_ReqReport(MOTOR_STA_ARRIVED, MOTOR_DIR_STOP);
    return;
  }

  /* 误差 > 0 = 要让计数增大才能到目标 */
  if (err > 0)
  {
    s_dir = (UP_IS_POSITIVE != 0) ? MOTOR_DIR_UP : MOTOR_DIR_DOWN;
  }
  else
  {
    s_dir = (UP_IS_POSITIVE != 0) ? MOTOR_DIR_DOWN : MOTOR_DIR_UP;
  }

  s_stall_ticks  = 0U;
  s_move_ticks   = 0U;
  s_sta_div      = 0U;
  s_away_ticks   = 0U;
  s_last_abs_err = (err < 0) ? -err : err;
  s_state        = TR_MOVE;

  Traction_ReqReport(MOTOR_STA_RUN, s_dir);
}

/* ------------------------------- LED 指示 ------------------------------- */

static void Traction_LedInit(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Pin   = LED_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;   /* PC13/14/15 是低驱动脚，只能用低速档 */
  HAL_GPIO_Init(LED_GPIO_PORT, &gpio);

  LED_OFF();
}

static void Traction_LedTick(void)
{
  /* ⓪ 上电自检：LED 常亮 600ms。
     看到这段常亮 = LED 是好的、引脚假设正确，"不闪"才有诊断意义。
     看不到 → 说明 LED 不在 PC13（或者板子没有板载 LED），
              那么后面的"不闪"结论全部作废，得改用调试器/万用表。 */
  if (s_led_boot_ticks > 0U)
  {
    s_led_boot_ticks--;
    LED_ON();
    return;
  }

  /* ① 出过故障 → 优先且持续播报故障码。
     不能按"电机是否在驱动"来切换：故障-重试循环里电机每约 1s 就动一下，
     故障态只存在几十毫秒，根本没有一整段时间留给播报，连一次都闪不完。 */
  if (s_led_code == 0U)
  {
    /* ② 没出过故障：驱动期间（或编码器自检模式）用闪烁表示"编码器有脉冲" */
    if ((Motor_GetTargetDuty() > 0U) || (s_force_pulse_led != 0U))
    {
      int32_t d = Encoder_GetDelta();

      if (s_led_pulse_hold > 0U)
      {
        s_led_pulse_hold--;
        LED_ON();
        return;
      }

      LED_OFF();

      if (d < 0)
      {
        d = -d;
      }

      s_led_pulse_acc += d;
      if (s_led_pulse_acc >= LED_PULSE_STEP)
      {
        s_led_pulse_acc  = 0;
        s_led_pulse_hold = LED_PULSE_HOLD;
        LED_ON();
      }
    }
    else
    {
      LED_OFF();
    }
    return;
  }

  /* ③ 播报故障码：闪 N 次 → 停约 1s → 再闪 N 次 */
  s_led_pulse_acc  = 0;
  s_led_pulse_hold = 0U;

  s_led_timer++;
  if (s_led_timer < LED_SLOT_TICKS)
  {
    return;
  }
  s_led_timer = 0U;

  switch (s_led_phase)
  {
    case 0U:                          /* 开始一轮播报 */
      LED_ON();
      s_led_count = 1U;
      s_led_phase = 1U;
      break;

    case 1U:                          /* 亮够一段 → 灭 */
      LED_OFF();
      s_led_phase = 2U;
      break;

    case 2U:                          /* 灭够一段 → 还有码位就再亮 */
      if (s_led_count < s_led_code)
      {
        LED_ON();
        s_led_count++;
        s_led_phase = 1U;
      }
      else
      {
        s_led_phase = 3U;
        s_led_gap   = 0U;
      }
      break;

    default:                          /* 3 = 组间隔，停约 1s 后重来 */
      if (s_led_gap < LED_GAP_SLOTS)
      {
        s_led_gap++;
      }
      else
      {
        s_led_gap   = 0U;
        s_led_phase = 0U;
      }
      break;
  }
}

/* Exported functions --------------------------------------------------------*/

void Traction_Init(void)
{
  /* 调试用 LED（板载 PC13）：先初始化，后面故障时用来播报故障码 */
  Traction_LedInit();

  /* 本板没有平层传感器：上电默认认作停在 1F，并把当前位置当作 0 点。
     与非易失存储配合的话这里可以改成读上次的绝对位置。 */
  Encoder_Reset();

  s_pos_base     = 0;
  s_cur_floor    = FLOOR_MIN;
  s_target_floor = FLOOR_MIN;
  s_state        = TR_IDLE;
  s_dir          = MOTOR_DIR_STOP;
  s_fault_code   = FAULT_NONE;
  s_last_err_sign = 0;

  s_pending_goto  = -1;
  s_door_ok_ticks = 0U;
  s_stop_req      = 0U;

  s_rep_req   = 0U;
  s_rep_floor = FLOOR_MIN;
  s_rep_state = MOTOR_STA_IDLE;
  s_rep_dir   = MOTOR_DIR_STOP;

  s_stall_ticks  = 0U;
  s_move_ticks   = 0U;
  s_sta_div      = 0U;
  s_away_ticks   = 0U;
  s_last_abs_err = 0;

#if BENCH_AUTO_GOTO
  s_bench_ticks = 0U;
#endif

#if BENCH_ENCODER_TEST
  /* 编码器独立自检：关掉驱动器让输出轴自由。
     不关的话 TB6612 处于"短路制动"(PWM=0 且 AIN1≠AIN2)，
     377:1 减速箱的轴硬得基本拧不动，手转测试会得出错误结论。 */
  Motor_Disable();
  s_force_pulse_led = 1U;
#endif
}

void Traction_OnCanFrame(const CanFrame_t *frame)
{
  if (frame == NULL)
  {
    return;
  }

  switch (CAN_GET_TYPE(frame->id))
  {
    case MSG_CMD_GOTO:                    /* A→B：去 N 楼 */
      if (frame->len >= 1U)
      {
        s_pending_goto = (int16_t)frame->data[0];
      }
      break;

    case MSG_DOOR_OK:                     /* A→B：门已关好，允许运行 */
      if ((frame->len >= 1U) && (frame->data[0] != 0U))
      {
        s_door_ok_ticks = DOOR_OK_VALID_TICKS;
      }
      break;

    case MSG_EMERGENCY:                   /* A 目前不发；收到就当急停，宁可停错不可不停 */
      s_stop_req = 1U;
      break;

    default:
      break;
  }
}

void Traction_Tick(void)
{
  int32_t pos;
  int32_t err;
  int32_t abs_err;
  int16_t goto_floor;
  uint16_t duty;

  /* ⓪ LED 指示（每拍都要跑，不能放在任何提前 return 之后） */
  Traction_LedTick();

#if BENCH_ENCODER_TEST
  /* 编码器自检模式：不驱动电机、不理会 CAN 命令，只让 LED 反映编码器脉冲 */
  return;
#endif

  /* ① 联锁窗口倒计时 */
  if (s_door_ok_ticks > 0U)
  {
    s_door_ok_ticks--;
  }

#if BENCH_AUTO_GOTO
  /* 单板台架自测：等 CAN 起来后自己给自己发一个"去 3 楼" */
  if (s_bench_ticks < (1000U / TRACTION_TICK_MS))
  {
    s_bench_ticks++;
    if (s_bench_ticks == (1000U / TRACTION_TICK_MS))
    {
      s_door_ok_ticks = DOOR_OK_VALID_TICKS;   /* 没有 A 板，自己给自己授权 */
      s_pending_goto  = 3;
    }
  }
#endif

  /* ② 处理 A 下发的目标（中断里只置标志，这里消费） */
  goto_floor = s_pending_goto;
  if (goto_floor >= 0)
  {
    s_pending_goto = -1;

    if ((goto_floor < (int16_t)FLOOR_MIN) || (goto_floor > (int16_t)FLOOR_MAX))
    {
      Traction_Fault(FAULT_BAD_TARGET);
      return;
    }

    if (s_door_ok_ticks == 0U)
    {
      /* 没收到 DOOR_OK：门没关好，安全联锁不允许运行。
         报 FAULT 后 A 会退回 DOOR_CLOSED 重新调度，不会死锁。 */
      Traction_Fault(FAULT_DOOR_NOT_OK);
      return;
    }

    Traction_StartMove((uint8_t)goto_floor);
  }

  /* ③ A 下发的急停 */
  if (s_stop_req != 0U)
  {
    s_stop_req = 0U;

    /* 半途停住绝不能报到站：那会让 A 在错误楼层开门。
       一律报 FAULT，让 A 退回 DOOR_CLOSED 重新调度。 */
    Traction_Fault(FAULT_REMOTE_STOP);
    return;
  }

  if (s_state != TR_MOVE)
  {
    return;
  }

  /* ④ 位置闭环：目标位置 - 当前位置 */
  pos     = Traction_GetPos();
  err     = Traction_TargetPos(s_target_floor) - pos;
  abs_err = (err < 0) ? -err : err;

  /* 到站判据有两条，任一成立即算到站：
     ① 误差进了到站窗口
     ② 本拍跨过了目标（误差符号相对上一拍翻转）
     高速时一拍可能整拍跨过目标层，只靠 ① 会漏判，然后一直冲下去 */
  {
    int8_t  err_sign = (err > 0) ? 1 : ((err < 0) ? -1 : 0);
    uint8_t arrived  = 0U;

    if (abs_err <= STOP_WINDOW)
    {
      arrived = 1U;
    }
    if ((s_last_err_sign != 0) && (err_sign != 0) && (err_sign != s_last_err_sign))
    {
      arrived = 1U;
    }
    s_last_err_sign = err_sign;

    if (arrived != 0U)
    {
      /* 到站：刹停，并把位置重新挂到楼层网格上。
         每到一个楼层就重挂一次，避免多次运行的累积误差把楼层算歪。 */
      g_dbg.last_arrival_err = err;   /* 记在重挂之前，这才是真实的残余误差 */

      Motor_Brake();
      Encoder_Reset();
      s_pos_base  = Traction_TargetPos(s_target_floor);
      s_cur_floor = s_target_floor;
      s_state     = TR_IDLE;
      s_dir       = MOTOR_DIR_STOP;

      s_led_code = 0U;   /* 成功到站一次 → 清掉故障码播报，恢复正常显示 */

      Traction_ReqReport(MOTOR_STA_ARRIVED, MOTOR_DIR_STOP);
      return;
    }
  }

  /* ⑤ 方向（只取决于目标层在当前位置的上方还是下方，中途不变） */
  if (s_dir == MOTOR_DIR_UP)
  {
    Motor_SetDir((UP_IS_POSITIVE != 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE);
  }
  else
  {
    Motor_SetDir((UP_IS_POSITIVE != 0) ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD);
  }

  /* ⑥ 占空比：远处巡航，进入减速区后随误差线性下降 */
  if (abs_err >= SLOWDOWN_ERR)
  {
    duty = DUTY_CRUISE;
  }
  else
  {
    duty = (uint16_t)(DUTY_MIN_MOVE +
           (uint16_t)(((uint32_t)(DUTY_CRUISE - DUTY_MIN_MOVE) * (uint32_t)abs_err) /
                      (uint32_t)SLOWDOWN_ERR));
  }
  Motor_SetDuty(duty);

  /* ⑦ 楼层跟随：楼层权威的实时值 */
  {
    int32_t f = 1 + (pos / PULSES_PER_FLOOR);

    if (f < (int32_t)FLOOR_MIN)
    {
      f = (int32_t)FLOOR_MIN;
    }
    if (f > (int32_t)FLOOR_MAX)
    {
      f = (int32_t)FLOOR_MAX;
    }
    s_cur_floor = (uint8_t)f;
  }

  /* ⑧ 堵转 / 编码器断线检测
        只在软启动ramp到位之后才判定，否则起步阶段会误报 */
  if ((Motor_GetDuty() >= Motor_GetTargetDuty()) && (Encoder_GetDelta() == 0))
  {
    s_stall_ticks++;
    if (s_stall_ticks >= STALL_TICKS)
    {
      Traction_Fault(FAULT_ENCODER_LOST);
      return;
    }
  }
  else
  {
    s_stall_ticks = 0U;
  }

  /* ⑨ 反向检测：误差连续变大 = 在往反方向跑（A/B 相接反或方向映射错） */
  if (abs_err > s_last_abs_err)
  {
    s_away_ticks++;
    if (s_away_ticks >= AWAY_TICKS)
    {
      Traction_Fault(FAULT_WRONG_DIR);
      return;
    }
  }
  else
  {
    s_away_ticks = 0U;
  }
  s_last_abs_err = abs_err;

  /* ⑩ 总超时兜底 */
  s_move_ticks++;
  if (s_move_ticks >= MOVE_TIMEOUT_TICKS)
  {
    Traction_Fault(FAULT_MOVE_TIMEOUT);
    return;
  }

  /* ⑪ 运行中周期上报，喂 A 板的看门狗 */
  s_sta_div++;
  if (s_sta_div >= STA_PERIOD_TICKS)
  {
    s_sta_div = 0U;
    Traction_ReqReport(MOTOR_STA_RUN, s_dir);
  }
}

void Traction_Poll(void)
{
  uint8_t  floor;
  uint8_t  state;
  uint8_t  dir;
  uint8_t  code;
  uint32_t primask;

  if (s_rep_req == 0U)
  {
    return;
  }

  /* 极短临界区：把上报请求整体取出来并清标志，避免被 10ms 中断撕裂 */
  primask = __get_PRIMASK();
  __disable_irq();
  floor     = s_rep_floor;
  state     = s_rep_state;
  dir       = s_rep_dir;
  code      = s_fault_code;
  s_rep_req = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  /* 先报状态帧，让 A 能正确退出 DOOR_RUNNING；
     故障时再补一帧带故障码的 EMERGENCY（A 收到也会置 FAULT） */
  (void)Can_SendMotionSta(floor, state, dir);

  if (state == MOTOR_STA_FAULT)
  {
    (void)Can_SendEmergency(code);
  }
}

void Traction_DbgSync(void)
{
  g_dbg.ticks++;                     /* 心跳：证明 Watch 窗口是活的 */
  g_dbg.state         = (uint8_t)s_state;
  g_dbg.fault_code    = s_fault_code;
  g_dbg.cur_floor     = s_cur_floor;
  g_dbg.target_floor  = s_target_floor;
  g_dbg.dir           = s_dir;
  g_dbg.door_ok_ticks = s_door_ok_ticks;
  g_dbg.duty          = Motor_GetDuty();
  g_dbg.pos           = Traction_GetPos();
  g_dbg.err           = Traction_TargetPos(s_target_floor) - g_dbg.pos;
  g_dbg.delta         = (int16_t)Encoder_GetDelta();

  /* 同步到独立标量，方便在 Watch 窗口直接看 */
  g_dbg_pos     = g_dbg.pos;
  g_dbg_delta   = g_dbg.delta;
  g_dbg_ticks   = (int32_t)g_dbg.ticks;
  g_dbg_duty    = g_dbg.duty;
  g_dbg_fault   = g_dbg.last_fault_code;
  g_dbg_arr_err = g_dbg.last_arrival_err;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
