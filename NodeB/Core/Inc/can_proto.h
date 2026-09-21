/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_proto.h
  * @brief   主控板A ↔ 驱动板B 的 CAN 2.0A 通信协议
  *
  * 本文件是两块板共用的"接口契约"。
  * A 板侧的原始定义在  MXtext/Core/Src/app.c:301-354
  * ▸ 任何一条定义改动都必须两块板同步，否则通信会静默失败（不报错，只是看不懂）
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CAN_PROTO_H__
#define __CAN_PROTO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/*==================================================================
 * ID 位段(11bit)： [10:8]消息类型   [7:4]源节点   [3:0]目标节点
 * CAN 仲裁规则：ID 数值越小优先级越高
 *   → 消息类型按实时性倒排，急停排最小
 *==================================================================*/
#define CAN_ID_TYPE_SHIFT   8U
#define CAN_ID_SRC_SHIFT    4U
#define CAN_ID_TYPE_MASK    0x7U
#define CAN_ID_NODE_MASK    0xFU

/** @brief 消息类型（值越小总线优先级越高） */
typedef enum
{
  MSG_EMERGENCY = 0U,   /*!< 故障/急停：优先级最高(ID最小，能抢占总线) */
  MSG_MOTION_STA,       /*!< B→A  运行状态帧 */
  MSG_WEIGHT,           /*!< B→A  称重（本项目 B 板不做，A 板自己有 HX711） */
  MSG_DOOR_OK,          /*!< A→B  门已关好，允许运行（安全联锁） */
  MSG_CMD_GOTO          /*!< A→B  目标楼层 */
} CanMsgType_e;

/** @brief 节点号 */
typedef enum
{
  NODE_MAIN   = 1U,     /*!< 主控板 A */
  NODE_DRIVER = 2U      /*!< 驱动板 B（本工程） */
} CanNode_e;

/* 运行状态（MSG_MOTION_STA.data[1]） */
#define MOTOR_STA_IDLE     0U
#define MOTOR_STA_RUN      1U
#define MOTOR_STA_ARRIVED  2U
#define MOTOR_STA_FAULT    3U

/* 运行方向（MSG_MOTION_STA.data[2]）
   用 1/2 表示而不是 -1，避免 uint8 的符号坑 */
#define MOTOR_DIR_STOP     0U
#define MOTOR_DIR_UP       1U
#define MOTOR_DIR_DOWN     2U

/* B 板上报的故障码（MSG_EMERGENCY.data[0]）
   ❗A 板只把 EMERGENCY 当作"故障"，不解析具体码；这里是给调试用的 */
#define FAULT_NONE          0U
#define FAULT_DOOR_NOT_OK   1U   /*!< 没收到 DOOR_OK，安全联锁不允许运行 */
#define FAULT_BAD_TARGET    2U   /*!< 目标楼层越界 */
#define FAULT_ENCODER_LOST  3U   /*!< 运行中收不到编码器脉冲（断线或堵转） */
#define FAULT_MOVE_TIMEOUT  4U   /*!< 单次运行总超时没到站 */
#define FAULT_REMOTE_STOP   5U   /*!< 收到 A 的急停，半途停住 */
#define FAULT_WRONG_DIR     6U   /*!< 越走越远：A/B 相接反或方向映射配错 */

/*==================================================================
 * 帧 ID 拼装 / 拆解
 * 实际 ID（B 板用得到的）：
 *   MSG_CMD_GOTO   type=4 src=A(1) dst=B(2) → 0x412   A→B  目标楼层
 *   MSG_DOOR_OK    type=3 src=A(1) dst=B(2) → 0x312   A→B  允许运行
 *   MSG_MOTION_STA type=1 src=B(2) dst=A(1) → 0x121   B→A  运行状态
 *   MSG_EMERGENCY  type=0 src=B(2) dst=A(1) → 0x021   B→A  故障码
 *==================================================================*/
#define CAN_MAKE_ID(type,src,dst) \
        ((uint32_t)((((uint32_t)(type) << CAN_ID_TYPE_SHIFT) | \
                     ((uint32_t)(src)  << CAN_ID_SRC_SHIFT)) | \
                     ((uint32_t)(dst))))

#define CAN_GET_TYPE(id)  ((uint8_t)(((id) >> CAN_ID_TYPE_SHIFT) & CAN_ID_TYPE_MASK))
#define CAN_GET_SRC(id)   ((uint8_t)(((id) >> CAN_ID_SRC_SHIFT)  & CAN_ID_NODE_MASK))
#define CAN_GET_DST(id)   ((uint8_t)( (id)                       & CAN_ID_NODE_MASK))

/*==================================================================
 * 数据域约定（每帧只用前几个字节）
 *   MSG_CMD_GOTO  : [0]=目标楼层
 *   MSG_DOOR_OK   : [0]=1 允许运行
 *   MSG_MOTION_STA: [0]=当前楼层 [1]=运行状态 [2]=方向
 *   MSG_EMERGENCY : [0]=故障码
 *   MSG_WEIGHT    : [0]=重量低字节 [1]=重量高字节 [2]=超载标志
 *==================================================================*/

/** @brief 一帧 CAN 报文 */
typedef struct
{
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
} CanFrame_t;

/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTO_H__ */
