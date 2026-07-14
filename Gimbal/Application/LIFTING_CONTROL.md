# 升降控制维护说明

本文面向后续维护人员。阅读代码时先看本文件和 `lifting_control.c`，只有调试某种传感器保护时才进入对应 protection 文件。

## 1. 文件职责

| 文件 | 职责 |
| --- | --- |
| `inc/lifting_config.h` | 所有目标值、PID、时间、电流、距离和角度参数；参数名带单位 |
| `inc/lifting_types.h` | 状态、故障和当前机型的调试结构体 |
| `src/lifting_control.c` | 1kHz状态机、串级PID、到位判定、故障停机和恢复入口 |
| `src/lifting_tof_protection.c` | OLD：TOF毛刺、遮挡恢复、快速堵转、空转和里程比例 |
| `src/lifting_encoder_protection.c` | NEW：编码器帧统计、堵转和空转 |
| `src/encoder.c` | 外置绝对编码器全局数据和有效帧计数 |

机型由 `Config/Robot_config.h` 中的 `ROBOT_SELECT` 决定。OLD和NEW只会编译各自的保护结构和实现。

## 2. 1kHz主流程

```text
Lifting_Control
  -> 检查遥控需求是否变化
  -> 执行当前phase
  -> 检查位置传感器和M2006是否在线
  -> OLD在PID前过滤TOF新帧并检查遮挡/空转
  -> 位置PID：位置误差 -> 速度目标
  -> 速度PID：速度误差 -> C610电流
  -> 执行当前机型的堵转/空转保护
  -> 连续满足到位条件后进入COMPLETE
```

PID输入单位：

- OLD位置环：mm。
- NEW位置环：编码器count。
- 两种机型速度环：M2006减速器输出轴deg/s，因此速度环参数不能随位置量程缩放。

## 3. 状态与故障

### 状态 `lifting_controller.phase`

| 状态 | 含义 |
| --- | --- |
| `LIFT_PHASE_IDLE` | 待机，电流为0 |
| `LIFT_PHASE_ASCENDING` | 上升到high_state |
| `LIFT_PHASE_DESCENDING` | 等待云台回正后下降到low_state |
| `LIFT_PHASE_BLOCKED` | 故障停机 |
| `LIFT_PHASE_COMPLETE` | 已稳定到达目标，电流为0 |

### 故障 `lifting_controller.protection.common.fault`

| 故障 | 自动恢复 | 处理策略 |
| --- | --- | --- |
| `LIFT_FAULT_FEEDBACK_OFFLINE` | NEW可以；OLD不自动 | 立即停机 |
| `LIFT_FAULT_MOTOR_STALL` | 否 | 改变遥控升降需求后重新启动 |
| `LIFT_FAULT_MOTOR_SPIN` | 否 | 检查丝杆、联轴器和传动连接 |
| `LIFT_FAULT_TOF_OCCLUDED` | OLD可以 | 连续可信TOF帧确认后恢复故障前phase |

堵转和空转不得自动重试，避免机械故障未解除时反复冲击。

## 4. OLD保护

### 快速碰顶

快速通道不等待TOF，只比较C610电流和M2006累计角度：

```text
指令电流或反馈电流达到门槛
+ 100ms内M2006未有效前进
+ 连续两个异常窗口
= LIFT_FAULT_MOTOR_STALL
```

启动即被压住时约250ms停机。该通道在下降切换上升的TOF稳定期内仍然工作。

### TOF毛刺和遮挡

- 只在 `Tof_UpdateCounter` 增加时处理一次数据，1kHz循环不会重复使用旧帧。
- 单帧疑似跳变只隔离，不进入位置PID。
- 连续异常帧才确认遮挡并进入BLOCKED。
- 遮挡停机后，连续新帧与遮挡前里程预测吻合才自动续跑。

### 空转

空转判断由真实可信TOF帧驱动。每收到配置数量的新帧评估一次：

```text
M2006已经前进较大里程
+ TOF基本没有变化
+ 连续多个帧窗
= LIFT_FAULT_MOTOR_SPIN
```

不要再把TOF帧数绑定到固定的1kHz时间窗口；传感器周期变化时只会改变确认耗时，不会制造“窗口样本不足”。

## 5. NEW保护

编码器是断电保持的绝对位置，正常控制直接追踪标定目标；保护只比较100ms窗口内的相对变化。

```text
编码器未沿目标方向前进：
  M2006前进达到空转门槛 -> MOTOR_SPIN
  否则持续高负载       -> MOTOR_STALL
```

每个窗口必须收到足够的新编码器帧。新帧由 `Encoder_UpdateCounter` 识别，不能把旧值重复计数。

## 6. 参数修改规则

只在 `inc/lifting_config.h` 修改升降参数。

重点参数组：

- `LIFT_TOF_*_POINT_MM`：老云台高低点。
- `LIFT_ENCODER_*_POINT_COUNT`：新云台绝对编码器高低点。
- `*_POSITION_KP`、`LIFT_SPEED_KP/KD`：串级PID。
- `*_FINISH_*`：到位误差和保持时间。
- `*_FAST_STALL_*`：快速碰顶。
- `LIFT_TOF_OCCLUSION_*`、`LIFT_TOF_RECOVERY_*`：遮挡与恢复。
- `LIFT_TOF_SPIN_*`、`LIFT_ENCODER_*`：空转和编码器联合保护。

调整原则：

- 时间缩短、位移门槛调大、电流门槛调低：响应更快，但误报风险增加。
- 时间加长、位移门槛调小、电流门槛调高：更稳，但机械持续受力时间增加。
- 修改前记录正常上升、正常下降、碰顶和空转数据，不凭感觉同时修改多个参数。

## 7. Debug观察

两种机型都先看：

- `lifting_controller.phase`
- `lifting_controller.protection.common.fault`
- `lift_pos_pid.Err`
- `lift_speed_pid.Ref`
- `lift_info.angle/speed/torque_current`
- `send_current`

OLD快速堵转：

- `fast_startup_ticks`
- `fast_window_ticks`
- `fast_effort_ticks`
- `fast_motor_progress`
- `fast_stall_windows`

OLD遮挡和空转：

- `trusted_tof`
- `tof_delta`
- `odometry_residual`
- `occlusion_frames`
- `recovery_frames`
- `spin_window_frames`
- `spin_windows`
- `tof_per_motor_degree`、`ratio_valid`

NEW编码器：

- `encoder.value`、`Encoder_UpdateCounter`
- `encoder_samples`
- `motor_progress`、`sensor_progress`
- `effort_ticks`
- `stall_windows`、`spin_windows`

## 8. 修改后的最低测试集合

每次修改升降逻辑后至少验证：

1. 正常上升并到位。
2. 正常下降并到位。
3. 启动即碰顶。
4. 上升途中碰顶。
5. 传动空转。
6. 位置传感器离线。
7. M2006反馈离线。
8. OLD单帧TOF毛刺不会停机。
9. OLD持续遮挡会停机，遮挡解除后自动恢复。
10. 改变遥控需求可以解除堵转/空转锁定。

OLD和NEW都必须完成Keil全工程编译，不能只验证当前机型。
