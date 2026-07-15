# 云台系统辨识移植与使用说明

## 为什么采用“少量宏 + 配置结构体”

- `GIMBAL_SYSID` 和 `GIMBAL_SYSID_STEP` 会改变条件编译，只能使用宏。
- 机构、速度环和运动范围相关参数放在一个带字段名及单位的配置结构体中，避免几十个初始化形参，也避免散落宏难以理解。
- 采样数量、滤波频率、拟合阈值保留为算法内部默认值。只有出现采样不足或拟合质量问题时才修改它们。

## 辨识状态与普通控制器相互独立

`GimbalController` 不需要包含任何辨识状态。模块在
`GimbalSystemID.h` 中公开独立上下文：

```c
extern GimbalSystemIDContext gimbal_sysid;
```

调试器直接查看：

```text
gimbal_sysid.yaw
gimbal_sysid.pitch
```

因此移植到一个不含 `Gimbal_SI` 成员的控制器结构体时，不需要修改对方的
结构体布局。`GimbalSystemID_Init()` 接收控制器指针，仅用于接入本项目的陀螺仪、
电机反馈和速度环；若目标项目的控制器命名不同，只需在系统辨识实现的输入/输出
接入位置做适配，不需要改辨识状态结构。

移植者主要接触三个位置：

1. `Application/inc/GimbalSystemIDConfig.h`：选择辨识轴、步骤，完成移植确认。
2. `Application/src/GimbalSystemIDConfig.c`：设置机构和测试运动参数。
3. `Application/src/Gimbal.c`：设置辨识时使用的速度环 PID。

复制模块到新工程时，还必须把 `GimbalSystemID.c` 和
`GimbalSystemIDConfig.c` 同时加入构建系统；只复制头文件会在链接时找不到
`gimbal_sysid_user_config`。

## 首次移植必须确认

### 1. 坐标和力矩符号

先用很小的正速度参考确认：

- Yaw 正方向应与项目定义一致。
- Pitch 正方向必须是向上。
- `torque_feedback_coef * t_ff_Receive` 在电机沿模型正方向发力时应为正。

如果正速度下模型力矩为负，修改对应的 `torque_feedback_coef`，不要在辨识公式中临时改符号。

### 2. Pitch 硬安全角度

手动缓慢移动 Pitch，记录 IMU 实际可达角度，再设置：

- `safe_min_deg`：略高于机械下限。
- `safe_max_deg`：略低于机械上限。

必须满足：

```text
safe_min < safe_max，并且可用跨度不少于 20°
```

这两个值是安全边界，不是普通控制限位。其余 Pitch 测试角度由程序自动生成：

- 扫描换向点距离上下边界 1°。
- 有效采样区和 J 运动区距离上下边界 2.5°。
- 三个 J 加减速切换点按运动区间四等分。
- 两个 J 配对中心位于相邻切换点中间。

调试器可查看 `gimbal_sysid.pitch_angle_layout`，确认自动生成的全部角度。
设置错误仍可能导致撞击机械限位，因此首次运行应低速观察换向是否正常。

### 3. 辨识专用速度环

辨识只使用速度环输出。先关闭辨识，以手动速度测试确认：

- 正负方向均能启动。
- 实际速度能稳定，不持续振荡。
- 输出不长期饱和。
- Pitch 上升和下降可使用不同参考速度，以得到相近的实际速度。

不要通过系统辨识运动顺便调 PID；应先把速度环调到可用，再开始采集。

### 4. Pitch 重力扫描速度

设置 `gravity_up_ref_dps` 和 `gravity_down_ref_dps`。两者不要求数值相等，要求实测上升和下降速度幅值接近。

例如：上升参考 50 得到实际 20，下降参考 1 得到实际 -20，则应保留不对称参考值。

### 5. Pitch B/C 三档速度映射

分别填写：

- `bc_up_ref_dps[3]`
- `bc_down_ref_dps[3]`
- `bc_expected_actual_dps[3]`

每个下标代表同一档实际速度。目标是同档上升、下降实际速度幅值接近，并覆盖低、中、高三个稳定运动速度。

### 6. Pitch J 运动范围

J 的起止、切换和配对角度不再手填，统一由 `safe_min_deg`、`safe_max_deg`
生成。`j_actual_peak_speed_dps` 应明显高于最低可稳定运动速度，但不能让输出饱和。

### 7. 老/新云台配置档案

`GimbalSystemIDConfig.c` 中分别保存 `OLD` 和 `NEW` 两套 Pitch 参数，切换时
不会覆盖另一套实测数据。默认档案由 `gimbal_sysid_pitch_profile_id` 决定；如果
整车能读取硬件版本，应在 `GimbalSystemID_Init()` 前选择：

```c
GimbalSystemID_SelectPitchProfile(GIMBAL_SYSID_PITCH_PROFILE_OLD);
/* 或 GIMBAL_SYSID_PITCH_PROFILE_NEW */
```

这属于运行时数据选择，不改变条件编译，也不要求在算法文件中增加车型宏。

### 8. Yaw B/C 和 J 运动

- `bc_speed_dps[8]` 必须同时包含正、负速度，推荐正负交替。
- 每档速度都必须是速度环能够稳定跟踪的值。
- `j_pair_center_dps` 必须位于 `0` 与 `j_max_ref_dps` 之间，并避开起停低速区。
- Yaw J 单方向三角速度轨迹的近似转角为：

```text
单方向转角 ≈ j_max_ref_dps² / j_ref_accel_dps2
```

当前配置约为 `200²/80=500°`。必须确认没有线缆缠绕或机械限位风险。

## 启用步骤

完成以上检查后，在 `GimbalSystemIDConfig.h` 中设置：

```c
#define GIMBAL_SYSID_PORTING_CONFIRMED 1
#define GIMBAL_SYSID GIMBAL_PITCH_SYSID   /* 或 GIMBAL_YAW_SYSID */
#define GIMBAL_SYSID_STEP GIMBAL_SYSID_STEP_ALL
```

编译、下载后，程序仍保持安全互锁。确认周围安全后，在调试器中手动设置：

```c
gimbal_sysid.pitch.sysid_done = 0;
/* 或 */
gimbal_sysid.yaw.sysid_done = 0;
```

辨识结束后必须恢复：

```c
#define GIMBAL_SYSID_PORTING_CONFIRMED 0
#define GIMBAL_SYSID GIMBAL_SYSID_DISABLED
```

## 结果判定

成功结束应满足：

```text
sysid_done  = 1
sysid_valid = 1
sysid_error = 0
```

常见错误：

| `sysid_error` | 含义 | 优先检查 |
|---:|---|---|
| 1 | 激励或有效平均点不足 | 实际速度、采样窗口、是否卡住 |
| 2 | Pitch触发安全角度 | 安全范围、角度符号、机械限位 |
| 3 | B/C/J出现非物理结果 | 力矩符号、重力模型、运动方向 |
| 4 | 不同配对点差异过大 | 加速度是否稳定、是否饱和、窗口是否合理 |
| 5 | 拟合残差过大 | 速度环振荡、结构松动、采样噪声 |
| 6 | 用户配置关系无效 | 配置数组、角度顺序、正负速度、确认项 |

## 推荐的移植顺序

1. 只验证方向、角度和力矩单位，不运动辨识。
2. 调好辨识专用速度环。
3. Pitch 按 `GRAVITY -> BC -> J` 分步运行并保存中间结果。
4. Yaw 按 `BC -> J` 分步运行。
5. 分步结果稳定后再使用 `ALL`。
6. 将最终参数写入控制模型，关闭辨识宏，重新验证普通控制。
