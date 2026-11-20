# Real-Cable 实机直接速度控制

`deploy_real_hexapod_tethered_speed.py` 对应 MJLab 的
`hexapod_tethered_estimate_real_cable` slope 策略：

- Actor 输入为 69 维，机身线速度由 GRU 估计。
- 前 18 维动作是腿关节位置动作。
- 第 19 维是收放绳线速度，单位 `m/s`：正值放绳，负值收绳。
- 实机不再使用“目标拉力 + 张力 PD/前馈”计算卷扬速度。
- 卷筒编码器提供实际线速度，拉力传感器提供实际拉力观测。
- 绝对自然绳长不进入 Actor；配置初值加编码器相对转角得到的 L0 仅用于内部
  软限位和日志，不应视为实测真值。

## 配置文件

策略接口、观测/动作缩放以及卷扬绳模型的可调参数位于：

```text
deploy/deploy_real/configs/hexapod_tethered_real_cable.yaml
```

对应的读取和参数检查代码是：

```text
deploy/deploy_real/config_hexapod_tethered_real_cable.py
```

运行前主要检查 YAML 中的这些字段：

```yaml
policy_path: "{ROOT_DIR}/deploy/pre_train/hexapod_tethered_estimate_real_cable/policy.pt"

winch_radius_m: 0.06
winch_gear_ratio: 1.0
winch_initial_length_m: 5.0
winch_payout_sign_from_motor: -1.0
winch_max_motor_rpm: 157.0
```

串口地址、波特率、EtherCAT 网卡、启动等待、日志刷新频率和关节 PD
沿用其他实机部署脚本的风格，直接写在主脚本中，不放进 YAML。

`policy_path` 必须指向 Real-Cable 的 stateful TorchScript policy，不能使用旧的
effort policy，也不能直接使用训练保存的原始 `model_*.pt`。

`winch_initial_length_m` 只是启动姿态下人为配置的外露自然绳长估计值。旋转
编码器只能测量长度变化，不能自动得到绝对绳长；该估计只影响软限位和日志，
不会送入 Actor。如果编码器位于电机侧，
`winch_gear_ratio` 应填写“电机转角 / 卷筒转角”。

实机正式行走前应先悬空低速确认方向：正的第 19 维动作必须实际放绳。如果相反，
将 `winch_payout_sign_from_motor` 改为相反号。

## 运行

```bash
cd /home/lgl/Hexapod_Sim2Real/deploy/deploy_real
python real_cable_experiment/deploy_real_hexapod_tethered_speed.py
```

日志保存在 `real_cable_experiment/data_logs/`，包含完整 19 维动作、策略目标绳速、
实际下发绳速、编码器实际绳速、内部自然长度估计和实测拉力。
