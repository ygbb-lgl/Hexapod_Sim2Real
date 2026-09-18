# Hexapod Sim2Real

本工程由 Unitree 实机部署框架改造而来，用于将 PyTorch/TorchScript 策略部署到纯六足机器人和绳驱六足机器人。工程包含策略推理、手柄状态机、IMU/张力/绳方向传感器、EtherCAT 电机通信、绳轮张力控制、数据采集和离线建模工具。

> 安全提示：本工程会向实机电机下发位置、速度或力矩指令。运行前必须架起机器人、确认电机映射/方向/零位、传感器和手柄按键，并确保 LB 阻尼退出可用。不要在电机未卸载或人员位于机器人运动范围内时直接运行。

## 1. 当前主要入口

| 场景 | 主脚本 | 配置 | 说明 |
| --- | --- | --- | --- |
| 纯六足 | `deploy/deploy_real/deploy_real_hexapod.py` | `configs/hexapod.yaml` | 当前纯六足实机策略入口，18 个关节，包含 IMU、手柄、电机 CSV 日志和阻尼退出。 |
| 绳驱六足（速度式绳轮） | `deploy_real_hexapod_tethered.py` | `configs/hexapod_tethered.yaml` | 18 个腿关节加 1 个绳轮电机，策略输出目标张力，张力控制器生成绳轮转速。 |
| 绳驱六足（力矩式绳轮） | `deploy_real_hexapod_tethered_plot_torque.py` | `configs/hexapod_tethered.yaml` | 使用绳轮力矩控制，并实时绘制张力、绳方向和绳轮指令。 |

纯六足入口的基本状态机是：

1. 启动后进入零力矩状态。
2. 按 `Y` 运动到默认姿态。
3. 按 `A` 进入策略控制。
4. 策略运行或默认姿态等待期间按 `LB` 进入阻尼状态。
5. 策略运行时按 `Y` 返回默认姿态，之后可按 `A` 继续策略，或按 `LB` 进入阻尼。

## 2. 目录结构

| 路径 | 用途 |
| --- | --- |
| `deploy/deploy_real/` | 实机部署主程序、配置读取、数据记录和离线工具。 |
| `deploy/deploy_real/configs/` | YAML 运行参数：策略路径、控制周期、观测/动作维度、电机映射、增益和绳控制参数。 |
| `deploy/pre_train/` | 部署用 TorchScript 策略和张力预测模型。 |
| `deploy/deploy_real/common/` | 安全指令、Unitree 遥控器解析和坐标旋转工具。 |
| `deploy/deploy_real/motor_igh_sdk/` | EL4090 + PySOEM 电机通信及 PD/位置/速度/力矩控制封装。 |
| `deploy/deploy_real/imu_sdk_deta40/` | 当前 DETA40/FDILink IMU 读取、坐标系转换和速度估计。 |
| `deploy/deploy_real/hexapod_tethered_utils/` | 绳驱传感器、手柄、张力控制、运动学和 MoE/预测模型部署工具。 |
| `deploy/deploy_real/unitree_sdk2py/` | Unitree SDK2 Python/IDL 支持文件，主要服务于原 G1/H1/Go2 部署框架。 |

## 3. `deploy/deploy_real/` 脚本说明

### 3.1 实机部署入口

| 脚本 | 作用 |
| --- | --- |
| `deploy_real_hexapod.py` | 纯六足主程序；组装 66 维观测，调用 18 维策略，下发 18 个关节 PD 指令，并保存每个电机的跟踪日志。 |
| `deploy_real_hexapod_tethered.py` | 基础绳驱入口；在腿部策略之外读取张力、绳端俯仰角和绳臂航向角，通过转速模式控制绳轮。 |
| `deploy_real_hexapod_tethered_speed.py` | 带实时曲线和 CSV 日志的绳轮速度控制实验版。 |
| `deploy_real_hexapod_tethered_plot.py` | 基础速度式绳驱程序的绘图/日志版。 |
| `deploy_real_hexapod_tethered_plot_torque.py` | 将绳轮改为力矩模式，记录并绘制张力、航向、速度和绳轮力矩。 |
| `deploy_real_hexapod_tethered_plot_torque_data.py` | 力矩式部署加训练数据采集；以因果时序记录状态、上一步/当前力矩和绳张力，供 `pre_tension.py` 使用。 |
| `deploy_real_hexapod_tethered_plot_torque_data_constant.py` | 恒定绳轮力矩对照版；当前代码把绳轮指令固定为 13.14 Nm，用于基线比较。 |
| `deploy_real_hexapod_tethered_plot_torque_data_trex.py` | TReX 文献方法对照版；根据斜坡、摩擦和机器人速度生成确定性目标张力。 |
| `deploy_real_hexapod_tethered_plot_torque_data_nagatani.py` | Nagatani 方法对照版；利用机器人姿态、绳方向和 MJCF 运动学计算目标张力。 |
| `deploy_real_hexapod_tethered_plot_torque_data_polzin.py` | Polzin–Hughes 方法对照版；基于六足接触/支撑几何搜索可用拉力方向和大小。 |
| `deploy_real_hexapod_tethered_plot_torque_data_moe.py` | 递归 MoE 策略部署版；使用 68 维观测、GRU 隐状态和多专家 actor，同时记录绳控制数据。 |
| `deploy_real_hexapod_tethered_plot_torque_pre.py` | 短时域张力预测模型部署版；在候选绳轮力矩中选择预测张力更合适的指令，并保留 shadow/debug 数据。 |
| `deploy_only_cable.py` | 不运行腿部策略，单独调试张力跟踪和绳轮速度控制。 |
| `deploy_only_cable_torque.py` | 不运行腿部策略，单独调试张力跟踪和绳轮力矩控制。 |
| `deploy_real.py` | 原 Unitree G1/H1/H1_2 通用部署入口，通过 DDS/SDK2 与原厂机器人通信。 |
| `deploy_real_go2.py` | 原 Unitree Go2 部署入口，主要用于保留原工程参考。 |

### 3.2 配置读取

| 脚本 | 作用 |
| --- | --- |
| `config_hexapod.py` | 读取 `configs/hexapod.yaml`，提供纯六足策略、观测、动作和电机映射参数。 |
| `config_hexapod_tethered.py` | 读取绳驱 YAML，除腿部参数外还包含绳轮速度/力矩控制器增益和限幅。 |
| `config_hexapod_tethered_moe.py` | 在绳驱配置上增加 MoE/GRU 策略路径、观测合同和部署检查。 |
| `config.py` | 原 G1/H1/H1_2 通用 YAML 读取器。 |
| `config_go2.py` | 原 Go2 YAML 读取器。 |

`configs/hexapod.yaml` 对应纯六足；`hexapod_tethered.yaml` 对应普通绳驱策略；`hexapod_tethered_moe.yaml` 对应 MoE 策略。`g1.yaml`、`go2.yaml`、`h1.yaml` 和 `h1_2.yaml` 为原 Unitree 机型配置。

### 3.3 数据、模型与调试工具

| 脚本 | 作用 |
| --- | --- |
| `convert_checkpoint_to_jit.py` | 从 RSL-RL checkpoint 重建 actor/观测归一化器，导出实机用 TorchScript policy。脚本内含固定输入/输出路径，使用前需检查。 |
| `check_keys.py` | 打印 PyTorch checkpoint 的 key，检查 actor 和 observation normalizer 是否存在。当前模型路径在脚本内硬编码。 |
| `test_load_model.py` | 加载/检查部署模型的小型测试脚本。 |
| `plot_motor_curves.py` | 读取 `motor_logs_*` 中的 18 个电机 CSV，按 HAA/HFE/KFE 绘制目标位置和实际位置曲线。 |
| `replay_rear_leg_trajectory.py` | 在右后腿或左后腿重放已记录轨迹；其他关节保持阻尼，用于单腿跟踪试验。 |
| `measure_cable_k.py` | 用逐步增加的绳轮力矩测量张力响应，保存到 `cable_data/K/`，用于识别绳系统参数。 |
| `measure_cable_kc.py` | 通过绳轮位置模式采集张力—位置数据，保存到 `cable_data/K_C/`。 |
| `generate_synthetic_pre_tension_data.py` | 生成确定性 50 Hz 合成 CSV，用于测试张力数据和训练流程；不能当作实机质量评估数据。 |
| `merge_pre_tension_csv.py` | 校验并合并多条张力轨迹 CSV，保留 `trajectory_id`、时间戳和力矩因果顺序。 |
| `pre_tension.py` | 检查张力 CSV，构造严格时序窗口，训练动作条件短时域张力预测模型。 |

## 4. 支持模块说明

### 4.1 `common/`

| 脚本 | 作用 |
| --- | --- |
| `command_helper_hexapod.py` | 为 EL4090 六足封装生成零力矩、阻尼、零速度等安全指令。 |
| `command_helper.py` | 原 Unitree SDK2 机型的命令初始化/安全指令辅助函数。 |
| `remote_controller.py` | 解析 Unitree 遥控器原始字节、按键和摇杆。 |
| `rotation_helper.py` | 四元数、重力方向和机体坐标变换工具。 |

### 4.2 `hexapod_tethered_utils/`

| 脚本 | 作用 |
| --- | --- |
| `joystick_reader.py` | 通过 Pygame 读取 Xbox 手柄，生成 `vx/vy/wz` 指令并提供 A/Y/LB 等按键状态。 |
| `cable_tension_sensor.py` | 读取绳张力传感器，输出标定后的张力值。 |
| `cable_tension_sensor_zero.py` | 张力传感器清零/零点标定脚本。 |
| `cable_end_pitch_sensor.py` | 读取绳端俯仰角传感器。 |
| `cable_arm_yaw_sensor.py` | 读取绳臂/挂点航向角传感器。 |
| `cable_arm_yaw_differ_test.py` | 联合驱动绳轮并读取 yaw 传感器，用于方向差和电机方向调试。 |
| `cable_arm_yaw_differ_test_1.py` | yaw 差测试的另一个实验版本，使用前应与主版对比。 |
| `tension_speed_controller.py` | 张力误差 + 速度/绳方向前馈的绳轮转速控制器，包含 PID、滤波、死区和限幅。 |
| `tension_torque_controller.py` | 对应的绳轮力矩控制器。 |
| `learned_tension_torque_selector.py` | 加载 `pre_tension.py` 模型，维护历史窗口，批量评估候选力矩并选择指令。 |
| `moe_policy.py` | 只包含部署所需的递归 MoE actor、观测构造和 checkpoint 加载，不依赖 MuJoCo/RSL-RL。 |
| `hexapod_mjcf_kinematics.py` | 从六足 MJCF 参数构建轻量正运动学，供 Polzin/Nagatani 对照控制使用。 |
| `test_unit.py` | 将 yaw/pitch 转为机体坐标系单位绳方向的简单测试。 |

历史/试验文件：

- `cable_tension_sensor copy.py`：张力传感器旧副本。
- `tension_speed_controller copy.py` 和 `tension_speed_controller copy 2.py`：张力—转速控制器的旧参数/算法版本。
- `hexapod_tethered_utils_gkq/diff1.py`、`test.py`、`test1.py`、`test_01.py`：个人/阶段性差分和数值试验，运行前需阅读源码确认输入。

### 4.3 `motor_igh_sdk/`

| 脚本 | 作用 |
| --- | --- |
| `el4090_motor_sdk.py` | EL4090 EtherCAT/CAN 数据结构、帧打包/解包和位置、速度、力矩指令底层函数。 |
| `deploy_real_el4090_pysoem.py` | 18 个六足关节的 PySOEM 主站封装，负责 policy index↔motor ID、方向/零位转换和 PD 收发循环。 |
| `deploy_real_el4090_pysoem_spool_speed.py` | 在 18 关节封装上增加 motor 19 绳轮速度模式。 |
| `deploy_real_el4090_pysoem_spool_torque.py` | 增加 motor 19 绳轮力矩模式。 |
| `deploy_real_el4090_pysoem_spool_position.py` | 增加 motor 19 绳轮位置模式。 |
| `deploy_real_el4090_speed_pysoem.py` | 单电机速度模式测试封装。 |
| `deploy_real_el4090_torque_pysoem.py` | 单电机力矩模式测试封装。 |
| `deploy_real_el4090_position_pysoem.py` | 单电机位置模式测试封装。 |
| `rl_real_el4090_demo.py` | 不连真实硬件的映射/缓冲区演示。 |

`motor_igh_sdk_hexapod/` 是较早的六足 EL4090 封装副本：`el4090_motor_sdk.py` 是底层帧编解码，`deploy_real_el4090_pysoem.py` 是关节通信封装，`rl_real_el4090_demo.py` 是映射演示。当前纯六足主程序导入的是 `motor_igh_sdk/deploy_real_el4090_pysoem.py`。

### 4.4 IMU

| 脚本 | 作用 |
| --- | --- |
| `imu_sdk_deta40/imu_sdk.py` | 当前主程序使用的 IMU SDK；解析串口数据，提供角速度、重力、线加速度和积分线速度。 |
| `imu_sdk_deta40/test_imu.py` | 实时打印 IMU 姿态/惯性数据的命令行测试。 |
| `imu_sdk_deta40/test_vel.py` | 测试速度积分、静止检测和 ZUPT 参数。 |
| `imu_sdk_deta40/demo.py` | FDILink 原始串口示例。 |
| `imu_sdk/imu_sdk.py` | 旧版 IMU 实现，主入口已改用 `imu_sdk_deta40/`。 |
| `imu_sdk/test_imu.py` | 旧版 IMU 数据打印测试。 |
| `imu_sdk/test_vel.py` | 旧版 IMU 线速度测试。 |

## 5. 根目录脚本

| 脚本 | 作用 |
| --- | --- |
| `startup.sh` | 先对张力传感器清零，然后启动指定的绳驱部署脚本。当前含绝对路径，需与实机用户名/环境保持一致。 |
| `env.sh` | 激活 `hexapod_lgl` Conda 环境并设置 HTTP/HTTPS 代理。 |
| `setup.py` | Python 包安装信息和基础依赖。 |
| `test_pack.py` | 检查单条电机 ctypes 结构的字节对齐/大小。 |
| `test_full_pack.py` | 检查完整 EtherCAT 消息 ctypes 结构的字节大小。 |

## 6. 其他文档与自动生成代码

- `deploy/deploy_real/MOTOR_DEBUG_LOGGING_GUIDE.md`：电机 CSV 日志和曲线绘制说明。
- `deploy/deploy_real/pre_data/README.md`：张力预测数据集说明。
- `deploy/deploy_real/pre_data/plot.py`：预张力 CSV 数据的快速绘图检查脚本。
- `deploy/deploy_real/README.md` 和 `README.zh.md`：原 Unitree 实机部署文档。
- `deploy/deploy_real/SIM2REAL_file_roles.md`：`common/` 和 `unitree_sdk2py/` 的更细文件索引。
- `unitree_sdk2py/`：大量 IDL/RPC/DDS 文件为 SDK 自动生成或原工程支持代码，不建议为六足功能直接修改。

## 7. 运行前检查

- 核对 YAML 中的 `policy_path`、观测维度、动作维度、`joint2motor_idx`、`motor_directions` 和 `motor_offsets`。
- 核对网卡名、IMU 串口、张力/yaw/pitch 传感器串口以及 udev 别名。部分脚本仍使用硬编码网卡或绝对路径，命令行参数未必会覆盖它们。
- 先分别测试手柄、IMU、传感器和单电机，再启动整机策略。
- 绳轮的位置、速度和力矩测试脚本会直接动作 motor 19，运行前必须卸载或限定张力。
- 带 `copy`、`test`、人名或文献名后缀的脚本主要用于对照实验，不要在未检查配置的情况下当作默认入口。

---

## 原 README 内容（命令和注释完整保留）

## 本项目是根据unitree go2机器人改的sim2real六足代码，目前只有hexapod是可以用的，hexapod_tethered还没有作出更改

### configs文件夹中存放了yaml文件，在config_xxx中进行读取，deploy_real_hexapod.py中是真正的sim2real主程序


### 执行sim2real的bash指令
```bash
sudo /home/lgl/anaconda3/envs/mujoco_rl/bin/python /home/lgl/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod.py enp109s0
```
```bash
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod_tethered.py enp86s0
```

```bash
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/motor_igh_sdk/deploy_real_el4090_pysoem_spool_speed.py
```
udev
```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```
test yaw angle
```bash
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python  /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/hexapod_tethered_utils/cable_arm_yaw_differ_test.py --ifname enp86s0 --motor-id 19 --slave-idx 3 --passage 1 --rpm 0 --current-limit-01a 500 --yaw-port /dev/ttyUSB_yaw --yaw-slave-id 1
```

```bash 
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/deploy_only_cable.py --tension-ref 100 --plot --arm-with-gamepad 
```

```bash 
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/deploy_only_cable_torque.py --tension-ref 50 --plot --arm-with-gamepad 
```

```bash
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/motor_igh_sdk/deploy_real_el4090_pysoem_spool_torque.py
```

```bash 
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod_tethered_speed.py enp86s0
```

#### deploy/deploy_real/deploy_real_hexapod_tethered_plot.py 和 /home/lgl/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod_tethered_speed.py未必可以
