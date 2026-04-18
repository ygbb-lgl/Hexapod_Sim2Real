import numpy as np
import time
import torch
import csv
import os
from datetime import datetime

from imu_sdk.imu_sdk import IMUSDK

from common.command_helper_hexapod import create_zero_cmd,create_damping_cmd

from motor_igh_sdk.deploy_real_el4090_pysoem import RL_Real_PySOEM

from hexapod_tethered_utils.joystick_reader import Gamepad

from hexapod_tethered_utils.cable_tension_sensor import CableTensionSensor
from hexapod_tethered_utils.cable_end_pitch_sensor import CableEndPitchSensor
from hexapod_tethered_utils.cable_arm_yaw_sensor import CableArmYawSensor


import config_hexapod_tethered

from config_hexapod_tethered import Config


class Controller:
    def __init__(self,config:Config) -> None:
        self.config = config
        #self.remote_controller = RemoteController()

        # 手柄（pygame）用于生成 cmd；遥控器仍用于按钮状态机（start/A/select）
        self.gamepad = None
        if Gamepad is not None:
            try:
                self.gamepad = Gamepad()
            except Exception as e:
                self.gamepad = None
                print(f"[gamepad] disabled due to error: {e}")

        self.ang_vel_scale = config.ang_vel
        self.dof_pos_scale = config.dof_pos
        self.dof_vel_scale = config.dof_vel
        self.lin_vel_scale = config.lin_vel

        # Policy-side per-joint gains (indexed by policy_idx 0..17)
        self._kp_policy, self._kd_policy = self._build_policy_joint_gains()

        self.policy = torch.jit.load(config.policy_path)
        # 预热网络，减少第一次推理的延迟
        self._warm_up()
        
        # 初始化状态和命令
        self.qj = np.zeros(config.num_leggeds_actions,dtype=np.float32)
        self.dqj = np.zeros(config.num_leggeds_actions,dtype=np.float32)
        self.action = np.zeros(config.num_actions,dtype=np.float32)
        self.target_dof_pos = config.default_angles.copy()
        self.obs = np.zeros(config.num_obs,dtype=np.float32)
        self.cmd = np.array([0, 0, 0],dtype=np.float32)
        self.counter = 0

        # Create log directory with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(os.getcwd(), f'motor_logs_{timestamp}')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Open 18 CSV files for logging motor data
        self.log_files = []
        self.log_writers = []
        for i in range(18):
            motor_id = int(self.config.joint2motor_idx[i])
            filepath = os.path.join(self.log_dir, f'motor_{motor_id:02d}_policy_idx_{i:02d}.csv')
            log_file = open(filepath, 'w', newline='')
            writer = csv.writer(log_file)
            writer.writerow(['time_step', 'counter', 'target_pos', 'actual_pos', 'error', 'action'])
            self.log_files.append(log_file)
            self.log_writers.append(writer)
            print(f'[Logging] Created {filepath}')

        # 电机初始化 EtherCAT
        self.robot = RL_Real_PySOEM('enp86s0')
        self.robot_start = self.robot.start()
        if not self.robot_start:
            print("[WARNING] Robot start failed. Will use zero data.")

        # imu init
        self.imu = IMUSDK(port='/dev/ttyUSB1', baudrate=921600)
        self.imu_started = self.imu.start()
        if not self.imu_started:
            print("[WARNING] IMU start failed. Will use zero data.")

        # 六位力传感器初始化 RS232
        self.tension_sensor = CableTensionSensor(port='/dev/ttyUSB2', baudrate=115200)
        self.tension_started = self.tension_sensor.start()
        if not self.tension_started:
            print("[WARNING] Tension sensor start failed. Will use zero data.")
        
        # 末端绝对角度传感器初始化 RS485
        self.pitch_sensor = CableEndPitchSensor(port='/dev/ttyUSB3', baudrate=9600)
        self.pitch_started = self.pitch_sensor.start()
        if not self.pitch_started:
            print("[WARNING] Pitch sensor start failed. Will use zero data.")

        # 磁栅编码器初始化 RS485
        self.yaw_sensor = CableArmYawSensor(port='/dev/ttyUSB4', baudrate=115200)
        self.yaw_started = self.yaw_sensor.start()
        if not self.yaw_started:
            print("[WARNING] Yaw sensor start failed. Will use zero data.")

    def __del__(self):
        # Close all log files
        for i in range(18):
            try:
                self.log_files[i].close()
            except:
                pass
        print(f'[Logging] All log files saved to {self.log_dir}')

    def _build_policy_joint_gains(self):
        """Build kp/kd arrays aligned with policy joint indices.

        User-defined groups are specified in motor_id space, but the runtime
        buffers are indexed by policy_idx (0..17). We map via
        config.joint2motor_idx.
        """

        group1 = {13, 7, 1, 16, 10, 4}
        group2 = {18, 9, 3, 15, 11, 6}
        group3 = {17, 8, 2, 14, 12, 5}

        kp = np.zeros(18, dtype=np.float32)
        kd = np.zeros(18, dtype=np.float32)

        unknown_motor_ids = []
        for policy_idx in range(18):
            motor_id = int(self.config.joint2motor_idx[policy_idx])
            if motor_id in group1:
                kp[policy_idx] = 55.0
                kd[policy_idx] = 0.45
            elif motor_id in group2:
                kp[policy_idx] = 75.0
                kd[policy_idx] = 0.55
            elif motor_id in group3:
                kp[policy_idx] = 70.0
                kd[policy_idx] = 0.55
            else:
                unknown_motor_ids.append(motor_id)

        if unknown_motor_ids:
            # 不阻塞运行，但提示配置/分组可能不完整
            uniq = sorted(set(unknown_motor_ids))
            print(f"[WARNING] Some motor_ids are not in any gain group: {uniq}")

        return kp, kd

    def _warm_up(self):
        obs = torch.ones((1, int(self.config.num_obs)))
        for _ in range(10):
            _ = self.policy(obs)
        print('Network has been warmed up.')


    # 零力矩模式
    def zero_torque_state(self):
        print("Enter zero torque state.")
        print("Waiting for the Y signal to default pos...")
        while self.gamepad.get_button_y() != 1:
            create_zero_cmd(self.robot)
            time.sleep(self.config.control_dt)

    # 移动到默认位置
    def move_to_default_pos(self):
        print('Moving to default pos.')
        total_time = 2
        num_step = int(total_time / self.config.control_dt)
        default_pos = self.config.default_angles


        init_dof_pos = np.zeros(18,dtype=np.float32)
        # 读取当前 DOF 位置作为起点
        # NOTE: RL_Real_PySOEM buffers are indexed by policy_idx (0..17).
        # SDK internally maps policy_idx -> motor_id and applies direction/offset.
        for i in range(18):
            init_dof_pos[i] = self.robot.motor_state_buffer.position[i]

        for i in range(num_step):
            alpha = i / num_step
            
            for j in range(18):
                target_pos = default_pos[j]
                q = init_dof_pos[j] * (1 - alpha) + target_pos * alpha
                self.robot.motor_command_buffer.kp[j] = 150.0
                self.robot.motor_command_buffer.kd[j] = 11.0
                self.robot.motor_command_buffer.target_position[j] = q
                self.robot.motor_command_buffer.target_velocity[j] = 0.0
                self.robot.motor_command_buffer.feedforward_torque[j] = 0.0

            time.sleep(self.config.control_dt)

    # 在默认位置等待，直到按下按钮A
    def default_pos_state(self):
        print("Enter default pos state.")
        print("Waiting for the Button A signal...")

        #init_dof_pos = np.zeros(18, dtype=np.float32)
        while self.gamepad.get_button_a() != 1:
            for i in range(18):
                default_pos = self.config.default_angles[i]
                self.robot.motor_command_buffer.kp[i] = 150.0
                self.robot.motor_command_buffer.kd[i] = 11.0
                self.robot.motor_command_buffer.target_position[i] = default_pos
                self.robot.motor_command_buffer.target_velocity[i] = 0.0
                self.robot.motor_command_buffer.feedforward_torque[i] = 0.0
                # init_dof_pos[i] = self.robot.motor_state_buffer.position[i]
                # print(init_dof_pos[i])
            time.sleep(self.config.control_dt)

    # 打印关节初始位置
    def print_initial_joint_positions(self):
        print("Initial joint positions:")
        for i in range(18):
            motor_id = self.config.joint2motor_idx[i]
            pos = self.robot.motor_state_buffer.position[i]
            print(f"policy_idx {i:02d} (motor_id {motor_id:02d}), Position: {pos:.3f}")

    # 打印imu数据
    def print_imu_data(self):
        vel = self.imu.get_linear_velocity()
        grav = self.imu.get_gravity_acceleration()
        if vel is not None and grav is not None:
            for i in range(10):
                print(f"Vel: [{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}] | Grav: [{grav[0]:6.3f}, {grav[1]:6.3f}, {grav[2]:6.3f}]")
                time.sleep(0.1)
        else:
            print("No IMU data available.")

    def print_pitch_angle(self):
        pitch_value = self.pitch_sensor.get_angle()
        if pitch_value is not None:
            print(f"Pitch Angle: {pitch_value:.3f} degrees")
        else:
            print("No pitch angle data available.")

    def run(self):
        self.counter += 1
        for i in range(18):
            self.qj[i] = self.robot.motor_state_buffer.position[i]
            self.dqj[i] = self.robot.motor_state_buffer.velocity[i]

        vel = self.imu.get_linear_velocity()
        imu_data = self.imu.get_imu_data()
        grav = self.imu.get_gravity_acceleration()

        tension_value = self.tension_sensor.get_cable_tension()
        if tension_value is None:
            tension_value = 0.0
            
        pitch_value = self.pitch_sensor.get_angle()
        if pitch_value is None:
            pitch_value = 0.0
            
        yaw_value = self.yaw_sensor.get_angle()
        if yaw_value is None:
            yaw_value = 0.0

        if imu_data is None:
            # 如果没有 IMU 数据，使用全 0
            linvel = np.zeros(3, dtype=np.float32)
            ang_vel = np.zeros(3, dtype=np.float32)
            gravity_orientation = np.array([0.0, 0.0, -1.0], dtype=np.float32)  # 假设重力向下
        else:
            linvel = np.asarray(vel, dtype=np.float32)
            ang_vel = np.asarray(
                [imu_data['gyro_x'], imu_data['gyro_y'], imu_data['gyro_z']], dtype=np.float32
            )
            gravity_orientation = np.asarray(grav, dtype=np.float32)

            # Match sim2sim convention: gravity is a unit vector in body frame.
            g_norm = float(np.linalg.norm(gravity_orientation))
            if g_norm > 1e-6:
                gravity_orientation = gravity_orientation / g_norm


        cmd = self.gamepad.get_command()
        
        self.cmd[0] = np.float32(cmd[0])
        self.cmd[1] = np.float32(cmd[1])
        self.cmd[2] = np.float32(cmd[2])

        qj_obs = self.qj.copy()
        qj_obs = qj_obs - self.config.default_angles
        dqj_obs = self.dqj.copy()
        dqj_obs = dqj_obs 

        self.obs[:3] = linvel * self.lin_vel_scale
        self.obs[3:6] = ang_vel * self.ang_vel_scale
        self.obs[6:9] = gravity_orientation
        self.obs[9:27] = qj_obs * self.dof_pos_scale
        self.obs[27:45] = dqj_obs * self.dof_vel_scale
        self.obs[45:64] = self.action

        # Command is stored in obs with scaling (match sim2sim layout).
        # Avoid printing at control rate (50Hz) since it can disturb timing.
        self.obs[64:67] = self.cmd * self.config.command_scale
        
        self.obs[67] = np.float32(tension_value)
        self.obs[68] = np.float32(yaw_value)
        self.obs[69] = np.float32(pitch_value)

        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
        self.action = self.policy(obs_tensor).detach().numpy().squeeze()

        # Action clipping: joint actions limited to [-2, 2], cable action limited to [0, 1]
        self.action[0:18] = np.clip(self.action[0:18], -2.0, 2.0)
        self.action[18] = np.clip(self.action[18], 0.0, 1.0)

        target_dof_pos = self.config.default_angles + self.action[0:18] * self.config.action_scale
        # target_dof_pos = self.config.default_angles

        for i in range(18):
            q = target_dof_pos[i]
            motor_id = int(self.config.joint2motor_idx[i])
            
            # Read actual motor position
            actual_pos = self.robot.motor_state_buffer.position[i]
            error = actual_pos - q

            # Add feedforward torque for Group2 joints with correct sign
            # Sign depends on motor_direction to ensure torque assists motion
            group2_motor_ids = {18, 9, 3, 15, 11, 6}
            if motor_id in group2_motor_ids:
                # Get motor direction (+1 or -1)
                motor_dir = self.config.motor_directions[i]
                # Apply feedforward in the direction that assists gravity compensation
                # If motor_dir is -1, we need to flip the sign
                feedforward_torque = 1.5 * motor_dir
            else:
                feedforward_torque = 0.0

            # Log to CSV
            self.log_writers[i].writerow([
                self.counter * self.config.control_dt,  # time_step
                self.counter,                            # counter
                f'{q:.6f}',                              # target_pos
                f'{actual_pos:.6f}',                     # actual_pos
                f'{error:.6f}',                          # error
                f'{self.action[i]:.6f}'                  # action
            ])

            self.robot.motor_command_buffer.kp[i] = float(self._kp_policy[i])
            self.robot.motor_command_buffer.kd[i] = float(self._kd_policy[i])
            self.robot.motor_command_buffer.target_position[i] = q
            self.robot.motor_command_buffer.target_velocity[i] = 0.0
            self.robot.motor_command_buffer.feedforward_torque[i] = feedforward_torque


            # motor_id 仅用于对照打印
            # motor_id = self.config.joint2motor_idx[i]
            # print(f"policy_idx: [{i}] | motor_id: [{motor_id}] | q: [{q}]")
            #print(f"Vel: [{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}] | Grav: [{grav[0]:6.3f}, {grav[1]:6.3f}, {grav[2]:6.3f}]")
            

        time.sleep(self.config.control_dt)


if __name__ == "__main__":
    # import argparse

    # parser = argparse.ArgumentParser()
    # parser.add_argument("net", type=str, help="network interface")
    # args = parser.parse_args()

    config_path = f"{config_hexapod_tethered.ROOT_DIR}/deploy/deploy_real/configs/hexapod.yaml"
    config = Config(config_path)

    # ChannelFactoryInitialize(0, args.net)

    controller = Controller(config)

    time.sleep(1) # 等待系统稳定
    controller.print_initial_joint_positions()
    controller.print_imu_data()
    controller.print_pitch_angle()

    controller.zero_torque_state()
    controller.move_to_default_pos()
    controller.default_pos_state()
    print("Start main control loop. Press LB to exit.")

    while True:
        try:
            
            controller.run()
            if controller.gamepad.get_button_lb() == 1:
                break

            if controller.gamepad.get_button_y() == 1:
                controller.move_to_default_pos()
                # 进入默认位置保持状态，等待再次按下A键恢复策略，期间保持静止不受力掉落
                controller.default_pos_state()
            
        except KeyboardInterrupt:
            break


    if getattr(controller, "gamepad", None) is not None:
        try:
            controller.gamepad.stop()
        except Exception:
            pass

    print("Entering damping mode to lower the robot safely. Press Ctrl+C to exit completely.")
    # Keep sending damping command continuously to keep the robot in damping mode
    try:
        while True:
            create_damping_cmd(controller.robot)
            time.sleep(config.control_dt)
    except KeyboardInterrupt:
        pass
        
    print('Exit')