import numpy as np
import time
from ray import get
import torch

# from unitree_sdk2py.core.channel import ChannelPublisher,ChannelSubscriber,ChannelFactoryInitialize
# from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_,unitree_go_msg_dds__LowState_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo

#from unitree_sdk2py.utils.crc import CRC

#from common.command_helper import create_zero_cmd,create_damping_cmd
#from common.rotation_helper import get_gravity_orientation

from imu_sdk.imu_sdk import IMUSDK

from hexapod_tethered_utils.leg_joints_motions.run_can_rv import RVMotorDriver

from common.command_helper_hexapod import create_zero_cmd,create_damping_cmd

# from common.remote_controller import RemoteController, KeyMap

from hexapod_tethered_utils.cable_tension_sensor import get_cable_tension
from hexapod_tethered_utils.cable_end_pitch_sensor import get_cable_end_pitch_angle

from hexapod_tethered_utils.joystick_reader import Gamepad

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

        self._tension_failed = False
        self._tension_value = 0.0

        self._pitch_failed = False
        self._pitch_value = 0.0

        self.policy = torch.jit.load(config.policy_path)
        # 预热网络，减少第一次推理的延迟
        self._warm_up()
        
        # 初始化状态和命令
        self.qj = np.zeros(config.num_actions,dtype=np.float32)
        self.dqj = np.zeros(config.num_actions,dtype=np.float32)
        self.action = np.zeros(config.num_actions,dtype=np.float32)
        self.target_dof_pos = config.default_angles.copy()
        self.obs = np.zeros(config.num_obs,dtype=np.float32)
        self.cmd = np.array([0, 0, 0],dtype=np.float32)
        self.counter = 0

        # self.low_state = unitree_go_msg_dds__LowState_()
        # self.lowcmd_publisher = ChannelPublisher(config.lowcmd_topic,LowCmdGo)
        # self.lowcmd_publisher.Init()
        #self.lowstate_subscriber = ChannelSubscriber(config.lowstate_topic,LowStateGo)
        #self.lowstate_subscriber.Init(self.LowStateHandler,10)
        # self.replay_buffer = ReplayBuffer(max_replay_buffer_size=200,flag='real_new')

        self.driver = RVMotorDriver.pcan(channel="PCAN_USBBUS1", bitrate=1_000_000)

        self.imu = IMUSDK(port='/dev/ttyUSB0', baudrate=921600)

        self.tension_value = get_cable_tension(port='/dev/ttyUSB1', baudrate=115200)
        
        # 设置接受类型
        for i in range(18):
            self.driver.set_comm_ack(motor_id=i+1)
            time.sleep(0.01)  # 避免过快发送导致问题


    @property
    def tension_value(self) -> float:
        """最新缆绳张力（自动刷新；失败则保持最后值并禁用刷新）。"""
        if not self._tension_failed:
            try:
                tension = get_cable_tension()
                if tension is not None:
                    self._tension_value = float(tension)
            except Exception as e:
                self._tension_failed = True
                print(f"[cable_tension] disabled due to error: {e}")
        return self._tension_value

    @property
    def pitch_value(self) -> float:
        """最新末端绝对角度（自动刷新；失败则保持最后值并禁用刷新）。"""
        if not self._pitch_failed:
            try:
                angle = get_cable_end_pitch_angle()
                if angle is not None:
                    self._pitch_value = float(angle)
            except Exception as e:
                self._pitch_failed = True
                print(f"[cable_end_pitch] disabled due to error: {e}")
        return self._pitch_value

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
            create_zero_cmd(self.driver)
            time.sleep(self.config.control_dt)

    # 移动到默认位置
    def move_to_default_pos(self):
        print('Moving to default pos.')
        total_time = 2
        num_step = int(total_time / self.config.control_dt)

        dof_idx = self.config.joint2motor_idx
        default_pos = self.config.default_angles


        init_dof_pos = np.zeros(18,dtype=np.float32)
        # 读取当前 DOF 位置作为起点
        for i in range(18):
            init_dof_pos[i] = self.driver.get_angle(motor_id=i+1)

        for i in range(num_step):
            alpha = i / num_step
            
            for j in range(18):
                motor_idx = dof_idx[j]
                target_pos = default_pos[j]
                q = init_dof_pos[j] * (1 - alpha) + target_pos * alpha
                self.driver.send_ctrl(motor_id=motor_idx, kp=50.0, kd=0.8, pos=q, spd=0.0, tor=0.0)

            time.sleep(self.config.control_dt)

    # 在默认位置等待，直到按下按钮A
    def default_pos_state(self):
        print("Enter default pos state.")
        print("Waiting for the Button A signal...")

        while self.gamepad.get_button_a() != 1:
            for i in range(18):
                motor_idx = self.config.joint2motor_idx[i]
                default_pos = self.config.default_angles[i]
                self.driver.send_ctrl(motor_id=motor_idx, kp=50.0, kd=0.8, pos=default_pos, spd=0.0, tor=0.0)
            time.sleep(self.config.control_dt)


    def run(self):
        self.counter += 1
        for i in range(18):
            self.qj[i] = self.driver.get_angle(motor_id=i+1)
            self.dqj[i] = self.driver.get_speed(motor_id=i+1)
        # shiwu imu jia - hao
        vel = self.imu.get_linear_velocity()
        imu_data = self.imu.get_imu_data()
        grav = self.imu.get_gravity_acceleration()

        linvel = np.array(vel)
        ang_vel = np.array([imu_data['gyro_x'], imu_data['gyro_y'], imu_data['gyro_z']])
        gravity_orientation =np.array(grav)

        cmd = self.gamepad.get_command()
        
        self.cmd[0] = np.float32(cmd[0])
        self.cmd[1] = np.float32(cmd[1])
        self.cmd[2] = np.float32(cmd[2])

        qj_obs = self.qj.copy()
        qj_obs = qj_obs - self.config.default_angles
        dqj_obs = self.dqj.copy()
        dqj_obs = dqj_obs 

        self.obs[:3] = linvel
        self.obs[3:6] = ang_vel
        self.obs[6:9] = gravity_orientation
        self.obs[9:27] = qj_obs
        self.obs[27:45] = dqj_obs
        self.obs[45:64] = self.action
        self.obs[64:67] = self.cmd * self.config.command_scale

        # 张力state
        self.obs[67] = np.float32(self.tension_value)

        # yaw state 目前随便写的
        self.obs[68] = np.float32(self.roll_value)

        # 末端绝对角度
        self.obs[69] = np.float32(self.pitch_value)


        obs_tensor = torch.from_numpy(self.obs).unsqueeze(0)
        self.action = self.policy(obs_tensor).detach().numpy().squeeze()

        target_dof_pos = self.config.default_angles + self.action[0:18] * self.config.action_scale
        # target_dof_pos = self.config.default_angles

        for i in range(18):
            motor_idx = self.config.joint2motor_idx[i]
            q = target_dof_pos[i]
            self.driver.send_ctrl(motor_id=motor_idx, kp=50.0, kd=0.8, pos=q, spd=0.0, tor=0.0)

        cable_action = self.action[18]
        cable_ctrl = np.clip(cable_action, 0, 1)
        cable_tension = 400.0 * cable_ctrl
        time.sleep(self.config.control_dt)


if __name__ == "__main__":
    # import argparse

    # parser = argparse.ArgumentParser()
    # parser.add_argument("net", type=str, help="network interface")
    # args = parser.parse_args()

    config_path = f"{config_hexapod_tethered.ROOT_DIR}/deploy/deploy_real/configs/hexapod_tethered.yaml"
    config = Config(config_path)

    # ChannelFactoryInitialize(0, args.net)

    controller = Controller(config)

    controller.zero_torque_state()
    controller.move_to_default_pos()
    controller.default_pos_state()

    while True:
        try:
            controller.run()
            if controller.gamepad.get_button_lb() == 1:
                break
        except KeyboardInterrupt:
            break


    if getattr(controller, "gamepad", None) is not None:
        try:
            controller.gamepad.stop()
        except Exception:
            pass

    create_damping_cmd(controller.driver)
    print('Exit')