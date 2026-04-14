from motor_igh_sdk.deploy_real_el4090_pysoem import RL_Real_PySOEM

class MotorMode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints



# 进入阻尼模式，关电机力矩，增加阻尼
def create_damping_cmd(robot: RL_Real_PySOEM):
    size = 18
    for i in range(size):
        robot.motor_command_buffer.kp[i] = 0.0
        robot.motor_command_buffer.kd[i] = 11.0
        robot.motor_command_buffer.target_position[i] = 0.0
        robot.motor_command_buffer.target_velocity[i] = 0.0
        robot.motor_command_buffer.feedforward_torque[i] = 0.0



# 进入零力模式，关电机力矩，不增加阻尼
def create_zero_cmd(robot: RL_Real_PySOEM):
    size = 18
    for i in range(size):
        robot.motor_command_buffer.kp[i] = 0.0
        robot.motor_command_buffer.kd[i] = 0.0 
        robot.motor_command_buffer.target_position[i] = 0.0
        robot.motor_command_buffer.target_velocity[i] = 0.0
        robot.motor_command_buffer.feedforward_torque[i] = 0.0

