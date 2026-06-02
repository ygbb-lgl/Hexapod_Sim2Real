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