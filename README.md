## 本项目是根据unitree go2机器人改的sim2real六足代码，目前只有hexapod是可以用的，hexapod_tethered还没有作出更改

### configs文件夹中存放了yaml文件，在config_xxx中进行读取，deploy_real_hexapod.py中是真正的sim2real主程序


### 执行sim2real的bash指令
```bash
sudo /home/lgl/anaconda3/envs/mujoco_rl/bin/python /home/lgl/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod.py enp109s0
```