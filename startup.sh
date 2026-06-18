echo "start up"

sudo timeout --signal=SIGINT --kill-after=5s 15s /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/hexapod_tethered_utils/cable_tension_sensor_zero.py

echo "start deploy"

# sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod_tethered.py enp86s0
#sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod_tethered_plot.py enp86s0
sudo /home/hexapod/anaconda3/envs/hexapod_lgl/bin/python /home/hexapod/Hexapod_Sim2Real/deploy/deploy_real/deploy_real_hexapod_tethered_plot_torque.py enp86s0