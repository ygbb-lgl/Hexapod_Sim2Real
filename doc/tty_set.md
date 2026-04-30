六维力传感器 /dev/ttyUSB_cable_tension
pitch /dev/ttyUSB_pitch
yaw /dev/ttyUSB_yaw

# 1. FTDI 设备 (第一根)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="AH06DU0A", SYMLINK+="ttyUSB_cable_tension", MODE="0666"

# 2. Prolific PL2303 设备 A (第二根)
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23a3", ATTRS{serial}=="DOARb114J19", SYMLINK+="ttyUSB_pitch", MODE="0666"

# 3. Prolific PL2303 设备 B (第三根，这一根)
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23a3", ATTRS{serial}=="DBBYb114J19", SYMLINK+="ttyUSB_yaw", MODE="0666"
