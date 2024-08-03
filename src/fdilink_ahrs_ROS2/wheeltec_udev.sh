#CP2102 串口号0003 设置别名为fdilink_ahrs
# echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/fdi_imu.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout"' >/etc/udev/rules.d/ch340.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout"' >/etc/udev/rules.d/cp2102.rules

service udev reload
sleep 2
service udev restart


