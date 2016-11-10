if [ ! -e /boot/config.txt.set-oculus.bk ]; then
	sudo cp /boot/config.txt /boot/config.txt.set-oculus.bk
	sudo cp /boot/config.txt.oculus /boot/config.txt
	
	sudo cp /etc/rc.local /etc/rc.local.set-oculus.bk
	sudo cp /etc/rc.local.oculus /etc/rc.local
	
	sudo reboot
else
	sudo sh /home/pi/picam360/picam360-capture/lunch.sh -w 1440 -h 1440 -W 512 -H 512 -r -n 2 -s &
fi