if [ ! -e /boot/config.txt.set-oculus.bk ]; then
	sudo cp /boot/config.txt /boot/config.txt.set-oculus.bk
	sudo cp /boot/config.txt.oculus /boot/config.txt
	
	sudo cp /etc/rc.local /etc/rc.local.set-oculus.bk
	sudo cp /etc/rc.local.oculus /etc/rc.local
	
	sudo reboot
fi