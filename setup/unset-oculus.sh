if [ -e /boot/config.txt.set-oculus.bk ]; then
	sudo cp /boot/config.txt.set-oculus.bk /boot/config.txt
	sudo rm /boot/config.txt.set-oculus.bk
	
	sudo cp /etc/rc.local.set-oculus.bk /etc/rc.local
	sudo rm /etc/rc.local.set-oculus.bk
	
	sudo reboot
fi