if [ ! -e /boot/config.txt.set-oculus.bk ]; then
	sudo cp /boot/config.txt /boot/config.txt.set-oculus.bk
	sudo cp /boot/config.txt.oculus /boot/config.txt
	sudo reboot
fi