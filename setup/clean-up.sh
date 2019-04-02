cd /home/pi/picam360/picam360-capture
git remote set-url origin https://github.com/picam360/picam360-capture.git
git reset --hard & git pull
cmake .
make
sudo make install

cd /home/pi/picam360/picam360-server/armv6l
sudo rm -r userdata
mkdir userdata
git remote set-url origin https://github.com/picam360/picam360-server.git
git reset --hard & git pull
echo "make sure picam360-server/armv6l ready manually"

cd /home/pi/picam360/picam360-server/armv7l
sudo rm -r userdata
mkdir userdata
git remote set-url origin https://github.com/picam360/picam360-server.git
git reset --hard & git pull
echo "make sure picam360-server/armv7l ready manually"

cd /home/pi/picam360/picam360-server/armv6l/www
git remote set-url origin https://github.com/picam360/picam360-viewer.git
git reset --hard & git pull

cd /home/pi/picam360/picam360-server/armv7l/www
git remote set-url origin https://github.com/picam360/picam360-viewer.git
git reset --hard & git pull

echo "rm history files"
sudo rm /root/.*_history
sudo rm /home/pi/.*_history
sudo rm /home/pi/.ssh/known_hosts
sudo rm /home/pi/.gitconfig
sudo rm /home/pi/.xsession-errors*
sudo rm /home/pi/picam360/picam360-capture/.picam360_history
sudo rm -r /home/pi/.config/chromium
sudo rm -r /home/pi/.cache/chromium

echo "reset pf"
bash /home/pi/picam360/picam360-capture/setup/setup-pf.sh
sudo rm /home/pi/.ssh/*.key
sudo systemctl disable ssh-pf
sudo systemctl disable www-pf
sudo systemctl disable nodedebug-pf
sudo systemctl disable pythondebug-pf

#echo "reset rc.local"
#sudo cp /home/pi/picam360/picam360-capture/setup/rc.local /etc/rc.local

echo "reset config.json"
sudo cp /home/pi/picam360/picam360-capture/config.json.tmp /usr/local/etc/picam360-capture.conf
sudo cp /home/pi/picam360/picam360-server/armv6l/config.json.tmp /usr/local/etc/picam360-server.conf
sudo cp /home/pi/picam360/picam360-server/armv6l/www/config.json.tmp /var/www/picam360-server/armv6l/www/
sudo cp /home/pi/picam360/picam360-server/armv7l/www/config.json.tmp /var/www/picam360-server/armv7l/www/

echo "auto start up"
sudo systemctl enable picam360-capture.service
sudo systemctl enable picam360-server.service

#need to be last because wifi connection will be disable
echo "disable wpa? [y/N]"
read WPA
case $WPA in
	y)
		bash /home/pi/picam360/picam360-capture/setup/setup-dhcp-server.sh
		echo "switch to ap mode"
		sudo cp /etc/wpa_supplicant/wpa_supplicant.conf.init /etc/wpa_supplicant/wpa_supplicant.conf
		echo "wpa disabled."
		;;
	*)
		;;
esac

sync
