sudo rm /etc/network/interfaces.d/wlan0-dhcp-server
sudo systemctl disable hostapd
sudo systemctl disable isc-dhcp-server
sudo sed -e 's/^denyinterfaces wlan0/#denyinterfaces wlan0/g' /etc/dhcpcd.conf > dhcpcd.conf.tmp
sudo mv dhcpcd.conf.tmp /etc/dhcpcd.conf