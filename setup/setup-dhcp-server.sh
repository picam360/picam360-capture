sudo cp wlan0-dhcp-server /etc/network/interfaces.d/
sudo systemctl enable hostapd
sudo systemctl enable isc-dhcp-server
sudo sed -e 's/^#denyinterfaces wlan0/denyinterfaces wlan0/g' /etc/dhcpcd.conf > dhcpcd.conf.tmp
sudo mv dhcpcd.conf.tmp /etc/dhcpcd.conf