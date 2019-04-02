
CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

cp ssh_config /home/pi/.ssh/config
sudo cp ssh-pf.service /etc/systemd/system/ssh-pf.service
sudo cp www-pf.service /etc/systemd/system/www-pf.service
sudo cp nodedebug-pf.service /etc/systemd/system/nodedebug-pf.service
sudo cp pythondebug-pf.service /etc/systemd/system/pythondebug-pf.service
