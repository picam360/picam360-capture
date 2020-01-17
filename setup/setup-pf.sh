
CURRENT=$(cd $(dirname $0) && pwd)
cd $CURRENT

port=$1
echo port is $port
sed -e "s/%PORT%/$port/g" ssh_config > ~/.ssh/config
sudo cp ssh-pf.service /etc/systemd/system/ssh-pf.service
sudo cp www-pf.service /etc/systemd/system/www-pf.service
sudo cp nodedebug-pf.service /etc/systemd/system/nodedebug-pf.service
sudo cp pythondebug-pf.service /etc/systemd/system/pythondebug-pf.service
