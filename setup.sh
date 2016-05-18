SUDO=''
if (( $EUID != 0 )); then
    SUDO='sudo'
fi

BIN_DIR="/usr/local/bin"
CONFIG_DIR="/etc/"

#$SUDO apt-get update || exit 0
#$SUDO apt-get install python python-serial python-smbus python-pygame
$SUDO cp main.py $BIN_DIR/mp3_distance
$SUDO cp mp3_distance.json $CONFIG_DIR
$SUDO cp mp3_distance.service /etc/systemd/system/
$SUDO systemctl daemon-reload
$SUDO systemctl enable mp3_distance.service
$SUDO systemctl start mp3_distance.service

