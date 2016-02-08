
sudo ifconfig usb0 down
sudo ifconfig usb0 10.1.1.2 up

rsync -r --progress . 10.1.1.3:~/server
