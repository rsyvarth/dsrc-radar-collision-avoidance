# sudo ifconfig usb0 down
# sudo ifconfig usb0 10.1.1.2 up

# ssh 10.1.1.3 'bash -s' < run.sh

addr="169.254.$1"
echo $addr
ssh duser@$addr 'bash -s' < run.sh
