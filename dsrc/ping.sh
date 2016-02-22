# sudo ifconfig usb0 down
# sudo ifconfig usb0 10.1.1.2 up

# ssh 10.1.1.3

addr="169.254.$1"
echo $addr
ping $addr
