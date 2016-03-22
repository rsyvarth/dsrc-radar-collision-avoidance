
self_ip="10.1.1.2"
ip="10.1.1.3"

echo "-----------------------------------"
echo "This script will connect to and setup the Cohda DSRC Module for use"
echo "-----------------------------------"
echo ""

echo -n "Are you connected via USB [y/n]: "
read -n 1 using_usb
echo ""

if [[ $using_usb == 'y' ]]; then
    echo "Using usb"
    sudo ifconfig usb0 down
    sudo ifconfig usb0 10.1.1.2 up
else
    echo "Using ethernet"
    ip_selected=false
    while [[ $ip_selected == false ]]; do
        echo "Available ethernet conections are:"
        echo "1) 169.254.113.221"
        echo "2) 169.254.115.37"
        echo "3) 169.254.115.157"

        echo -n "Which device are you using [1-3]: "
        read -n 1 ethernet_device
        echo ""

        ip_selected=true
        if [[ $ethernet_device == '1' ]]; then
            ip="169.254.113.221"
        elif [[ $ethernet_device == '2' ]]; then
            ip="169.254.115.37"
        elif [[ $ethernet_device == '3' ]]; then
            ip="169.254.115.157"
        else
            echo ""
            echo "Error: Invalid ip selected, please use of the available options"
            ip_selected=false
        fi
    done

    self_ip=$(ifconfig | grep 169.115 | awk '{print $2}' | sed -e s/addr://)
    echo "Found current machine's IP to be $self_ip"
fi

echo "-----------------------------------"
echo "Running: ping $ip"
echo "-----------------------------------"
ping -c 1 -W 1000 $ip > /dev/null
if [[ $? != "0" ]]; then
    echo "Error: Cannot connect to host, is everything connected?"
    exit 84115
fi
echo ""


echo "Checking for sshpass"
sshpass_prefix="sshpass -pduser"
sshpass -V > /dev/null
if [[ $? != "0" ]]; then
    sshpass_prefix=""
    echo "sshpass not installed, sshpass saves you from having to type in passwords"
    echo "Try installing via 'sudo apt-get install sshpass' for a better experience"
fi
echo ""

pass_prompt() {
    if [[ $sshpass_prefix == "" ]]; then
        echo " - Please enter the password 'duser' when prompted"
    fi
}

echo "-----------------------------------"
echo "Running: ssh duser@$ip 'sudo start can' to start the can reading service"
pass_prompt
echo "-----------------------------------"
$sshpass_prefix ssh duser@$ip "sudo start can"
echo ""

echo "-----------------------------------"
echo "Running: ssh duser@$ip 'watch -n0 cansend can0 7DF#02010D0000000000 &'"
echo " - This command polls the CAN bus 10 times/sec for vehicle's speed"
pass_prompt
echo "-----------------------------------"
# Kill running cansend script if one exists
$sshpass_prefix ssh duser@$ip "sudo pkill cansend"
# Start up a new cansend script
$sshpass_prefix ssh duser@$ip "watch -n0 cansend can0 7DF#02010D0000000000 &"
echo ""

echo "-----------------------------------"
echo "Running: ssh duser@$ip 'sudo D_LEVEL=5 ./bsm-shell -c 174 -a 176 -n 174 -x 0x00 --LogUdp --UdpAddr $self_ip --UdpTxPort 5005'"
echo " - This command starts sending/recieving DSRC messages and logging them to our local application"
pass_prompt
echo "-----------------------------------"
# Kill running bsm script if one exists
$sshpass_prefix ssh duser@$ip "sudo pkill bsm-shell"
# Start up a new bsm script
$sshpass_prefix ssh duser@$ip "sudo D_LEVEL=5 ./bsm-shell -c 174 -a 176 -n 174 -x 0x00 --LogUdp --UdpAddr $self_ip --UdpTxPort 5005 &"
echo ""

echo "-----------------------------------"
echo "Done! We should be setup and ready to start logging now!"
echo "-----------------------------------"
