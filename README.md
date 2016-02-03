# dsrc-radar-collision-avoidance

## Installing Kvaser LeafLight v2 Drivers

Drivers for the Kvaser LeafLight v2 USB device can be found here:

https://www.kvaser.com/products/kvaser-leaf-light-hs-v2/#/!

For this project we will be using the linux version of CAN lib and the Python Library Extensions. The Linux Drivers can be fund by going to the link provided above, clicking on Downloads, and clicking on "Kvaser LINUX Driver and SDK". Instructions and examples for how to use the Python. Our project has been compiled on Ubuntu 14.04 LTS

Once the drivers have been downloaded on on a linux distribution, run the following commands in order to install the derivers on your machine.

`tar xzfv linuxcan.tar.gz`
`mv linuxcan ~`
`cd ~/linuxcan`
`make`

After you have executed these commands, the Kvaser LeafLight v2 drivers should be properly installed on your machine.
