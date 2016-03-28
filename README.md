# dsrc-radar-collision-avoidance

## Installing Kvaser LeafLight v2 Drivers

Drivers for the Kvaser LeafLight v2 USB device can be found here:

https://www.kvaser.com/products/kvaser-leaf-light-hs-v2/#/!

For this project we will be using the linux version of CAN lib and the Python
Library Extensions. The Linux Drivers can be fund by going to the link provided
above, clicking on Downloads, and clicking on "Kvaser LINUX Driver and SDK".
Version 5.13 has already be installed and cloned into this repository

Once the drivers have been downloaded on on a linux distribution, run the
following commands in order to install the derivers on your machine.

`make` `sudo make install`

After you have executed these commands, the Kvaser LeafLight v2 drivers should
be properly installed on your machine.

## Installing OpenCV

You will need virtualenv to install all of the dependencies for this project.
Run `which virtualenv` to make sure that you have the command installed and if
you don't then run `sudo pip install virtualenv`

To install openCV for this project, all you have to do is go to the opencv
directory and run the following command.

`./install.sh`

### Known issues with the Install script

There is a common bug found with the installation script. If you open up a
python shell and run the following command.

``` python
import cv2
```

And you get an error about how the cv2 module is not found, then perform the
following actions:

1. Check the `src/venv/lib/python2.7/site-packages` directory to make sure that
   the cv.so binary is installed. If it is not there then copy it from the
   location mentioned in step 2.
2. If the binary is installed on your virtual environment, then go to
   `/usr/local/lib/python2.7/site-packages` to make sure that the cv2.so binary
   is also located there.
3. If the binary is not there, then something was wrong with the installation
   and you need to rerun `./install.sh` from the opencv directory
4. Copy the cv2.so binary into the `/usr/local/lib/python2.7/dist-packages`
   directory.
5. Reopen up your virtualenv again and run

``` python
import cv2
```
6. At this point you should not receive any errors.
