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

## General Setup Instructions for Logging

I wrote this up in an email and thought it would be helpful to have it reproduced
here since it details the setup of our system pretty well.

```
> 1. Can this project log radar + DSRC messages (both Tx and Rx)?

Yes the software can do all of that - although much of the content of the DSRC BSMs is currently empty due to issues with non-standardized representations of that data on vehicles. We currently only populate GPS related fields + speed in the packets. We have done some research on collecting the rest of the information but it is vehicle specific and we don't want to write code that will only work for the 2010 Prius that we are using. Just something to be aware of.

> 2. What's the best way to use the code to generate the combined logs?

If you just run `python main.py` with all of the hardware setup properly it will automatically generate logs into the logs/dsrc.log and logs/radar.log directories. It is worth noting that we are pretty much constantly changing this codebase right now so we are likely to break things pretty frequently. We have at least one known issue with the way we decode track width from the radar that still needs to be resolved. There are likely more fields which we are incorrectly decoding as well - we still have much of our testing left to do.

The process for getting the DSRC "setup properly" is slightly involved - check the dsrc_connect.sh script to see the steps you have to run. That script might work, but some of the error handling is poor so it will fail silently so I would recommend doing it by hand. Our radar interface will also only work with the completely black radars (not the one you have been using).

> 3. How do you set up the system with a computer?

I think I explained most of the software side of this at part of #2. The only other major thing you will need to do software-wise is ensure you have all of the Kvaser drivers setup. There are instructions for how to do this on linux in the repository - otherwise you can download/install the windows version from Kvaser's website (which I presume you have already done). You do not need to worry about installing OpenCV in order to collect logs - that is just for the visualization stuff we are doing and I believe we have it properly setup at this point that you can still run everything (other than the visualization) fine without OpenCV installed.

As far as the hardware goes you will need to connect the OBD-II connector to the OBD-II port on your car; connect the Kvaser to one of the DB9 connectors and plug that into your computer; connect the DSRC to the other DB9, its power cable, antenna (3 wires), and ethernet to your laptop (there is also a USB interface but it isn't as stable we have found); and connect the radar to its proprietary connector.
```
