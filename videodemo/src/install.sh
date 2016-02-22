#!/usr/bin/env bash

# Script's directory
_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
_dest="$(readlink -m $_dir/..)"

# Script to install openCV with Python 2.7+
echo "*** updating your packages! ***"
sudo apt-get -y update
sudo apt-get -y upgrade

echo "*** installing developer tools ***"
sudo apt-get -y install build-essential cmake git pkg-config

echo "*** installing image libraries ***"
sudo apt-get -y install libjpeg8-dev libtiff4-dev libjasper-dev libpng12-dev

echo "*** installing basic openCV GUI"
sudo apt-get -y install libgtk2.0-dev

echo "*** installing video processing libraries ***"
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev

if [ ! $(which pip) ]; then
    echo "*** installing pip ***"
    wget https://bootstrap.pypa.io/get-pip.py
    sudo python get-pip.py
fi

echo "*** optimizing library calls ***"
sudo apt-get install libatlas-base-dev gfortran

cd $_dest
if [ ! $(which virtualenv) ]; then
    echo "*** setting up a virtual environment ***"
    pip install virtualenv
fi

echo "*** installing python2.7 dev tools ***"
sudo apt-get install python2.7-dev

echo "*** creating an opencv virtual environment ***"
virtualenv -p /usr/bin/python2.7 cv
source $_dest/cv/bin/activate

echo "*** installing python NumPy lib ***"
pip install numpy

deactivate

if [ ! -d opencv ]; then
    echo "*** installing opencv from git ***"
    cd $_dest
    git clone https://github.com/Itseez/opencv.git
    cd $_dest/opencv
    git checkout 3.1.0
    cd $_dest
else
    echo "*** opencv already cloned ***"
fi

if [ ! -d opencv_contrib ]; then
    echo "*** installing opencv_contrib from git ***"
    cd $_dest
    echo $_dest
    git clone https://github.com/Itseez/opencv_contrib.git
    cd $_dest/opencv_contrib
    git checkout 3.1.0
else
    echo "*** opencv_contrib already cloned ***"
fi


echo "*** building opencv using cmake ***"
cd $_dest/opencv
if [ ! -e $_dest/opencv/build ]; then
    mkdir $_dest/opencv/build
fi
cd $_dest/opencv/build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_C_EXAMPLES=OFF \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D BUILD_EXAMPLES=ON ..

echo "*** compiling opencv (assumes 4 cores) ***"
make -j4
sudo make install
sudo ldconfig

if [ ! -e $_dest/cv/lib/python2.7/site-packages ]; then
    echo "*** symlinking opencv to your virtualenv ***"
    ln -s /usr/local/lib/python2.7/dist-packages/cv2.so $_dest/cv/lib/python2.7/site-packages/
fi

exit 0
