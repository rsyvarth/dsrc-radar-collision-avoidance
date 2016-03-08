#!/usr/bin/env bash

# Script's directory
_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
_dest="$(readlink -m $_dir/..)"

echo "*** Installing Dependencies ***"
sudo apt-get -y update &> /dev/null
sudo apt-get -y upgrade &> /dev/null
sudo apt-get -y install build-essential cmake git pkg-config libjpeg8-dev libtiff5-dev libjasper-dev \
    libpng12-dev libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libatlas-base-dev \
    gfortran &> /dev/null
sudo apt-get -y install python2.7-dev &> /dev/null


if [ ! $(which pip) ]; then
    echo "*** installing pip ***"
    wget https://bootstrap.pypa.io/get-pip.py &> /dev/null
    sudo python get-pip.py &> /dev/null
else
    echo "pip already installed"
fi

cd $_dest
if [ ! $(which virtualenv) ]; then
    echo "***"
    pip install virtualenv &> /dev/null
fi
echo "*** Done Installing Dependencies ***"

virtualenv cv
pip install numpy

if [ ! -d $_dest/opencv ]; then
    cd $_dest
    git clone https://github.com/Itseez/opencv.git
    cd $_dest/opencv
    git checkout 3.1.0
else
    echo "*** opencv already cloned ***"
fi

if [ ! -d $_dest/opencv_contrib ]; then
    cd $_dest
    git clone https://github.com/Itseez/opencv_contrib.git
    cd $_dest/opencv_contrib
    git checkout 3.1.0
else
    echo "*** opencv_contrib already installed ***"
fi

cd $_dest/opencv
if [ -d $_dest/opencv/build ]; then
    echo "*** Removing old build directory ***"
    rm -rf $_dest/opencv/build
fi
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=$_dest/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..

make -j4
sudo make install
sudo ldconfig

if [ ! -e $_dest/cv/lib/python2.7/site-packages/cv2.so ]; then
    echo "*** moving opencv to your virtualenv ***"
    sudo mv /usr/local/lib/python2.7/dist-packages/cv2.so $_dest/cv/lib/python2.7/site-packages/cv2.so
fi

exit 0
