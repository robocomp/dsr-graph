# dsr-graph
Development of new DSR shared graph

To install this component, 

* Clone https://github.com/ryanhaining/cppitertools in /usr/local/include
* Install version 9 of g++ (https://askubuntu.com/questions/1140183/install-gcc-9-on-ubuntu-18-04/1149383#1149383)
* Install the middleware Fast-RTPS de eProsima manually

    *  https://github.com/eProsima/Fast-RTPS#manual-installation
    
    You will need three libs: Fast CDR, Foonathan memory and Fast RTPS in this order.

* Install dependencies:
```
sudo apt install libasio-dev/bionic libtinyxml2-dev/bionic
```
