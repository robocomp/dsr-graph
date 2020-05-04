# dsr-graph
Development of new DSR shared graph

To install this component, 

* Clone https://github.com/ryanhaining/cppitertools in /usr/local/include
* Install version 9 of g++ (https://askubuntu.com/questions/1140183/install-gcc-9-on-ubuntu-18-04/1149383#1149383)
* Install the middleware Fast-RTPS de eProsima manually

    *  https://github.com/eProsima/Fast-RTPS#manual-installation
    
    You will need three libs: Fast CDR, Foonathan memory and Fast RTPS in this order.
    
```
Compile and install:
```bash
mkdir Fast-CDR/build && cd Fast-CDR/build
cmake ..
cmake --build . --target install
```

*  Install Foonathan Memory:
    * https://github.com/foonathan/memory
    * https://github.com/eProsima/Fast-RTPS/issues/620#issuecomment-525274544
```
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
cmake --build . --target install
  ```

* Install dependencies:
```
sudo apt install libasio-dev/bionic libtinyxml2-dev/bionic
```

*  Install Fast rtps 
```bash
git clone https://github.com/eProsima/Fast-RTPS.git

mkdir Fast-RTPS/build && cd Fast-RTPS/build

cmake ..

cmake --build . --target install
```
