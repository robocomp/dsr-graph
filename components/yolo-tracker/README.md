```
```
#
``` yoloserver
```


You need cuda. The version Ubuntu repo works fine:

    sudo apt-get install libopencv-dev nvidia-cuda-dev nvidia-cuda-toolkit

Download darknet from the Yolo(V3) repository

Edit the Makefile to set OPenCV=0

Compile and build the library:

make

Copy the library and .h files to /usr/local

    sudo cp libyolo.so /usr/local/lib

Copy yolo weights and data to the yolodata directory in the yoloserver compnent.

wget https://pjreddie.com/media/files/yolo.weights -O wherever-you-have-your-component

cp -r src/yololib/data/ wherever-you-have-your-component






## Configuration parameters
As any other component,
``` *yoloserver* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <yoloserver 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```yoloserver ```

    --Ice.Config=config
