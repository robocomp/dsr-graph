# YOLOv4 Tracker

An agent that performs object tracking using YOLOv4 network. This is an upgrade of `yolo-tracker` agent.

## Installation

-   Install CUDA >= 10.2 if you don't have it already. Follow instructions in https://developer.nvidia.com/cuda-toolkit-archive

-   Navigate to `home` directory :
    ```bash
    cd ~
    ```

-   Clone `darknet` repository :
    ```bash
    git clone https://github.com/AlexeyAB/darknet.git
    ```

-   Edit `Makefile` tags (set `CUDNN` to `1` for faster inference and `CUDNN_HALF` to `1` (for _RTX_ GPUS) for _3X_ speed) :
    ```
    GPU=1
    CUDNN=0
    CUDNN_HALF=0
    OPENCV=0
    AVX=0
    OPENMP=0
    LIBSO=1
    ZED_CAMERA=0
    ZED_CAMERA_v2_8=0
    USE_CPP=0
    DEBUG=0
    ```

-   Compile Darknet library to get `.so` file :
    ```bash
    make
    ```

-   Download [pretrained weights](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights) in `darknet` directory.

-   Navigate to `yolov4-tracker` directory.

-   Update the library path in `src/CMakeListsSpecific.txt` and `etc/config` by replacing the word `REPLACE` in both files by the username for correct YOLOv4 library path, using the following commands :
    ```bash
    sed -i s/REPLACE/<username>/g src/CMakeListsSpecific.txt
    sed -i s/REPLACE/<username>/g etc/config
    ```
    For example, if the username is `robocomp` :
    ```bash
    sed -i s/REPLACE/robocomp/g src/CMakeListsSpecific.txt
    sed -i s/REPLACE/robocomp/g etc/config
    ```
    So, the library path is `/home/robocomp/darknet/`.

## Configuration parameters

Like any other component, *yolov4-tracker* needs a configuration file to start. In `etc/config`, you can change the ports and other parameters in the configuration file, according to your setting.

## Starting the component

To run `yolov4-tracker` component, navigate to the component directory :
```bash
cd <yolov4-tracker's path> 
```

Then compile the component :
```bash
cmake .
make
```

Then run the component :
```bash
./bin/yolov4-tracker
```
