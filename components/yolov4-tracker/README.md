# YOLOv4 Tracker

An agent that performs object tracking using YOLOv4 network. This is an upgrade of `yolo-tracker` agent.

## Installation

-   Navigate to `home` directory :
    ```bash
    cd ~
    ```
-   Clone `darknet` repository :
    ```bash
    git clone https://github.com/AlexeyAB/darknet.git
    ```
-   Edit `Makefile` tags :
    ```
    GPU=1
    OPENCV=1
    LIBSO=1
    ```
-   Compile Darknet library to get `.so` file :
    ```bash
    make
    ```
-   Navigate to `yolov4-tracker` directory.
-   Update the library path in `src/CMakeListsSpecific.txt` :
    ```bash
    sed -i s/REPLACE/<username>/g src/CMakeListsSpecific.txt
    ```

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
