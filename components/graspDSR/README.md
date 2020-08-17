# graspDSR

This component is responsible for planning targets for the robot arm, enabling it to grasp a specific object in the scene. It estimates poses using `objectPoseEstimation` component, which uses DNN-based methods to estimate poses from RGBD data.

## Configuration parameters

Like any other component, *graspDSR* needs a configuration file to start. In `etc/config`, you can change the ports and other parameters in the configuration file, according to your setting.

## Starting the component

To run `graspDSR` component, navigate to the component directory :
```bash
cd <graspDSR's path> 
```

Then compile the component :
```bash
cmake .
make
```

Then run the component :
```bash
./bin/graspDSR
```
