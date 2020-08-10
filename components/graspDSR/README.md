# graspDSR

This component is responsible for planning targets for the robot arm, enabling it to grasp a specific object in the scene. It estimnates poses using `objectPoseEstimation` component, which uses DNN-based methods to estimate poses from RGBD data.

## Configuration parameters

As any other component, *graspDSR* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
etc/config
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <graspDSR's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/graspDSR config
```
