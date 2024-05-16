# depthDSR
The agent is responsible for estimating depths in real-time from RGBD data in Graph(G). Depth is estimated using `depthEstimation`, which is a DNN component.
## Configuration Parameters
depthDSR needs a configuration file to start. In etc/config, you can change the ports and other parameters in the configuration file, according to your setting.<br/>

## Starting the Component
To run graspDSR component, navigate to the component directory :<br/>

`cd <depthDSR's path>`<br/>

Compile the component :<br/>

`cmake .` <br/>
`make`<br/>

Now run the component :<br/>

`./bin/depthDSR etc/config`



