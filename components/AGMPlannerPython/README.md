# AGMPlannerPython

AGMPlannerPython is a component designed in conjunction with AGMPlanner. This component is responsible for executing the AGM scheduler (made in python).Currently it does so by opening a new terminal, but the best thing to do would be to import the python file and use the functions for efficiency reasons. 


## Configuration parameters
As any other component, *AGMPlannerPython* needs a configuration file to start. In
```
etc/config
```
This component does only have one config param. You can change the port if needed:
AGGLPlanner.Endpoints=tcp -p 37000

This file only implements an interface that is call in the AGMPlanner component (the C++ side). For more information check the code in specificworker.py

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <AGMPlannerPython's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
python3 src/AGMPlannerPython.py etc/config
```
