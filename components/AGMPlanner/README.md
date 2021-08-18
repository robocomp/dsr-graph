# AGMPlanner
AGMPlanner is a component designed to work in conjunction with another component: AGMPlannerPython.
This is the C ++ side of the suite, which is responsible for connecting to the dsr-graph and regularly 
calling the interface implemented by AGMPlannerPython to run the AGM scheduler.

## Configuration parameters
As any other component, *AGMPlanner* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:

Note: Right now, the arguments are not implemented by config despite most code is implemented due to a lack of time in the end of
the project. For configure the component right now, you must enter the file routes in the specificworker.py. Check pending work section
```
#Rules file
#Test value = examples/logistics/domain.aggl
aggl_file = examples/logistics/domain.aggl
#Starting world file
#Test value = examples/logistics/init0.xml
init_file = examples/logistics/init0.xml
#Goal world file
#Test value = examples/logistics/prueba0.aggt
aggt_goal = examples/logistics/prueba0.aggt
#Result plan file 
#Test value = examples/logistics/resultado.plan
result_plan = examples/logistics/resultado.plan
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <AGMPlanner's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/AGMPlanner config
```

## Pending work

Right now the parameters in the config file are not working, and they are directly set in the specificworker.py.
Another pending work is change the c++ component so instead of execute the planner each X number of seconds, it get executed when the C++ 
component receive a signal from another component.


