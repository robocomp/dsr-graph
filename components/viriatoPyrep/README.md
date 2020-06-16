# viriatoPyrep

You need to copy the viriato.py file in this directory to:

/home/xxxyour-userxxx/.local/lib/python3.6/site-packages/pyrep/robots/mobiles/viriato.py
(for a local install of Pyrep in python3.6)



## Configuration parameters
As any other component, *viriatoPyrep* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <viriatoPyrep's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/viriatoPyrep config
```
