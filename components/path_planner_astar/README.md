# social_navigation
Intro to component here

# Installation notes

-   Before compiling the agent, make sure to replace `TriangleFunctor` in `/usr/include/osg` to avoid conflicts between _FCL_ and _OSG_ :
    ```bash
    sudo cp TriangleFunctor /usr/include/osg
    ```

-   Build and compile the component :
    ```bash
    cmake .
    make
    ```

## Configuration parameters
As any other component, *social_navigation* needs a configuration file to start. In
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
cd <social_navigation's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/social_navigation config
```
