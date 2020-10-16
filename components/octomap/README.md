# octomap
Intro to component here

## Installation

-   Install `octomap` libraries from the original repo :
    ```bash
    git clone https://github.com/OctoMap/octomap.git
    cd octomap
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

-   Copy `ViewerWidget.h` from `octomap/octovis/include/octovis` to `/usr/local/include/octovis/` :
    ```bash
    sudo cp ViewerWidget.h /usr/local/include/octovis/
    ```

-   Build and compile the component :
    ```bash
    cmake .
    make
    ```

## Configuration parameters
As any other component, *octomap* needs a configuration file to start. In
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
cd <octomap's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/octomap config
```
