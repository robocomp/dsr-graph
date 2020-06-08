

# IDServer

## List of node types, edge type and attribute names
### Node types
* differentialrobot
* robot
* omnirobot
* laser
* battery
* camera
* slam-device
* imu
* 

* room
* pose

* plane
* box
* cylinder
* ball
* mesh

* person
* face
* body
* chest
* nose
* left_eye
* right_eye
* left_ear
* right_ear
* left_arm
* right_arm
* left_shoulder
* right_shoulder
* left_elbow
* right_elbow
* left_wrist
* right_wrist
* left_hip
* right_hip
* left_leg
* right_leg
* left_knee
* right_knee
* left_ankle
* right_ankle
* left_hand
* righ_hand
* left_foot
* right_foot

* mug
* noodles
* table
* chair
* shelve
* dish
* fork
* spoon

### Edge types
* RT
* reachable
* in
* knows
* transitable
* graspable
* talking
* looking-at
* sitting
* standing
* close-to

### Attribute types
* level
* pos_x
* pos_y
* parent
* parent
* rotation_euler_xyz
* translation
* depth
* height
* width
* texture
* mass

## Configuration parameters
As any other component,
``` *idserver* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE


## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <idserver 's path> ```

    cp etc/config config

After editing the new config file we can run the component:

    bin/

```idserver ```

    --Ice.Config=config
