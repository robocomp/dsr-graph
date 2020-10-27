# DSR Components
- [Description](#description)
- [Definitions](#definitions)
- [Installation](#installation)
- [Basic use case](#basic-use-case)
  * [Running](#running)
  * [Create a new agent to interact with the running CORTEX instance.](#create-a-new-agent-to-interact-with-the-running-cortex-instance)
- [Related papers](#related-papers)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


Note: This documentation is related to the DSR/CORTEX **components**. You can find the DSR/CORTEX related **library** in robocomp repository (![classes/dsr](https://github.com/robocomp/robocomp/tree/development/classes/dsr)) with some developer documentation to use the APIs and UIs.

# Description
CORTEX is a long term effort to build a series of architectural designs around the simple idea 
of a group of agents that share a distributed, dynamic representation acting as a working memory.
This data structure is called **Deep State Representation (DSR)** due to the hybrid nature 
of the managed elements, geometric and symbolic, and concrete (laser data) and abstract (logical predicates).
A CORTEX instance is a set of software components, called _agents_, that share a distributed data structured called (_G_)raph playing the role of a working memory. Agents are C++ programs that can be generated using RoboComp's code generator, _robocompdsl_. 

<img src="https://user-images.githubusercontent.com/5784096/90373871-e3257d80-e072-11ea-9933-0392ea9ae7f1.png" width="800">

<sup>*The illustration shows a possible instance of the CORTEX architecture. The central part of the ring contains the **DSR graph** that is shared by all agents, from whom a reference implementation is presented here. Coloured boxes represent agents providing different functionalities to the whole. The purple box is an agent that can connect to the real robot or to a realistic simulation of it, providing the basic infrastructure to explore prediction and anticipation capabilities*</sup>

# Definitions
- CORTEX, a Robotics Cognitive Architecture based on the idea of a set of software modules sharing a common representation or working memory. 
- G, a distributed graph used as the CORTEX working memory. It exists as the set of local copies held by all the participating components in a CORTEX configuration
- DSR Agent (DA), is a C++ RoboComp component extended with the functionality to edit the distributed graph G.
- Node, a vertex of the graph G.
- Edge, a directional edge connecting to nodes. Edges can have attributes.
- All nodes and edges have type. The list of possible types is shown below.
- Attribute, a property of a node or an edge. Attributes can be of any of these types
  1. string
  2. int (signed 32)
  3. float
  4. vector<float>
  5. bool
  6. in V1.1 vector<byte>
  All attributes have a name and in V1.1 will hava a timestamp with the last modification time
  
- Attributes' names must be taken from a predefined list of tokens with types, for example ('name', string)('pos_x', float). A complete lists is provided below.
- _RT edges_ are edges of type _RT_ that code a geometric relation between two nodes. RT edges have two atributes _rotation_euler_xyz_ and _translation_ that hold    the euler angles and translation coordinates of the R|t 4x4 matrix that represents a rotation followed by a translation pose of the child with respect to the parent's reference frame. There are methods in the G-API to directly recover the Rt matrix from the edge as a RoboComp RTMat.
- There is a singular node in G named, "world", that representes the origin of the current reference frame for the robot. This node is the root of a kinematic tree       linked by RT edges. This tree is embedded in G and represent the set of physical elements believed to exist in the world. This tree is called _innermodel_ and can be automatically drawn in 3D using OpenSceneGraph and in 2D using Qt. Both representation are included in the UI off all generated agents.
- An agent generated with _robocompdsl_ includes the object _G_ that can be accessed using its public API

# Installation
Check the [classes/dsr](https://github.com/robocomp/robocomp/tree/development/classes/dsr) documentation for specific instructions on the installation of the needed infraestructure. 

To install these components in this repositor it's assumed that you have already installed ![robocomp](https://github.com/robocomp/robocomp/blob/development/README.md#installation-from-source).
You must clone this (dsr-graph) repository in ~/robocomp/components/
```sh
cd ~/robocomp/components/
git clone https://github.com/robocomp/dsr-graph/
cd dsr-graph/components/
```

Note: To be able to build social_navigation component you need to install
  ```sh
  sudo apt install libfclib-dev
  ```
  And rebuild robocomp core with fcl support:
  ```sh
  cd ~/robocomp/build
  cmake -D FCL_SUPPORT=1 ..
  make
  sudo make install
  ```
  
  Note: In Ubuntu 20.04 you need to replace the file in 
   ```sh
  sudo cp TriangleFunctor /usr/include/osg
  ```
  
  Note: To compile some agents you need g++-10.2 and change the directive
    set(CMAKE_CXX_STANDARD 17) to set(CMAKE_CXX_STANDARD 20)
  but then you'll get a compilation error in one of ZeroC's Ice include files. Please, replace it with:
  ```sh
  sudo cp Connection.h /usr/include/Ice/Connection.h
  ```

# Basic use case

We are working on an initial use-case that involves RoboLab's adapted aparment ALab, the mobile manipulator Viriato and a few agents and components.

  
  ## Running
  To start the basic configuration for this use case:
  
  0. Execute rcnode in a free terminal to initiate ZeroC Ice pub/sub broker.
  1. Execute ViriatoPyrep (Python3) You should have installed first CoppeliaRobotics simulator. Then follow the instructions in ViriatoPyrep's README.md
  2. Compile and run idserver
  3. Compile and run viriatoDSR
  
  If you have a JoyStick, you can use the robocomp-robolab under ~/robocomp/components and go to components/hardware/external_control/joystickPub. Compile the component and run it. You'll see the robot moving in the simulator. You can change the name and ranges of axes in the configuration file.

## Create a new agent to interact with the running CORTEX instance.
TODO

# Related papers
P. Bustos García, L. Manso Argüelles, A. J. Bandera, J. P. Bandera, I. García-Varea, and J. Martínez-Gómez, «The CORTEX cognitive robotics architecture: Use cases,» Cognitive Systems Research, vol. 55, pp. 107-123, 2019. 
https://robolab.unex.es/wp-content/plugins/papercite/pdf/luis-pablo-cortex.pdf

P. Núñez Trujillo, L. J. Manso Argüelles, P. Bustos García, P. Drews-Jr, and D. G. Macharet, «A Proposal for the Design of a Semantic Social Path Planner using CORTEX ˜,» in Workshop of Physical Agents 2016, Málaga, Spain, 2016. 
https://robolab.unex.es/wp-content/plugins/papercite/pdf/proposal-design-semantic.pdf

L. V. Calderita Estévez and P. Bustos García, «Deep State Representation: an Unified Internal Representation for the Robotics Cognitive Architecture CORTEX,» PhD Thesis, ., 2015. 
https://robolab.unex.es/wp-content/plugins/papercite/pdf/deep-state-representation.pdf

P. Bustos García, L. Manso Argüelles, A. J. Bandera, J. P. Bandera, I. García-Varea, and J. Martínez-Gómez, «The CORTEX cognitive robotics architecture: Use cases,» Cognitive Systems Research, vol. 55, pp. 107-123, 2019. 
https://robolab.unex.es/wp-content/plugins/papercite/pdf/luis-pablo-cortex.pdf

P. Bustos García, L. J. Manso Argüelles, A. Bandera, J. P. Bandera, I. García-Varea, and J. Martínez-Gómez, «CORTEX: a new Cognitive Architecture for Social Robots,» in EUCognition Meeting – Cognitive Robot Architectures, Viena, 2016. 
https://robolab.unex.es/wp-content/plugins/papercite/pdf/cortex-new-cognitive-architecture.pdf

Paulo Sérgio Almeida, Ali Shoker, Carlos Baquero, «Delta State Replicated Data Types» Journal of Parallel and Distributed Computing, Volume 111, January 2018, Pages 162-173 
https://arxiv.org/abs/1603.01529
