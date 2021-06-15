# DSR Components
- [Description](#description)
- [Definitions](#definitions)
- [Installation](#installation)
- [Basic use case](#basic-use-case)
  * [Running](#running)
  * [Create a new agent to interact with the running CORTEX instance.](#create-a-new-agent-to-interact-with-the-running-cortex-instance)
- [Related papers](#related-papers)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


Note: This documentation is related to the DSR/CORTEX **components**. You can find the DSR/CORTEX related **library** in robocomp repository ([libs/dsr](https://github.com/robocomp/robocomp/tree/development/libs/dsr)) with some developer documentation to use the APIs and UIs.

# Description
CORTEX is a long term effort to build a series of architectural designs around the simple idea 
of a group of agents that share a distributed, dynamic representation acting as a working memory.
This data structure is called **Deep State Representation (DSR)** due to the hybrid nature 
of the managed elements, geometric and symbolic, and concrete (laser data) and abstract (logical predicates).
A CORTEX instance is a set of software components, called _agents_, that share a distributed data structured called (_G_)raph playing the role of a working memory. Agents are C++ programs that can be generated using RoboComp's code generator, _robocompdsl_. 

<img src="https://user-images.githubusercontent.com/5784096/90373871-e3257d80-e072-11ea-9933-0392ea9ae7f1.png" width="800">

<sup>*The illustration shows a possible instance of the CORTEX architecture. The central part of the ring contains the **DSR graph** that is shared by all agents, from whom a reference implementation is presented here. Coloured boxes represent agents providing different functionalities to the whole. The purple box is an agent that can connect to the real robot or to a realistic simulation of it, providing the basic infrastructure to explore prediction and anticipation capabilities*</sup>


# Installation
Check the [libs/dsr](https://github.com/robocomp/robocomp/tree/development/libs/dsr) documentation for specific instructions on the installation of the needed infrastructure. 

To install these components in this repositor it's assumed that you have already installed [robocomp](https://github.com/robocomp/robocomp/blob/development/README.md#installation-from-source).
You must clone this (dsr-graph) repository in ~/robocomp/components/
```sh
cd ~/robocomp/components/
git clone https://github.com/robocomp/dsr-graph/
cd dsr-graph/components/
```

Note: To be able to build social_navigation component you need to install
  ```sh
  sudo apt install libfcl-dev
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

Goto to this [tutorial](https://github.com/robocomp/robocomp/blob/development/doc/DSR-start.md) where you can follow several use cases of increasing complexity.

# Related papers
P. Bustos García, L. Manso Argüelles, A. J. Bandera, J. P. Bandera, I. García-Varea, and J. Martínez-Gómez, «The CORTEX cognitive robotics architecture: Use cases,» Cognitive Systems Research, vol. 55, pp. 107-123, 2019. 
https://robolab.unex.es/wp-content/papercite-data/pdf/luis-pablo-cortex.pdf

P. Núñez Trujillo, L. J. Manso Argüelles, P. Bustos García, P. Drews-Jr, and D. G. Macharet, «A Proposal for the Design of a Semantic Social Path Planner using CORTEX ˜,» in Workshop of Physical Agents 2016, Málaga, Spain, 2016. 
https://robolab.unex.es/wp-content/papercite-data/pdf/proposal-design-semantic.pdf

L. V. Calderita Estévez and P. Bustos García, «Deep State Representation: an Unified Internal Representation for the Robotics Cognitive Architecture CORTEX,» PhD Thesis, ., 2015. 
https://robolab.unex.es/wp-content/papercite-data/pdf/deep-state-representation.pdf

P. Bustos García, L. Manso Argüelles, A. J. Bandera, J. P. Bandera, I. García-Varea, and J. Martínez-Gómez, «The CORTEX cognitive robotics architecture: Use cases,» Cognitive Systems Research, vol. 55, pp. 107-123, 2019. 
https://robolab.unex.es/wp-content/papercite-data/pdf/luis-pablo-cortex.pdf

P. Bustos García, L. J. Manso Argüelles, A. Bandera, J. P. Bandera, I. García-Varea, and J. Martínez-Gómez, «CORTEX: a new Cognitive Architecture for Social Robots,» in EUCognition Meeting – Cognitive Robot Architectures, Viena, 2016. 
https://robolab.unex.es/wp-content/papercite-data/pdf/cortex-new-cognitive-architecture.pdf

Paulo Sérgio Almeida, Ali Shoker, Carlos Baquero, «Delta State Replicated Data Types» Journal of Parallel and Distributed Computing, Volume 111, January 2018, Pages 162-173 
https://arxiv.org/abs/1603.01529

<<<<<<< HEAD
=======
* Clone https://github.com/ryanhaining/cppitertools in /usr/local/include
* Install version 9 of g++ (https://askubuntu.com/questions/1140183/install-gcc-9-on-ubuntu-18-04/1149383#1149383)
* Install the middleware Fast-RTPS de eProsima manually

    *  https://github.com/eProsima/Fast-RTPS#manual-installation
    
    You will need three libs: Fast CDR, Foonathan memory and Fast RTPS in this order.

* Install dependencies:
```
sudo apt install libasio-dev libtinyxml2-dev libqglviewer-dev-qt5
```
>>>>>>> 9c91cabed26ae695de398067db2c697fcc79c1d6
