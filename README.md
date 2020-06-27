# CORTEX infrastructure for agents and the mighty working memory named DSR (aka G)

This software is the new infrastructure that will help in the creation of CORTEX instances. A CORTEX instance is a set of software components, called _agents_, that share a distributed data structured called (_G_)raph playing the role of a working memory. Agents are C++ programs that can be generated using RoboComp's code generator, _robocompdsl_.  Agents are created with an instance of the FastRTPS middleware (by eProsima) configured in reliable multicast mode. A standard graph data structure is defined using FastRTPS's IDL and converted into a C++ class that is part of the agent's core. This class is extended using the _delta mutators_ CRDT code kindly provided by Carlos Baquero. With this added functionality, all the copies of G held by the participating agents acquire the property of _eventual consistency_. This property entails that all copies, being different at any given moment, will converge to the exact same state in a finite amount of time, after all agents stop editing the graph. With _delta mutators_, modifications made to G by an agent are propagated to all others and integrated in their respective copies. This occurs even if the network cannot guarantee that each packet is delivered exactly once, _causal broadcast_. Note that if operations on the graph were transmitted instead of changes in state, then _exactly once_ semantics would be needed.  Finally, the resulting local graph _G_ is used by the developer through a carefully designed thread safe API, _G_API_.

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

The Figure below shows an artist view of a CORTEX instantiation.

![alt text](https://github.com/robocomp/dsr-graph/blob/development/dsr-graph.png "CORTEX")

## Basic use-case

We are working on an initial use-case that involves RoboLab's adapted aparment ALab, the mobile manipulator Viriato and a few agents and components.

Definitions:

- CORTEX, a Robotics Cognitive Architecture based on the idea of a set of sorftware modules sharing a common representation or working memory. 
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
- There is a singular node in G named, "world", that representes the origin o the current reference frame for the robot. This node is the root of a kinematic tree       linked by RT edges. This tree is embedded in G and represent the set of physical elements believed to exist in the world. This tree is called _innermodel_ and can be automatically drawn in 3D using OpenSceneGraph and in 2D using Qt. Both representation are included in the UI off all generated agents.
- An agent generated with _robocompdsl_ includes the object _G_ that can be accessed using its public API
  
  
  
