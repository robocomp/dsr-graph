# CORTEX infrastructure for agents and the mighty working memory named DSR (aka G)

This software is the new infrastructure that will help in the creation of CORTEX instances. A CORTEX instance is a set of software components, called _agents_, that share a distributed data structured called (_G_)raph playing the role of a working memory. Agents are C++ programs that can be generated using RoboComp's code generator, _robocompdsl_. Development of a new CRDT-based shared graph to be used as a highly efficient working memory.
Agents are created with an instance of the FastRTPS middleware (by eProsima) configured in reliable multicast mode. A standard graph data structure is defined using FastRTPS's IDL and converted into a C++ class that is part of the agent's core. This class is extended using the _delta mutators_ CRDT code provided by Carlos Baquero. With this added functionality, all the copies of G held by the participating agents acquire the property of _eventual consistency_. This property entails that all copies, being different at any given moment, will converge to the exact same state in a finite amount of time, after all agents stop editing the graph.

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

## Basic use-case

We are working on an initial use-case that involves our e-aparment ALab, our robot Viriato and a few agents and components.

Definitions:

- CORTEX, a Robotics Cognitive Architecture based on the idea of a set of sorftware modules sharing a common representation or working memory. 
- G, a distributed graph used as the CORTEX working memory. It exists as the set of local copies held by all the participating components in a CORTEX configuration
- DSR Agent (DA), is a C++ RoboComp component extended with the functionality to edit the distributed graph G.
- Node,
- Edge,
- Atrribute,

