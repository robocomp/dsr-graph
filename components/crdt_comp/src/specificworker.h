#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "../../graph-related-classes/CRDT.h"
#include "../../graph-related-classes/libs/DSRGraph.h"

#define LAPS 150
#define NODES 25
#define TIMEOUT 5

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	
public slots:
	void compute();
	void initialize(int period);

private:
	InnerModel *innerModel;
	std::string agent_name;
	std::shared_ptr<CRDT::CRDTGraph> graph;

};

#endif
