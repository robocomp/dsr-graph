#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <DataStorm/DataStorm.h>
#include <chrono>
#include <thread>
#include "../../graph-related-classes/libs/delta-crdts.cc"
#include "../../graph-related-classes/CRDT.h"
#include "DSRGraph.h"

using N = RoboCompDSR::Node; // For each node


#define LAPS 30
#define NODES 500

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

	std::string agent_name, filter;
	bool work;
	int graph_root;
	DataStorm::Node node;
    std::shared_ptr<CRDT::CRDTNodes> graph;
	std::shared_ptr<DataStorm::SingleKeyWriter<std::string, RoboCompDSR::AworSet >> writer;
	std::shared_ptr<DataStorm::Topic<std::string, RoboCompDSR::AworSet >> topic;

	void subscribeThread();
	void serveFullGraphThread();
	void newGraphRequestAndWait();

	std::thread read_thread, full_graph;

};

#endif
