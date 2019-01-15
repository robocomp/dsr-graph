#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <DataStorm/DataStorm.h>
#include <chrono>
#include <thread>
#include "../../graph-related-classes/libs/delta-crdts.cc"
#include "DSRGraph.h"

using N = RoboCompDSR::Content; // For each node
using G = RoboCompDSR::DSRGraph; // For full graph

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
	DataStorm::Node node;
	std::shared_ptr<DataStorm::SingleKeyWriter<std::string, N>> writer;
	std::shared_ptr<DataStorm::Topic<std::string, N>> topic;

	void subscribeThread();
	void serveFullGraphThread();
	void newGraphRequestAndWait();

	std::thread read_thread, full_graph;

    ormap<string,aworset<N>> nodes; // Real data

};

#endif
