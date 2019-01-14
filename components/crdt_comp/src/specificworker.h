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
	string role;
	DataStorm::Node node;
	std::shared_ptr<DataStorm::SingleKeyWriter<std::string, N>> writer;
	std::shared_ptr<DataStorm::Topic<std::string, N>> topic;

	void subscribeThread();
	std::thread read_thread;

	DataStorm::SingleKeyWriter<std::string, N>
	makeGraphWriter(DataStorm::Topic<std::string, N>& topic, std::string sender, N node)
	{
		auto writer = DataStorm::makeSingleKeyWriter(topic, move(sender));
    	writer.add(move(node));
		return writer;
	}
    ormap<string,aworset<N>> nodes; // Real data
};

#endif
