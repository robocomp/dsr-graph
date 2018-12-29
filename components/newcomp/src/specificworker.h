#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <DataStorm/DataStorm.h>
#include "src/DSRGraph.h"
#include <chrono>
#include <thread>

using G = RoboCompDSR::DSRGraph;

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
	std::shared_ptr<DataStorm::SingleKeyWriter<std::string, G>> writer;
	std::shared_ptr<DataStorm::Topic<std::string, G>> topic;
	void subscribeThread();
	std::thread read_thread;

	DataStorm::SingleKeyWriter<std::string, G>
	makeGraphWriter(DataStorm::Topic<std::string, G>& topic, std::string sender, G graph)
	{
		auto writer = DataStorm::makeSingleKeyWriter(topic, move(sender));
    	writer.add(move(graph));
		return writer;
	}

	G my_graph;
};

#endif
