
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	// DSR Graph creation
	graph = std::make_shared<DSR::Graph>();
	//graph->readFromFile("/home/robocomp/robocomp/files/innermodel/simpleworld-hybrid.xml");
	graph->readFromFile("caca.xml");
	//graph->print();

	this->Period = period;
	timer.setSingleShot(true);
	timer.start(Period);
}

void SpecificWorker::compute()
{
	//Let's start the party

	{
		{
			auto n = graph->getNodePtr(100);
			auto &node = *(n.get());
		// }  //if scope is removed the second new blocks here
		std::cout << "waiting" << std::endl;
		auto n2 = graph->getNodePtr(100);
		auto &node2 = *(n2.get());
		
		
		std::cout << "id " << node2->id << " type " << node2->type  << std::endl;
		//std::cout << "did " << node->id << " type " << node->type  << std::endl;
	}
	std::cout << "line" << std::endl;	
	auto nodeT = graph->nodes[100];
	std::cout << "Truth, should be: " << nodeT.id << " " << nodeT.type  << std::endl;
}



