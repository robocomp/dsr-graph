
#include "specificworker.h"
#include <random>
#include <chrono> 


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

	//this->Period = period;
	//timer.setSingleShot(true);
	//timer.start(Period);
	compute();
}

void SpecificWorker::compute()
{
	//Let's start the party
	// TEST 1
	{
		std::cout << "---------------- TEST 1 ----------------" << std::endl;
		{
			auto n = graph->getNodePtr(100);
			auto &node = *(n.get());
		}  
		//if scope is removed the second new blocks here
		
		std::cout << "waiting" << std::endl;
		auto n2 = graph->getNodePtr(100);
		auto &node2 = *(n2.get());
		
		
		std::cout << "id " << node2->id << " type " << node2->type  << std::endl;
		//std::cout << "did " << node->id << " type " << node->type  << std::endl;
	
		std::cout << "line" << std::endl;	
		auto nodeT = graph->nodes[100];
		std::cout << "Truth, should be: " << nodeT.id << " " << nodeT.type  << std::endl;
	}

	// TEST 2
	{
		std::cout << "---------------- TEST 2 ----------------" << std::endl;
		auto n = graph->getNode(100);
		std::cout << "before id " << n.id << " type " << n.type  << std::endl;
		n.type = "morningsinger";
		graph->replaceNode(100, n);
		auto n2 = graph->getNode(100);
		std::cout << "after change, id " << n2.id << " type " << n2.type  << std::endl;
	}

	// TEST 3
	{
		std::cout << "---------------- TEST 3 ----------------" << std::endl;
		std::random_device random;  //Will be used to obtain a seed for the random number engine
    	std::mt19937 gen(random()); //Standard mersenne_twister_engine seeded with rd()
    	std::uniform_int_distribution<> dist(100, 139);
		auto start = std::chrono::high_resolution_clock::now();
		for(auto i: iter::range(100000))
		{
			auto ind = dist(gen);
			auto n = graph->getNode(ind);
			//std::cout << "before id " << n.id << " type " << n.type  << std::endl;
			n.type = "morningsinger";
			graph->replaceNode(ind, n);
			//auto n2 = graph->getNode(ind);
			//std::cout << "after change, id " << n2.id << " type " << n2.type  << std::endl;
		}
		auto finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = finish - start;
		std::cout << "Elapsed time: " << elapsed.count() << " s\n";
	}

	// TEST 4
	{
		std::cout << "---------------- TEST 4 ----------------" << std::endl;
		std::random_device random;  //Will be used to obtain a seed for the random number engine
    	std::mt19937 gen(random()); //Standard mersenne_twister_engine seeded with rd()
    	std::uniform_int_distribution<> dist(100, 139);
		auto start = std::chrono::high_resolution_clock::now();
		for(auto i: iter::range(100000))
		{
			auto ind = dist(gen);
			auto n2 = graph->getNodePtr(ind);
			auto &node2 = *(n2.get());
			node2->type = "morningsinger";
		}
		auto finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = finish - start;
		std::cout << "Elapsed time: " << elapsed.count() << " s\n";
	}
}



