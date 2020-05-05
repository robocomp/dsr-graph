#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "../../../graph-related-classes/CRDT.h"
#include "../../../graph-related-classes/CRDT_graphviewer.h"
//#include "../../../graph-related-classes/libs/DSRGraph.h"
#include <random>

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(TuplePrx tprx);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		
		std::shared_ptr<CRDT::CRDTGraph> G;
		std::string agent_name;

	public slots:
		void compute();
		void initialize(int period);

	signals:
		void addNodeSIGNAL(std::int32_t id, const std::string &name, const std::string &type, float posx, float posy, const std::string &color);
		void addEdgeSIGNAL(std::int32_t from, std::int32_t to, const std::string &ege_tag);

	private:
		std::shared_ptr<InnerModel> innerModel;

		//params
		
		int agent_id;
		std::string dsr_input_file;
		std::string dsr_output_file;
		std::string test_output_file;
		bool read_file = false;
		
		std::unique_ptr<DSR::GraphViewer> graph_viewer;

		// Random
		std::random_device rd;
		std::mt19937 mt;
		std::uniform_real_distribution<float> unif_float;
        std::uniform_int_distribution<int> unif_int;

};

#endif
