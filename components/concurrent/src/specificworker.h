#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "../../graph-related-classes/graphNEW.h"
//#include "../../graph-related-classes/graphviewer.h"

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
	std::shared_ptr<DSR::Graph> graph;

};

#endif
