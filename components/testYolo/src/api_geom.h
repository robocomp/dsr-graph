#include <cmath>
#include <QVector>
#include <functional>
#include <QPoint>
#include <typeinfo>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include  "../../../etc/graph_names.h"

/*
Comments:
    - X,Y,Z they must be always refered as width,depth,height respectively (Coppelia)
*/
class api_geom
{
    private:
        //DSR Graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::shared_ptr<DSR::RT_API> rt_api;
    
    public:
        
        api_geom(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, std::shared_ptr<DSR::InnerEigenAPI> inner_eigen);
        float distance_between_objects(DSR::Node node1, DSR::Node node2);
        float distance_object_parent(DSR::Node little_object, DSR::Node big_object);
};