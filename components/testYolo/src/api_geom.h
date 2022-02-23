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

    
    public:
        
        static float distance_between_objects(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, DSR::Node node1, DSR::Node node2);
        static float distance_object_parent(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, 
                                            std::unique_ptr<DSR::InnerEigenAPI> innerEigen_api, DSR::Node little_object, DSR::Node big_object);
};