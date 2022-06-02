#include <cmath>
#include <QVector3D>
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
        std::optional<float> distance_between_objects(DSR::Node &node1, DSR::Node &node2);
        //It returns the closest one but it might me to far of the node1, if we create another method returning with an unmbral we need optional node (idk how to do that)
        std::optional<DSR::Node> get_closest_objects_by_type(const DSR::Node &node1, const std::string &type);
        //HAS_VALUES ELSES
        std::optional<float> distance_of_over_object(DSR::Node &little_object, DSR::Node &big_object);
        bool strictly_over_object(DSR::Node &little_object, DSR::Node &big_object);
};