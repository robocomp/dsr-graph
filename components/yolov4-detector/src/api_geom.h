#include <cmath>
#include <QVector3D>
#include <functional>
#include <QPoint>
#include <typeinfo>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include "../../../etc/graph_names.h"

/*
Comments:
    - X,Y,Z they must be always refered as width,depth,height respectively (Coppelia)
*/
class api_geom
{
private:
    // DSR Graph
    std::shared_ptr<DSR::DSRGraph> G;
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
    std::shared_ptr<DSR::RT_API> rt_api;

public:
    api_geom(std::shared_ptr<DSR::DSRGraph> G, std::shared_ptr<DSR::RT_API> rt_api, std::shared_ptr<DSR::InnerEigenAPI> inner_eigen);
    //Return the distance between the two nodes positions
    std::optional<float> distance_between_objects(const DSR::Node &node1,const DSR::Node &node2);
    //Return the closest node to node of the specific type 
    std::optional<DSR::Node> get_closest_objects_by_type(const DSR::Node &node, const std::string &type);
    //Return distance between from litlle object to big object if the first is contained in the second one, otherwise return void optional
    std::optional<float> distance_of_over_object(const DSR::Node &little_object,const DSR::Node &big_object);
    //Doenst work with cup on the floor,maybe using bottom points
    std::optional<float> height_difference(const DSR::Node &node1,const DSR::Node &node2);
    //Update node and its RT with the new parent
    bool update_node(DSR::Node &node,DSR::Node &new_parent);
    //TODO not functional
    bool strictly_over_object(DSR::Node &little_object, DSR::Node &big_object);
    //TODO check which container contains the objet nad update it 
};

