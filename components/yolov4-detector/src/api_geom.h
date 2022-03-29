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
namespace DSR
{
    class Api_Geom
    {
    private:
        // DSR Graph
        std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::shared_ptr<DSR::RT_API> rt_api;
        

    public:
        Api_Geom(std::shared_ptr<DSR::DSRGraph> G);
        // Return the distance between the two nodes positions
        std::optional<float> distance_between_objects(const DSR::Node &node1, const DSR::Node &node2);
        // Return the closest node to node of the specific type
        std::optional<DSR::Node> get_closest_objects_by_type(const DSR::Node &node, const std::string &type);
        // Return distance between from litlle object to big object if the first is contained in and above the second one, otherwise return void optional
        //world reference system
        std::optional<float> distance_to_object_below(const DSR::Node &little_object, const DSR::Node &big_object);
        // Return distance from translation point of the object (middle point of the boundin box) world reference system
        std::optional<float> height_difference(const DSR::Node &node1, const DSR::Node &node2);
        //IN PROGRESS
        bool strictly_over_object(DSR::Node &little_object, DSR::Node &big_object);
        // Check which container contains the objet and update it, change name
        bool insert_node_in_container(DSR::Node &node);
        // Update node and its RT with the new parent
        bool update_node(DSR::Node &node, DSR::Node &new_parent);
        //Set the attribute average size of the object
        void set_average_size(DSR::Node &node);
    };
}