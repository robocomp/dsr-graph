#include <cmath>
#include <QVector3D>
#include <functional>
#include <typeinfo>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include  "../../../etc/graph_names.h"

class api_geom
{
    private:

    
    public:

        api_geom();
        //Which is better optional or a node and force the user to .value()
        static float distance_between_objects(std::shared_ptr<DSR::DSRGraph> G, std::unique_ptr<DSR::RT_API> rt_api, std::optional<DSR::Node> node1, std::optional<DSR::Node> node2);
        static float distance_object_parent(std::shared_ptr<DSR::DSRGraph> G, std::unique_ptr<DSR::RT_API> rt_api, std::optional<DSR::Node> node1, std::optional<DSR::Node> node2);
};