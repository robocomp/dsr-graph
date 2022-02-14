#include <cmath>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include  "../../../etc/graph_names.h"

class api_geom
{
    private:

    
    public:

        api_geom();
        static float distance_between_objects(std::shared_ptr<DSR::DSRGraph> G, std::optional<DSR::Edge> edge1, std::optional<DSR::Edge> edge2);

};