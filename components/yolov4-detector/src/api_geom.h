//
// Created by alumno on 19/1/22.
//

#ifndef YOLOV4_DETECTOR_API_GEOM_H
#define YOLOV4_DETECTOR_API_GEOM_H

#include <cmath>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include  "../../../etc/graph_names.h"
#include <vector>

class api_geom
{

    public:
        static float distance_between_objects(std::shared_ptr<DSR::DSRGraph> G, std::unique_ptr<DSR::RT_API> rt_api, std::optional<DSR::Node> node1, std::optional<DSR::Node> node2);
        
};


#endif //YOLOV4_DETECTOR_API_GEOM_H
