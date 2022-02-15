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
        static float distance_between_objects(std::shared_ptr<DSR::DSRGraph> G, std::optional<DSR::Edge> edge1, std::optional<DSR::Edge> edge2);
};


#endif //YOLOV4_DETECTOR_API_GEOM_H
