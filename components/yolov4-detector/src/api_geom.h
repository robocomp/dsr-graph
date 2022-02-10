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
    //static std::vector<> get_large_size_objects();
    //static std::vector<> get_container_objects();
    public:
        static float distancia(std::shared_ptr<DSR::DSRGraph> G, DSR::Node n1, DSR::Node n2);
};


#endif //YOLOV4_DETECTOR_API_GEOM_H
