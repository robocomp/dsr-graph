/*
 *    Copyright (C) 2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include  "../../../etc/viriato_graph_names.h"
#include <doublebuffer/DoubleBuffer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/GridMapMath.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void update_node_slot(const std::uint64_t id, const std::string &type);

private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;
	std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;

	//DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	bool startup_check_flag;

    //Signal subscription
    using LaserData = std::tuple<std::vector<float>, std::vector<float>>;  //<angles, dists>
    DoubleBuffer<LaserData, std::vector<Mat::Vector2d>> laser_buffer;

    //Grid
    grid_map::GridMap map;
    cv::Mat originalImage;

     /*!
      * Creates a cv mat from a grid map layer.
      * This conversion sets the corresponding black and white pixel value to the
      * min. and max. data of the layer data.
      * @param[in] grid map to be added.
      * @param[in] layer the layer that is converted to the image.
      * @param[in] encoding the desired encoding of the image.
      * @param[in] lowerValue the value of the layer corresponding to black image pixels.
      * @param[in] upperValue the value of the layer corresponding to white image pixels.
      * @param[out] image the image to be populated.
      * @return true if successful, false otherwise.
     */
    template<typename Type_, int NChannels_>
    bool toImage(const grid_map::GridMap& gridMap, const std::string& layer, const int encoding,
                const float lowerValue, const float upperValue, cv::Mat& image)
    {
        const float minValue = gridMap.get(layer).minCoeff();
        const float maxValue = gridMap.get(layer).maxCoeff();

        // Initialize image.
        if (gridMap.getSize()(0) > 0 && gridMap.getSize()(1) > 0)
        {
            image = cv::Mat::zeros(gridMap.getSize()(0), gridMap.getSize()(1), encoding);
        }
        else
        {
            std::cerr << "Invalid grid map?" << std::endl;
            return false;
        }

        // Get max image value.
        Type_ imageMax;
        if (std::is_same<Type_, float>::value || std::is_same<Type_, double>::value)
        {
            imageMax = 1.0;
        }
        else if (std::is_same<Type_, unsigned short>::value || std::is_same<Type_, unsigned char>::value)
        {
            imageMax = (Type_)std::numeric_limits<Type_>::max();
        }
        else
        {
            std::cerr << "This image type is not supported." << std::endl;
            return false;
        }

        // Clamp outliers.
        grid_map::GridMap map = gridMap;
        //map.get(layer) = map.get(layer).unaryExpr(grid_map::Clamp<float>(lowerValue, upperValue));

        const grid_map::Matrix& data = map[layer];

        // Convert to image.
        bool isColor = false;
        if (image.channels() >= 3) isColor = true;
        bool hasAlpha = false;
        if (image.channels() >= 4) hasAlpha = true;

        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index index(*iterator);
            const float& value = data(index(0), index(1));
            if (std::isfinite(value)) {
                const Type_ imageValue = (Type_)(((value - lowerValue) / (upperValue - lowerValue)) * (float)imageMax);
                const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
                unsigned int channel = 0;
                image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[channel] = imageValue;

                if (isColor) {
                    image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
                    image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
                }
                if (hasAlpha) {
                    image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageMax;
                }
            }
        }
        return true;
    };

};

#endif
