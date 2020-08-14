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
#include <QHBoxLayout>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include "/home/pbustos/software/darknet/include/darknet.h"	
#include <fps/fps.h>

extern "C" 
{
	int size_network(network *net);
	void remember_network(network *net);
	void avg_predictions(network *net);
	char** get_labels(char *);
	void cuda_set_device(int);
}
using namespace std::chrono_literals;

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

	private:
		
		// DSR graph
		std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<DSR::InnerAPI> innermodel;

        //DSR params
        std::string agent_name;
        int agent_id;
        bool read_dsr;
        std::string dsr_input_file;
        bool tree_view;
        bool graph_view;
        bool qscene_2d_view;
        bool osg_3d_view;

        // DSR graph viewer
        std::unique_ptr<DSR::DSRViewer> dsr_viewer;
        QHBoxLayout mainLayout;
        QWidget window;
        bool startup_check_flag;

		// YOLO
		struct Box
		{
			string name;
			int left;
			int top;
			int right;
			int bot;
			float prob;
		};
        std::vector<Box> process_image_with_yolo(const cv::Mat& img);
        yolo::image createImage(const cv::Mat &src);
		yolo::image createImage(const std::vector<uint8_t> &src, int width, int height, int depth);
		network* init_detector(std::string path_to_yolodata_);
		std::vector<Box> detectLabels(yolo::network *ynet, const yolo::image &img, float thresh, float hier_thresh);
		char** names;
		clock_t ytime1;
		std::vector<yolo::network*> ynets;
		const std::size_t YOLO_INSTANCES = 1;
		bool SHOW_IMAGE = false;
		bool READY_TO_GO = false;
		std::string path_to_yolodata;
		FPSCounter fps;

        void show_image(cv::Mat &imgdst, const vector<SpecificWorker::Box> &boxes, const std::vector<SpecificWorker::Box> synth_boxes);
        cv::Mat get_rgb_image(const Node &rgb_node);
        vector<Box> process_graph_with_yolosynth(const std::vector<std::string> &object_names, const Node& rgb_cam);
         void compute_prediction_error(const vector<SpecificWorker::Box> &real_boxes, const vector<Box> synth_boxes);
};

#endif
