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
#include "specificworker.h"
#include <cppitertools/zip.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    conf_params  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);
	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "", dsrgetid_proxy); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		    current_opts = current_opts | opts::tree;
		if(graph_view)
        {
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
			    current_opts = current_opts | opts::scene;
		if(osg_3d_view)
		    current_opts = current_opts | opts::osg;

		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::update_node_slot);

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att , cam_depth_att>();

        //Custom widget
        graph_viewer->add_custom_widget_to_dock("Octomap", &custom_widget);

        // Octomap
		octo = std::make_unique<octomap::OcTree>(1);
        auto octree_drawer = new octomap::OcTreeDrawer();
		custom_widget.addSceneObject(octree_drawer);

		// initialize from file or G
		//initialize_octomap(false, "octomap.map");

		this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    if( const auto laser_data = laser_buffer.try_get(); laser_data.has_value())
    {
        for(const auto &point : laser_data.value())
            octo->updateNode(point, true);
        std::cout << octo->size() << std::endl;
        // if changes, publish in G
    }
}

void SpecificWorker::initialize_octomap(bool read_from_file, const std::string file_name)
{
    qDebug() << __FUNCTION__ << "FileName:" << QString::fromStdString(file_name);
    Dimensions dim;
    QRectF outerRegion;
    auto world_node_o = G->get_node(world_name);
    if(not world_node_o.has_value())
        qFatal("Not World node found at initialize-octomap");
    auto world_node = world_node_o.value();
    auto l = G->get_attrib_by_name<OuterRegionLeft_att>(world_node);
    auto r = G->get_attrib_by_name<OuterRegionRight_att>(world_node);
    auto b = G->get_attrib_by_name<OuterRegionBottom_att>(world_node);
    auto t = G->get_attrib_by_name<OuterRegionTop_att>(world_node);
    if( not (l.has_value() and r.has_value() and b.has_value() and t.has_value()))
        qFatal("In initialize_octomap, Outer region of the scene not found in G. Aborting");
    outerRegion.setLeft(l.value());
    outerRegion.setRight(r.value());
    outerRegion.setBottom(b.value());
    outerRegion.setTop(t.value());

    // if read_from_file is true we should read the parameters from the file to guarantee consistency
    dim.HMIN = std::min(outerRegion.left(), outerRegion.right());
    dim.WIDTH = std::max(outerRegion.left(), outerRegion.right()) - dim.HMIN;
    dim.VMIN = std::min(outerRegion.top(), outerRegion.bottom());
    dim.HEIGHT = std::max(outerRegion.top(), outerRegion.bottom()) - dim.VMIN;
    // std::cout << __FUNCTION__ << "TileSize is " << conf_params->at("TileSize").value << std::endl;
    //dim.TILE_SIZE = stoi(conf_params->at("TileSize").value);
    dim.TILE_SIZE = 10;
    dim.MAX_HEIGHT = 1600;

    QStringList ls = QString::fromStdString(conf_params->at("ExcludedObjectsInCollisionCheck").value).replace(" ", "" ).split(',');
    std::cout << __FILE__ << __FUNCTION__ << " " << ls.size() << "objects read for exclusion list" << std::endl;
    std::vector<std::string> excluded_objects;
    foreach(const QString &s, ls)
        excluded_objects.emplace_back(s.toStdString());

    collisions =  std::make_shared<Collisions>();
    collisions->initialize(G, excluded_objects);

    qInfo() << __FUNCTION__ << dim.HMIN << dim.WIDTH << dim.VMIN << dim.HEIGHT;

    if(read_from_file and not file_name.empty())
    {
        //readFromFile(file_name);
    }
    else
    {
        std::cout << __FUNCTION__ << "Collisions - checkRobotValidStateAtTargetFast" << std::endl;
        auto G_copy = G->G_copy();
        for (int i = dim.HMIN; i < dim.HMIN + dim.WIDTH; i += dim.TILE_SIZE)
        {
            for (int j = dim.VMIN; j < dim.VMIN + dim.HEIGHT; j += dim.TILE_SIZE)
            {
                for(int z = 0; z < dim.MAX_HEIGHT ; z += dim.TILE_SIZE)
                {
//                    bool occupied = collisions->checkOccupancyOfVoxel(G_copy, i, j, z, dim.TILE_SIZE);
//                    occupied = true;
                }
            }
            std::cout << __FUNCTION__ << " Progress: " << i*100/(dim.HMIN+dim.WIDTH) << std::endl;
        }
        //if(not file_name.empty())
        //    saveToFile(file_name);
    }
}
///////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////////////////////
void SpecificWorker::update_node_slot(const std::int32_t id, const std::string &type)
{
    if (type == laser_type)    // Laser node updated
    {
        //qInfo() << __FUNCTION__ << " laser node change";
        if( auto node = G->get_node(id); node.has_value())
        {
            auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
            auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
            if(dists.has_value() and angles.has_value())
            {
                if(dists.value().get().empty() or angles.value().get().empty()) return;
                //qInfo() << __FUNCTION__ << dists->get().size();
                laser_buffer.put(std::make_tuple(angles.value().get(), dists.value().get()),
                                 [this](const LaserData &in, std::vector<octomap::point3d> &out) {
                                     const auto &[angles, dists] = in;
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
                                     {
                                         //convert laser polar coordinates to cartesian
                                         float x = dist * sin(angle);
                                         float y = dist * cos(angle);
                                         Mat::Vector3d laserWorld = inner_eigen->transform(world_name,Mat::Vector3d(x, y, 0), laser_name).value();
                                         out.emplace_back(laserWorld.x(), laserWorld.y(), 10);
                                     }
                                 });
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////77
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompDSRGetID you can call this methods:
// this->dsrgetid_proxy->getID(...)

