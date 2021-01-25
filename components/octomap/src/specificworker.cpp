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
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id); // Init nodes
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

        //Apis
        inner_eigen = G->get_inner_eigen_api();
        if( const auto cam_node = G->get_node(viriato_head_camera_name); cam_node.has_value())
            camera_api = G->get_camera_api(cam_node.value());
        else
        {
            qWarning() << __FUNCTION__ << "Not camera " << QString::fromStdString(viriato_head_camera_name) << " found in G. Aborting";
            std::terminate();
        }

        // Ignore attributes from G
        G->set_ignored_attributes<cam_rgb_att>();

        //Custom widget
        graph_viewer->add_custom_widget_to_dock("Octomap", &custom_widget);
        custom_widget.show();

        // Octomap
		octo = new octomap::OcTreeStamped(0.1);
		//initialize_octomap(false, "octomap.map");
        otr.id = 0;
        otr.octree_drawer = new octomap::OcTreeDrawer();
        otr.octree_drawer->enableOcTree();
        otr.octree = octo;
        otr.origin = octomap::pose6d();
        custom_widget.addSceneObject(otr.octree_drawer);

        initialize_octomap();

		timer.start(200);
	}
}

void SpecificWorker::compute()
{
    static octomap::point3d robot_pose{0,0,0};
    if( const auto r_pose = robot_pose_buffer.try_get(); r_pose.has_value())
        robot_pose = r_pose.value();
    if( const auto scan = laser_buffer.try_get(); scan.has_value())
        octo->insertPointCloud(scan.value(), robot_pose, 10);
    if( const auto cloud = pointcloud_buffer.try_get(); cloud.has_value())
        octo->insertPointCloudRays(cloud.value(), robot_pose, 10);

    std::cout << "Total in tree: " << octo->memoryUsage() << std::endl;
    //octo->updateInnerOccupancy();
    show_OcTree();
    //forget_data(5);
    octo->degradeOutdatedNodes(5);
    //project_map_on_floor();

    //std::ostringstream data;
    //octo->writeBinary(data);

}

//void SpecificWorker::project_map_on_floor()
//{
//    for(octomap::OcTreeStamped::leaf_iterator it = octo->begin_leafs(), end = octo->end_leafs(); it!= end; ++it)
//    {
//        if(octo->isNodeOccupied(*it))
//        {
//            const auto coor = it.getCoordinate();
//            //qInfo() << coor.x()*1000 << coor.y()*1000;
//            grid.setOccupied(grid.pointToGrid(coor.x()*1000, coor.y()*1000));
//        }
//    }
//    qInfo() << "Grid size " << grid.size();
//}


//std::list<QPointF> SpecificWorker::computePath(const QPointF &source_, const QPointF &target_)
//{
//    //Key source = pointToGrid(source_.x(), source_.y());
//    //Key target = pointToGrid(target_.x(), target_.y());
//    octomap::OcTreeKey source(source_.x(), source_.y(), 0.1);
//    octomap::OcTreeKey target(target_.x(), target_.y(), 0.1);
//
//    // Admission rules
//    if (!(target.x >= dim.HMIN and target.x < dim.HMIN + dim.WIDTH and target.z >= dim.VMIN and target.z < dim.VMIN + dim.HEIGHT))
//    {
//        qDebug() << __FUNCTION__ << "Target " << target_.x() << target_.y() << "out of limits " << dim.HMIN << dim.VMIN << dim.HMIN+dim.WIDTH << dim.VMIN+dim.HEIGHT
//                 << "Returning empty path";
//        return std::list<QPointF>();
//    }
//    if (!(source.x >= dim.HMIN and source.x < dim.HMIN + dim.WIDTH and source.z >= dim.VMIN and source.z < dim.VMIN + dim.HEIGHT))
//    {
//        qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
//        return std::list<QPointF>();
//    }
//    if (source == target)
//    {
//        qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
//        return std::list<QPointF>();
//    }
//    // vector de distancias inicializado a DBL_MAX
//    std::vector<double> min_distance(octo->getNumLeafNodes(), std::numeric_limits<double>::max());
//
//    // std::uint32_t id with source value
//    //const auto &[success, val] = getCell(source);
//    auto val =  octo->search(source);
//    if(val == nullptr)
//    {
//        qWarning() << __FUNCTION__ << " Could not find source position in Grid";
//        return {};
//    }
//    auto id = val->getValue();
//    // initialize source position to 0
//    min_distance[id] = 0;
//    // vector de pares<std::uint32_t,Key> initialized to (-1, Key())
//    std::vector<std::pair<std::uint32_t, Key>> previous(fmap.size(), std::make_pair(-1, Key()));
//    // lambda to compare two vertices: a < b if a.id<b.id or
//    auto comp = [this](std::pair<std::uint32_t, Key> x, std::pair<std::uint32_t, Key> y) {
//        if (x.first <= y.first)
//            return true;
//            //else if(x.first == y.first)
//            //	return std::get<T&>(getCell(x.second)).id <= std::get<T&>(getCell(y.second)).id;
//        else
//            return false;
//    };
//
//    // OPEN List
//    std::set<std::pair<std::uint32_t, Key>, decltype(comp)> active_vertices(comp);
//    active_vertices.insert({0, source});
//
//    while (!active_vertices.empty())
//    {
//        Key where = active_vertices.begin()->second;
//        if (where == target)
//        {
////				qDebug() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap.at(where).id];  //exit point
//            auto p = orderPath(previous, source, target);
//            if (p.size() > 1)
//                return p;
//            else
//                return std::list<QPointF>();
//        }
//        active_vertices.erase(active_vertices.begin());
//        for (auto ed : neighboors_8(where))
//        {
////				qDebug() << __FILE__ << __FUNCTION__ << "antes del if" << ed.first.x << ed.first.z << ed.second.id << fmap[where].id << min_distance[ed.second.id] << min_distance[fmap[where].id];
//            if (min_distance[ed.second.id] > min_distance[fmap[where].id] + ed.second.cost)
//            {
//                active_vertices.erase({min_distance[ed.second.id], ed.first});
//                min_distance[ed.second.id] = min_distance[fmap[where].id] + ed.second.cost;
//                previous[ed.second.id] = std::make_pair(fmap[where].id, where);
//                active_vertices.insert({min_distance[ed.second.id], ed.first}); // Djikstra
//                // active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } ); //A*
//            }
//        }
//    }
//    qDebug() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") not  found. Returning empty path";
//    return std::list<QPointF>();
//};


///////////////////
void SpecificWorker::forget_data( unsigned int time_thres )
{
    unsigned int query_time = (unsigned int) time(NULL);
    for(octomap::OcTreeStamped::leaf_iterator it = octo->begin_leafs(), end = octo->end_leafs(); it!= end; ++it)
    {
        if ( octo->isNodeOccupied(*it) and ((query_time - it->getTimestamp()) > time_thres) )
        {
            //octo->integrateMissNoTime(&*it);
            octo->deleteNode(it.getKey());
        }
    }
}
void SpecificWorker::show_OcTree()
{
    // update viewer stat
    double minX, minY, minZ, maxX, maxY, maxZ;
    minX = minY = minZ = -10; // min bbx for drawing
    maxX = maxY = maxZ = 10;  // max bbx for drawing
    double sizeX, sizeY, sizeZ;
    sizeX = sizeY = sizeZ = 0.;
    size_t memoryUsage = 0;
    size_t num_nodes = 0;
    size_t memorySingleNode = 0;

    // get map bbx
    double lminX, lminY, lminZ, lmaxX, lmaxY, lmaxZ;
    otr.octree->getMetricMin(lminX, lminY, lminZ);
    otr.octree->getMetricMax(lmaxX, lmaxY, lmaxZ);
    // transform to world coords using map origin
    octomap::point3d pmin(lminX, lminY, lminZ);
    octomap::point3d pmax(lmaxX, lmaxY, lmaxZ);
    pmin = otr.origin.transform(pmin);
    pmax = otr.origin.transform(pmax);
    lminX = pmin.x(); lminY = pmin.y(); lminZ = pmin.z();
    lmaxX = pmax.x(); lmaxY = pmax.y(); lmaxZ = pmax.z();
    // update global bbx
    if (lminX < minX) minX = lminX;
    if (lminY < minY) minY = lminY;
    if (lminZ < minZ) minZ = lminZ;
    if (lmaxX > maxX) maxX = lmaxX;
    if (lmaxY > maxY) maxY = lmaxY;
    if (lmaxZ > maxZ) maxZ = lmaxZ;
    double lsizeX, lsizeY, lsizeZ;
    // update map stats
    otr.octree->getMetricSize(lsizeX, lsizeY, lsizeZ);
    if (lsizeX > sizeX) sizeX = lsizeX;
    if (lsizeY > sizeY) sizeY = lsizeY;
    if (lsizeZ > sizeZ) sizeZ = lsizeZ;
    memoryUsage += otr.octree->memoryUsage();
    num_nodes += otr.octree->size();
    memorySingleNode = std::max(memorySingleNode, otr.octree->memoryUsageNode());

    custom_widget.setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));

    QString size = QString("%L1 x %L2 x %L3 m^3; %L4 nodes").arg(sizeX).arg(sizeY).arg(sizeZ).arg(unsigned(num_nodes));
    QString memory = QString("Single node: %L1 B; ").arg(memorySingleNode)
                     + QString ("Octree: %L1 B (%L2 MB)").arg(memoryUsage).arg((double) memoryUsage/(1024.*1024.), 0, 'f', 3);
    //m_mapMemoryStatus->setText(memory);
    //m_mapSizeStatus->setText(size);
    custom_widget.update();

    // generate cubes -> display
    // timeval start;
    // timeval stop;
    // gettimeofday(&start, NULL);  // start timer
//    for (std::map<int, OcTreeRecord>::iterator it = m_octrees.begin(); it != m_octrees.end(); ++it) {
//        it->second.octree_drawer->setMax_tree_depth(m_max_tree_depth);
//        it->second.octree_drawer->setOcTree(*it->second.octree, it->second.origin, it->second.id);
//    }

    otr.octree_drawer->setOcTree(*otr.octree, otr.origin, otr.id);
    //    gettimeofday(&stop, NULL);  // stop timer
    //    double time_to_generate = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
    //    fprintf(stderr, "setOcTree took %f sec\n", time_to_generate);
    custom_widget.update();
}
void SpecificWorker::initialize_octomap(bool read_from_file, const std::string file_name)
{
    qDebug() << __FUNCTION__ << "FileName:" << QString::fromStdString(file_name);
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
    dim.TILE_SIZE = 100;
    //dim.MAX_HEIGHT = 1600;

//    grid.initialize(dim);
//    QStringList ls = QString::fromStdString(conf_params->at("ExcludedObjectsInCollisionCheck").value).replace(" ", "" ).split(',');
//    std::cout << __FILE__ << __FUNCTION__ << " " << ls.size() << "objects read for exclusion list" << std::endl;
//    std::vector<std::string> excluded_objects;
//    foreach(const QString &s, ls)
//        excluded_objects.emplace_back(s.toStdString());
//
//    collisions =  std::make_shared<Collisions>();
//    collisions->initialize(G, excluded_objects);
//
//    qInfo() << __FUNCTION__ << dim.HMIN << dim.WIDTH << dim.VMIN << dim.HEIGHT;
//
//    if(read_from_file and not file_name.empty())
//    {
//        //readFromFile(file_name);
//    }
//    else
//    {
//        std::cout << __FUNCTION__ << "Collisions - checkRobotValidStateAtTargetFast" << std::endl;
//        auto G_copy = G->G_copy();
//        for (int i = dim.HMIN; i < dim.HMIN + dim.WIDTH; i += dim.TILE_SIZE)
//        {
//            for (int j = dim.VMIN; j < dim.VMIN + dim.HEIGHT; j += dim.TILE_SIZE)
//            {
//                for(int z = 0; z < dim.MAX_HEIGHT ; z += dim.TILE_SIZE)
//                {
////                    bool occupied = collisions->checkOccupancyOfVoxel(G_copy, i, j, z, dim.TILE_SIZE);
////                    occupied = true;
//                }
//            }
//            std::cout << __FUNCTION__ << " Progress: " << i*100/(dim.HMIN+dim.WIDTH) << std::endl;
//        }
//        //if(not file_name.empty())
//        //    saveToFile(file_name);
//    }
}
///////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
///////////////////////////////////////////////////////////////////
void SpecificWorker::update_node_slot(const std::uint64_t id, const std::string &type)
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
                                 [this](const LaserData &in, octomap::Pointcloud &out) {
                                     const auto &[angles, dists] = in;
                                     Mat::Vector3d laser_world;
                                     octomap::Pointcloud pointcloud;
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
                                     {
                                         //convert laser polar coordinates to cartesian
                                         float x = dist * sin(angle); float y = dist * cos(angle); float z = 10;
                                         laser_world = inner_eigen->transform(world_name,Mat::Vector3d(x, y, z), laser_name).value();
                                         //qInfo() << laser_world.x() << laser_world.y() << laser_world.z();
                                         pointcloud.push_back(laser_world.x()/1000., laser_world.y()/1000., laser_world.z()/1000.);
                                     }
                                     out = pointcloud;
                                 });
            }
        }
    }
    else if (type == omnirobot_type)    // Laser node updated
    {
        if(auto r = inner_eigen->transform(world_name, robot_name); r.has_value())
            robot_pose_buffer.put(octomap::point3d(r.value().x(), r.value().y(), r.value().z()));
    }
    if (type == rgbd_type)    // Laser node updated
        if( auto node = G->get_node(id); node.has_value())
        {
            if( std::optional<std::vector<std::tuple<float,float,float>>> depth_data_o = G->get_pointcloud(node.value(), world_name, 10); depth_data_o.has_value())
            {
                pointcloud_buffer.put(depth_data_o.value(), //lambda transforms from float tuple to octomap::pointcloud and changes to meters
                            [this](const std::vector<std::tuple<float,float,float>> &depth_data, octomap::Pointcloud &out) {
                                octomap::Pointcloud pointcloud;
                                for (const auto &[x, y, z] : depth_data)
                                {
                                    pointcloud.push_back(x / 1000, y / 1000,
                                                         z / 1000);  //change to write on out directly after clearing it
                                }
                                out = pointcloud;
                            });
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

