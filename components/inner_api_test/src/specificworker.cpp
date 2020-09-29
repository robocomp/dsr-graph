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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }





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
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		this->Period = period;
		timer.start(Period);
	}

    //Inner Api
    auto innermodel = G->get_inner_api();
    std::cout << "Initialize worker" << std::endl;
    DSR::Node root = G->get_node_root().value();
    //create some nodes and RT edges to test inner_api
    DSR::Node node1 = add_node("transform", root.id());
    DSR::Node node2 = add_node("transform", node1.id());
    DSR::Node node3 = add_node("transform", node2.id());

    add_rt_edge(root.id(), node1.id(), {0,0,0}, {0,0,0}); //identity
    add_rt_edge(node1.id(), node2.id(), {1000,1000,1000}, {0,0,0}); //test translation
    add_rt_edge(node2.id(), node3.id(), {0, 0, 0}, {1.57079632679, 1.57079632679, 1.57079632679}); //test rotation

    auto n1_root_a = innermodel->getTransformationMatrixS(node1.name(), root.name());
    auto n2_n1_a = innermodel->getTransformationMatrixS(node2.name(), node1.name());
    auto n3_n2_a = innermodel->getTransformationMatrixS(node3.name(), node2.name());
    auto n3_root_a = innermodel->getTransformationMatrixS(node2.name(), node1.name());

    //get same value for transformation
    auto n1_root_b = innermodel->getTransformationMatrixS(node1.name(), root.name());
    auto n2_n1_b = innermodel->getTransformationMatrixS(node2.name(), node1.name());
    auto n3_n2_b = innermodel->getTransformationMatrixS(node3.name(), node2.name());
    auto n3_root_b = innermodel->getTransformationMatrixS(node2.name(), node1.name());

    //check cached values
    assertm(compare_matrix(n1_root_a.value(),n1_root_b.value()) == true, "No translation, rotation, test");
    assertm(compare_matrix(n2_n1_a.value(),n2_n1_b.value()) == true, "Translation test");
    assertm(compare_matrix(n3_n2_a.value(),n3_n2_b.value()) == true, "Rotation test");
    assertm(compare_matrix(n3_root_a.value(),n3_root_b.value()) == true, "Combined test");

    //change intermediate node => transform involving edge n2->n1 should changed
    add_rt_edge(node1.id(), node2.id(),{0,0,0}, {0,0,0});
    auto n1_root_c = innermodel->getTransformationMatrixS(node1.name(), root.name());
    auto n2_n1_c = innermodel->getTransformationMatrixS(node2.name(), node1.name());
    auto n3_n2_c = innermodel->getTransformationMatrixS(node3.name(), node2.name());
    auto n3_root_c = innermodel->getTransformationMatrixS(node2.name(), node1.name());

    assertm(compare_matrix(n1_root_a.value(),n1_root_c.value()) == true, "No translation rotation ");
    assertm(compare_matrix(n3_n2_a.value(),n3_n2_c.value()) == true, "Rotation test");
    assertm(compare_matrix(n3_root_a.value(),n3_root_c.value()) == false, "Combined test");
    assertm(compare_matrix(n2_n1_a.value(),n2_n1_c.value()) == false, "Translation test");

    //change intermediate node => getting initial values again
    add_rt_edge(node1.id(), node2.id(),{1000, 1000, 1000}, {0,0,0});
    auto n1_root_d = innermodel->getTransformationMatrixS(node1.name(), root.name());
    auto n2_n1_d = innermodel->getTransformationMatrixS(node2.name(), node1.name());
    auto n3_n2_d = innermodel->getTransformationMatrixS(node3.name(), node2.name());
    auto n3_root_d = innermodel->getTransformationMatrixS(node2.name(), node1.name());

    assertm(compare_matrix(n1_root_a.value(),n1_root_d.value()) == true, "No translation rotation ");
    assertm(compare_matrix(n3_n2_a.value(),n3_n2_d.value()) == true, "Rotation test");
    assertm(compare_matrix(n3_root_a.value(),n3_root_d.value()) == true, "Combined test");
    assertm(compare_matrix(n2_n1_a.value(),n2_n1_d.value()) == true, "Translation test");

    std::cout<<"ALL TESTS PASSED"<<std::endl;
    exit(0);
}

void SpecificWorker::compute()
{
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
//TODO: parent and level must be inserted automatically when new edge is created
DSR::Node SpecificWorker::add_node(const std::string &type, std::uint32_t parent)
{
    DSR::Node node;
    node.type(type);
    G->add_or_modify_attrib_local<pos_x_att>(node, 100.0f);
    G->add_or_modify_attrib_local<pos_y_att>(node, 130.0f);
    G->add_or_modify_attrib_local<color_att>(node, std::string("GoldenRod"));
    G->add_or_modify_attrib_local<parent_att>(node, parent);
    try
    {
        G->insert_node(node);
    }
    catch(const std::exception& e)
    {
        std::cout << __FUNCTION__ <<  e.what() << std::endl;
    }
    return node;
}

void SpecificWorker::add_rt_edge(int from, int to, std::vector<float> trans, std::vector<float> rot)
{
    std::optional<DSR::Node> from_node = G->get_node(from);
    std::optional<DSR::Node> to_node = G->get_node(to);
    if (from_node.has_value() and to_node.has_value())
    {
        try {
            G->insert_or_assign_edge_RT(from_node.value(), to, trans, rot);
        }
        catch (const std::exception &e) {
            std::cout << __FUNCTION__ << e.what() << std::endl;
        }
    }
}

bool SpecificWorker::compare_matrix(const RTMat &mat1, const RTMat &mat2)
{
    bool equal = true;
    for(int i=0;i < mat1.getRows();i++)
    {
        for(int j=0;j < mat1.getCols();j++)
        {
            if(mat1(i,j) != mat2(i,j)) {
                equal = false;
                break;
            }
        }
    }
    return equal;
}