/*
 *    Copyright (C)2018 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx) {
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
    gcrdt.reset();

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) 
{
    agent_name = params["agent_name"].value;
    read_file = params["read_file"].value == "true";
    agent_id = stoi(params["agent_id"].value);
    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    // create graph
    gcrdt = std::make_shared<CRDT::CRDTGraph>(0, agent_name, agent_id); // Init nodes

    // read graph content from file
    if(read_file)
    {
        gcrdt->read_from_file("grafo.xml");
        gcrdt->start_fullgraph_server_thread();
        gcrdt->start_subscription_thread(true);

    }
    else
    {
        //De momento no funciona bien
        gcrdt->start_fullgraph_request_thread();

    }

    sleep(TIMEOUT);
    //gcrdt->start_subscription_thread(false);
    //gcrdt->print();
    std::cout << "Starting compute" << std::endl;

    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );

    // Random initialization
    mt = std::mt19937(rd());
    dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
    randomNode = std::uniform_int_distribution((int)100, (int)140.0);
    timer.start(20);
}

void SpecificWorker::compute()
{
    if(read_file)
        test_laser();

   //   test_nodes_mov();
   // test_node_random();
}


void SpecificWorker::test_laser() 
{
    static int cont = 0;
    //if (cont<LAPS) {
         // get robot position
        try {
            RoboCompGenericBase::TBaseState bState;
            differentialrobot_proxy->getBaseState(bState);
            // get corresponding nodes in G
            //auto base = gcrdt->get_node("base");
            //auto world = gcrdt->get_node("world");
                    //OJO: THERE CAN BE SEVERAL EDGES BETWEEN TWO NODES WITH DIFFERENT LABELS
            // create Rt mat coding the robot's pose in the world
            RMat::RTMat rt;
            rt.setTr(QVec::vec3(bState.x, 0, bState.z));
            rt.setRX(0.f); rt.setRY(bState.alpha); rt.setRZ(0.f);
            // add Rt mat as the edge attribute between world and robot

            auto edge = gcrdt->get_edge( "world", "base");

            AttribValue at;
            at.type(RT_MAT);
            Val v;
            v.str(rt.serializeAsString());
            at.value(v);
            at.key("RT");

            std::map<string, AttribValue> attribMap;
            attribMap["RT"] = at;

            edge.attrs(attribMap);

            gcrdt->insert_or_assign_edge(edge);
            //gcrdt->add_edge_attribs(world_id, base_id, attribMap);
        }
        catch (const Ice::Exception &e) { std::cout << "Error reading from Base" << e << std::endl;}
        // get laser values
        try
        {
            auto ldata = laser_proxy->getLaserData();
            // extract distances 
            std::vector<float> dists;
            std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
            // get corresponding node
            // convert values to string
            //std::string s = "", a = "";
            //for (auto &x : dists)
            //    s += std::to_string(x) + " ";
            // extract angles
            std::vector<float> angles;
            std::transform(ldata.begin(), ldata.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });
            // convert angles to string
            //for (auto &x : angles)
            //    a += std::to_string(x) + " ";
            // insert both strings as attributes
            //std::vector<AttribValue> ma;
            //std::cout << "Size a, s: " <<  a.size() + s.size() << std::endl;
            //AttribValue at;
            //at.type("vector<float>");
            //at.value(s);
            //at.length((int) dists.size());
            //at.key("laser_data_dists");


            //ma.push_back(at);

            //AttribValue at2;
            //at2.type("vector<float>");
            //at2.value(a);
            //at2.length((int) angles.size());
            //at2.key("laser_data_angles");
            //ma.push_back(at2);
            // add attributes to node
            auto node = gcrdt->get_node("laser");

            gcrdt->add_attrib(node.attrs(), "laser_data_dists", dists);
            gcrdt->add_attrib(node.attrs(), "laser_data_angles", angles);

            gcrdt->insert_or_assign_node(node);
            //gcrdt->add_node_attribs(node_id, ma);
        }
        catch (const Ice::Exception &e) { std::cout << "Error reading from Laser" << e << std::endl; }
        
        std::cout<<"Working..." << cont << std::endl;
        cont++;
    //}
}

// void SpecificWorker::test_nodes_mov() {
//     static int cont = 0;
//     if (cont<LAPS) {
//         try {
//             for (auto x : gcrdt->get_list()) {
//                 for (auto &[k, v] : x.attrs) {
//                     if(k == "pos_x" || k == "pos_y") {
//                         std::string nValue = std::to_string(std::stoi(v.value) + dist(mt));
//                         cout << "Nodo: "<<x.id<<", antes: "<<v<<", ahora: "<<nValue<<endl;
//                         gcrdt->add_node_attrib(x.id, k, v.type, nValue, v.length);
//                     }
//                 }
//             }
//             std::cout<<"Working..."<<cont<<std::endl;
//             cont++;
// //            auto toDelete = randomNode(mt);
// //            std::cout<<"Deleting.... "<<toDelete<<std::endl;
// //            gcrdt->delete_node(toDelete);
//         }
//         catch (const Ice::Exception &e) {
//             std::cout << "Error reading from Laser" << e << std::endl;
//         }
//     } else if (cont == LAPS)
//     {
// //        auto to_delete = randomNode(mt);
// ////        int to_delete = 118;
// //        std::cout<<"Antes "<<to_delete<<std::endl;
// //        gcrdt->delete_node(to_delete);
// //        std::cout<<"Fin "<<std::endl;
//         cont++;
//     } else
//         std::cout<<"nada "<<std::endl;
// }

// void SpecificWorker::test_node_random()
// {   static int cont = 0;
//     if (cont<NODES) {
//         try {
//             int to_move = randomNode(mt);
//             if (gcrdt->in(to_move))
//             {
//                 std::cout << "[" << cont << "] to_move: " << to_move << std::endl;
//                 float p_x = gcrdt->get_node_attrib_by_name<float>(to_move, "pos_x");
//                 p_x += dist(mt);
//                 float p_y = gcrdt->get_node_attrib_by_name<float>(to_move, "pos_y");
//                 p_y += dist(mt);
//                 gcrdt->add_node_attrib(to_move, "pos_x", p_x);
//                 gcrdt->add_node_attrib(to_move, "pos_y", p_y);
//             }
//         } catch (const std::exception &e) {
//             std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
//                       << std::endl;
//         };
//         cont++;
//     }
// }

// void SpecificWorker::tester() {
//     try {
//         static int cont = 0, laps = 1;
//         if (laps < LAPS) {
//             try {
//                 cont++;
//                 auto test = Node{
//                         "foo_id:" + std::to_string(cont) + "_laps:" + std::to_string(laps) + "_" + agent_name, cont};
// //            std::cout <<" New node: "<< test << std::endl;
//                 gcrdt->insert_or_assign(cont, test);
//             }
//             catch (const std::exception &ex) { cerr << __FUNCTION__ << " -> " << ex.what() << std::endl; }

//             if (cont == NODES) {
//                 cont = 0;
//                 laps++;
//             }
//         } else if (laps == LAPS) {
// //            gcrdt->print();
//             laps++;
//         } else
//             sleep(5);
//     } catch(const std::exception &e){ std::cout <<__FILE__ << " " << __FUNCTION__ << " "<< e.what() << std::endl;};
// }
