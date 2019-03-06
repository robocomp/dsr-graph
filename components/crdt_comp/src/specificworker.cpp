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

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    agent_name = params["agent_name"].value;
    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;

    gcrdt = std::make_shared<CRDT::CRDTGraph>(0, agent_name); // Init nodes
    gcrdt->read_from_file("caca.xml");

    gcrdt->start_fullgraph_server_thread();
    gcrdt->start_fullgraph_request_thread();
    sleep(TIMEOUT);
    gcrdt->start_subscription_thread(true);
//    gcrdt->print();
    std::cout << "Starting compute" << std::endl;

    // GraphViewer creation
    graph_viewer = std::make_unique<DSR::GraphViewer>(std::shared_ptr<SpecificWorker>(this));
    setWindowTitle( agent_name.c_str() );

    // Random
    mt = std::mt19937(rd());
    dist = std::uniform_real_distribution((float)-40.0, (float)40.0);
    randomNode = std::uniform_int_distribution((int)100, (int)140.0);
    timer.start(20);
}


void SpecificWorker::tester() {
    try {
        static int cont = 0, laps = 1;
        if (laps < LAPS) {
            try {
                cont++;
                auto test = RoboCompDSR::Node{
                        "foo_id:" + std::to_string(cont) + "_laps:" + std::to_string(laps) + "_" + agent_name, cont};
//            std::cout <<" New node: "<< test << std::endl;
                gcrdt->insert_or_assign(cont, test);
            }
            catch (const std::exception &ex) { cerr << __FUNCTION__ << " -> " << ex.what() << std::endl; }

            if (cont == NODES) {
                cont = 0;
                laps++;
            }
        } else if (laps == LAPS) {
//            gcrdt->print();
            laps++;
        } else
            sleep(5);
    } catch(const std::exception &e){ std::cout <<__FILE__ << " " << __FUNCTION__ << " "<< e.what() << std::endl;};
}

void SpecificWorker::test_laser() {
    static int cont = 0;
    if (cont<LAPS) {
        try {
            // robot update
            RoboCompGenericBase::TBaseState bState;
            differentialrobot_proxy->getBaseState(bState);
            auto base_id = gcrdt->get_id_from_name("base");
            auto world_id = gcrdt->get_id_from_name(
                    "world");  //OJO: THERE CAN BE SEVERAL EDGES BETWEEN TWO NODES WITH DIFFERENT LABELS
            RMat::RTMat rt;
            rt.setTr(QVec::vec3(bState.x, 0, bState.z));
            rt.setRX(0.f);
            rt.setRY(bState.alpha);
            rt.setRZ(0.f);
            gcrdt->add_edge_attribs(world_id, base_id, RoboCompDSR::Attribs{
                    std::make_pair("RT", RoboCompDSR::AttribValue{"RTMat", rt.serializeAsString(), 1})});

            // laser update
            auto ldata = laser_proxy->getLaserData();
            std::vector<float> dists;
            std::transform(ldata.begin(), ldata.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
            int node_id = gcrdt->get_id_from_name("laser");
            std::string s, a;
            for (auto &x : dists)
                s += std::to_string(x) + " ";

            std::vector<float> angles;
            std::transform(ldata.begin(), ldata.end(), std::back_inserter(angles),
                           [](const auto &l) { return l.angle; });
            for (auto &x : angles)
                a += std::to_string(x) + " ";

            RoboCompDSR::Attribs ma;
            ma.insert_or_assign("laser_data_dists", RoboCompDSR::AttribValue{"vector<float>", s, (int) dists.size()});
            ma.insert_or_assign("laser_data_angles", RoboCompDSR::AttribValue{"vector<float>", a, (int) angles.size()});
            gcrdt->add_node_attribs(node_id, ma);

            std::cout<<"Working..."<<cont<<std::endl;
            cont++;
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Laser" << e << std::endl;
        }
    }
}

void SpecificWorker::test_nodes_mov() {
    static int cont = 0;
    if (cont<LAPS) {
        try {
            for (auto x : gcrdt->get_list()) {
                for (auto &[k, v] : x.attrs) {
                    if(k == "pos_x" || k == "pos_y") {
                        std::string nValue = std::to_string(std::stoi(v.value) + dist(mt));
                        cout << "Nodo: "<<x.id<<", antes: "<<v<<", ahora: "<<nValue<<endl;
                        gcrdt->add_node_attrib(x.id, k, v.type, nValue, v.length);
                    }
                }
            }
            std::cout<<"Working..."<<cont<<std::endl;
            cont++;
//            auto toDelete = randomNode(mt);
//            std::cout<<"Deleting.... "<<toDelete<<std::endl;
//            gcrdt->delete_node(toDelete);
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Laser" << e << std::endl;
        }
    } else if (cont == LAPS)
    {
//        auto to_delete = randomNode(mt);
////        int to_delete = 118;
//        std::cout<<"Antes "<<to_delete<<std::endl;
//        gcrdt->delete_node(to_delete);
//        std::cout<<"Fin "<<std::endl;
        cont++;
    } else
        std::cout<<"nada "<<std::endl;
}

void SpecificWorker::test_node_random()
{   static int cont = 0;
    if (cont<NODES) {
        try {
            int to_move = randomNode(mt);
            std::cout<<"["<<cont<<"] to_move: "<<to_move<<std::endl;
            float p_x = gcrdt->get_node_attrib_by_name<float>(to_move, "pos_x");
            p_x+= dist(mt);
            float p_y = gcrdt->get_node_attrib_by_name<float>(to_move, "pos_y");
            p_y+= dist(mt);
            gcrdt->add_node_attrib(to_move, "pos_x", p_x);
            gcrdt->add_node_attrib(to_move, "pos_y", p_y);
        } catch (const std::exception &e) {
            std::cout << "EXCEPTION: " << __FILE__ << " " << __FUNCTION__ << ":" << __LINE__ << " " << e.what()
                      << std::endl;
        };
        cont++;
    }
}

void SpecificWorker::compute()
{
//    test_laser();
//    test_nodes_mov();
    test_node_random();
}