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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx) 
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    role = params["role"].value;
    return true;
}

void SpecificWorker::initialize(int period) 
{
    std::cout << "Initialize worker" << std::endl;

    int argc=0;
    char *argv[0];
    node = DataStorm::Node(argc, argv);
    topic = std::make_shared<DataStorm::Topic<std::string, G>>(node, "DSR");
    topic->setWriterDefaultConfig({ Ice::nullopt, Ice::nullopt, DataStorm::ClearHistoryPolicy::Never });
    writer = std::make_shared<DataStorm::SingleKeyWriter<std::string, G>>(*topic.get(), role);
  
    //just to initialize the fake graph
    my_graph.insert(std::make_pair(0, RoboCompDSR::Content{"nodo0", 0, RoboCompDSR::Attribs(), RoboCompDSR::FanOut()}));
  
    read_thread = std::thread(&SpecificWorker::subscribeThread, this);

    timer.start(500);
}

void SpecificWorker::subscribeThread()
{
    DataStorm::Topic<std::string, G> topic(node, "DSR");
    topic.setReaderDefaultConfig({ Ice::nullopt, Ice::nullopt, DataStorm::ClearHistoryPolicy::Never });
    // regex to filter out myself as publisher
    std::string f = "^(?!" + role + "$).*$";
    auto reader = DataStorm::makeFilteredKeyReader(topic, DataStorm::Filter<string>("_regex", f.c_str()));
    std::cout << "starting reader " << std::endl;
    reader.waitForWriters();
    while(true)
    {
        try
        {
            auto sample = reader.getNextUnread();
            for( const auto &[k,v] : sample.getValue())
                cout << "Received: node " << k << "with laser_data " << v.attrs.at("laser_data_dists") << " from " << sample.getKey() << endl;
            std::cout << "--------------------" << std::endl;
        }
        catch (const std::exception &ex) 
        {   cerr << ex.what() << endl;  }
    }
}

void SpecificWorker::compute() 
{  
    static int cont = 0;
    topic->waitForReaders();
   
    // calls to graph editing method
    my_graph[0].attrs.insert_or_assign("laser_data_dists", std::to_string(cont++));

    writer->update(my_graph);
}

// some attrib has changed in some node
// void SpecificWorker::nodeAttrsChangedSLOT(DSR::IDType id, const DSR::Attribs &attrs)
// {
//     // do delta magic changing my_graph
//     // for now we update my_graph with attrs
//     for( const auto &[k,v] : attrs)
//         my_graph[0].attrs.at(k) = bestApproxTo(v);
//     writer->update(my_graph);
// }

