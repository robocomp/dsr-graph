#include "graphcrdt.h"
#include <cppitertools/range.hpp>
#include <qmat/QMatAll>
#include <QApplication>


using namespace DSR;

GraphCRDT::GraphCRDT(std::shared_ptr<DSR::Graph> graph_, const std::string &agent_name_): graph(graph_), agent_name(agent_name_)
{
    qRegisterMetaType<std::int32_t>("std::int32_t");
	qRegisterMetaType<std::string>("std::string");
	qRegisterMetaType<DSR::Attribs>("DSR::Attribs");

    int argc = 0; char *argv[0];
    node = DataStorm::Node(argc, argv);
    topic = std::make_shared<DataStorm::Topic<std::string, G>>(node, "DSR");
    topic->setWriterDefaultConfig({ Ice::nullopt, Ice::nullopt, DataStorm::ClearHistoryPolicy::Never });
    writer = std::make_shared<DataStorm::SingleKeyWriter<std::string, G>>(*topic.get(), agent_name);
    this->createGraph();
    // init subscription thread
    if(agent_name != "Laser Agent") 
        read_thread = std::thread(&GraphCRDT::subscribeThread, this);
}

GraphCRDT::~GraphCRDT()
{
    node.shutdown();
}

void GraphCRDT::createGraph()
{
	std::cout << __FUNCTION__ << "-- Entering createGraph" << std::endl;
	for(const auto &[k, v] : *graph)
	{
        RoboCompDSR::Attribs attrs;
        for(const auto &[ka, va] : v.attrs)
            attrs.insert_or_assign(ka, graph->printVisitor(va));
        RoboCompDSR::FanOut fano;
        for(const auto &[ka, va] : v.fanout)
        {
            RoboCompDSR::EdgeAttribs eattrs;
            eattrs.label = va.label;
            eattrs.from = va.from;
            eattrs.to = va.to;
            for(const auto &[ke, ve] : va.attrs)
                eattrs.attrs.insert_or_assign(ke, graph->printVisitor(ve));
            fano.insert_or_assign(ka, eattrs);
        }    
        ice_graph.insert(std::make_pair(k, RoboCompDSR::Content{v.type, v.id, attrs, fano}));
	}		

	// add edges after all nodes have been created
	for(const auto &par : *graph)
	{
		auto &node_fanout = graph->fanout(par.first);
		for( auto &[node_adj, edge_atts] : node_fanout)
		{
			auto edge_tag = graph->attr<std::string>(edge_atts.draw_attrs.at("name"));
			addEdgeSLOT(par.first, node_adj, edge_tag);
		}
	}
}

void GraphCRDT::subscribeThread()
{
    DataStorm::Topic<std::string, G> topic(node, "DSR");
    topic.setReaderDefaultConfig({ Ice::nullopt, Ice::nullopt, DataStorm::ClearHistoryPolicy::Never });
    // regex to filter out myself as publisher. Filters must be declared in the writer and in the reader
    std::string f = "^(?!" + agent_name + "$).*$";
    auto reader = DataStorm::makeFilteredKeyReader(topic, DataStorm::Filter<std::string>("_regex", f.c_str()));
    reader.waitForWriters();
    reader.onConnectedWriters([](const auto &ws){for(auto w:ws) std::cout << w << " " << std::endl;}, 
        [](auto reason, auto writer){ std::cout << "CB: " << " " << writer << std::endl;});
    
    std::cout << __FUNCTION__ << " Initiating thread" << std::endl;
    while(true)
    {
        std::cout << "Has writers: " << reader.hasWriters() << " " << reader.hasUnread() << std::endl;
        // auto printSample = [](const auto &sample){ std::cout << "Sample: " << sample.getKey() << std::endl;};
        // reader.onSamples([printSample](const auto &samples){ for(const auto &s : samples) printSample(s);}, printSample);
                
        if(reader.hasUnread())
            try
            {
                auto local = reader.getNextUnread();
            
                // recorrer ice_graph llamando a la API de graph para reescribirlo entero. Ojo que no lo borra antes
                for( const auto &[k, v] : local.getValue())
                {
                    DSR::Value value;
                    value.type = v.type;
                    value.id = v.id;
                    for(const auto &[ka, va] : v.attrs)
                        value.attrs.insert_or_assign(ka, iceToGraph(ka, va));
                    graph->addNode(k, v.type);
                    graph->addNodeAttribs(k, value.attrs);
                }
                for( const auto &[node_id, node_content] : local.getValue())
                {
                    for(const auto &[to, edge_content] : node_content.fano)
                    {
                        DSR::Attribs edge_attrs;
                        for(const auto &[name, value] : edge_content.attrs)
                            edge_attrs.insert_or_assign(name, iceToGraph(edge_content.label, value));
                        graph->addEdge(node_id, to, edge_content.label);
                        graph->addEdgeAttribs(node_id, to, edge_attrs);
                    }
                }
            }
            catch (const std::exception &ex) 
            {   std::cout << "exception: " << ex.what() << std::endl;  }
        std::this_thread::sleep_for(20ms);
    }
}

DSR::MTypes GraphCRDT::iceToGraph(const std::string &name, const std::string &val)
{
    static const std::list<std::string> string_types{ "imName", "imType", "tableType", "texture", "engine", "path", "render", "color", "name"};
    static const std::list<std::string> bool_types{ "collidable", "collide"};
    static const std::list<std::string> RT_types{ "RT"};
    static const std::list<std::string> vector_float_types{ "laser_data_dists", "laser_data_angles"};

    //std::cout << name << " " << val << std::endl;
    DSR::MTypes res;
    if(std::find(string_types.begin(), string_types.end(), name) != string_types.end())
        res = val;
    else if(std::find(bool_types.begin(), bool_types.end(), name) != bool_types.end())
    { if( val == "true") res = true; else res = false; }
    else if(std::find(vector_float_types.begin(), vector_float_types.end(), name) != vector_float_types.end())
    {
        std::vector<float> numbers;
        std::istringstream iss(val);
        std::transform(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), 
                        std::back_inserter(numbers), [](const std::string &s){ return (float)std::stod(s);});
        res = numbers;
    }
    else if(std::find(RT_types.begin(), RT_types.end(), name) != RT_types.end())
    {
        std::vector<float> numbers;
        std::istringstream iss(val);
        std::transform(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), 
                        std::back_inserter(numbers), [](const std::string &s){ return (float)std::stod(s);});
        RMat::RTMat rt;
    	rt.setTr( QVec::vec3(numbers[0], numbers[1], numbers[2]));
		rt.setRX(numbers[3]);
		rt.setRY(numbers[4]);
		rt.setRZ(numbers[5]);
        res = rt;
    }
	else 
    {   
        try
        { res = (float)std::stod(val); }
        catch(const std::exception &e) 
        { std::cout << __FUNCTION__ << "catch: " << name << " " << val << std::endl; res = std::string{""}; }
    }
    return res;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////
///// SLOTS
///////////////////////////////////////////////////////////////////////////////////////////////////////

void GraphCRDT::NodeAttrsChangedSLOT(const std::int32_t id, const DSR::Attribs& attrs)
{
    //std::cout << __FUNCTION__ << " Attribute change in node " << id << std::endl;
    // update ice_graph 
    for(const auto &[k,v] : attrs)
     {
        // std::cout << k << ": " << graph->printVisitor(v) << std::endl;
        try{ ice_graph.at(id).attrs.insert_or_assign(k, graph->printVisitor(v)); }
        catch(const std::exception &e) { std::cout << e.what() << std::endl;}
     }  
    std::cout << "Has readers: "<< writer->hasReaders() << std::endl;
    if( writer->hasReaders()) 
        writer->update(ice_graph);
}

void GraphCRDT::addNodeSLOT(std::int32_t id, const std::string &type)
{
    ice_graph.insert_or_assign(id, RoboCompDSR::Content{type, id, RoboCompDSR::Attribs(), RoboCompDSR::FanOut()});
}

void GraphCRDT::addEdgeSLOT(std::int32_t from, std::int32_t to, const std::string &tag)
{
    ice_graph.at(from).fano.insert_or_assign(to, RoboCompDSR::EdgeAttribs{tag, from, to, RoboCompDSR::Attribs()});
}
