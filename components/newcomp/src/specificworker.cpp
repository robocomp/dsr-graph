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
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    role = params["role"].value;
    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    timer.start(Period);
}

void SpecificWorker::compute() {

    int argc=0;
    char *argv[0];
    
    if (role == "publicator") {
        cout << "Im working as publicator..." << endl;
        try {
            DataStorm::Node node(argc, argv);
            DataStorm::Topic <string, string> topic(node, "hello");
            auto writer = DataStorm::makeSingleKeyWriter(topic, "foo");
            topic.waitForReaders();
            writer.update("hello");
            topic.waitForNoReaders();

        }
        catch (const std::exception &ex) {
            cerr << ex.what() << endl;
        }

    } else {
        cout << "Im working as reader..." << endl;
        try {
            DataStorm::Node node(argc, argv);
            DataStorm::Topic <string, string> topic(node, "hello");
            auto reader = DataStorm::makeSingleKeyReader(topic, "foo");
            auto sample = reader.getNextUnread();
            cout << sample.getKey() << " says " << sample.getValue() << "!" << endl;
        }
        catch (const std::exception &ex) {
            cerr << ex.what() << endl;
        }
    }
    sleep(1);


}



