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

#include <random>
#include <iterator>
#include <cppitertools/range.hpp>

// from gist: https://gist.github.com/cbsmith/5538174
template <typename RandomGenerator = std::default_random_engine>
struct random_selector
{
    //On most platforms, you probably want to use std::random_device("/dev/urandom")()
    random_selector(RandomGenerator g = RandomGenerator(std::random_device()()))
            : gen(g) {}

    template <typename Iter>
    Iter select(Iter start, Iter end) {
        std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
        std::advance(start, dis(gen));
        return start;
    }

    //convenience function
    template <typename Iter>
    Iter operator()(Iter start, Iter end) {
        return select(start, end);
    }

    //convenience function that works on anything with a sensible begin() and end(), and returns with a ref to the value type
    template <typename Container>
    auto operator()(const Container& c) -> decltype(*begin(c))& {
        return *select(begin(c), end(c));
    }

private:
    RandomGenerator gen;
};

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

};

#endif
