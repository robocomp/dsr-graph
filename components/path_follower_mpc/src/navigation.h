//
// Created by robolab on 17/01/20.
//

#ifndef PROJECT_NAVIGATION_H
#define PROJECT_NAVIGATION_H
#include <math.h>
#include "collisions.h"
#include <QPolygonF>
#include <QGraphicsScene>
#include <QPointF>

#include <cppitertools/chain.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/filter.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/slice.hpp>
#include <algorithm>
#include <localPerson.h>
#include <typeinfo>
#include <dsr/api/dsr_inner_api.h>
#include "grid.h"
#include "controller.h"

// Map
struct TMapDefault
{
};

struct TContDefault
{
    TContDefault(){};
};

template<typename TMap = TMapDefault, typename TController = TContDefault>
class Navigation
{
    public:
        void initialize( const std::shared_ptr<DSR::DSRGraph> &graph,
                         std::shared_ptr< RoboCompCommonBehavior::ParameterList > configparams_,
                         QGraphicsScene *scene,
                         bool read_from_file = false,
                         std::string file_name = std::string());
        void stopRobot();
        bool isCurrentTargetActive();
        enum class State{BLOCKED, PATH_NOT_FOUND, RUNNING, AT_TARGET, IDLE, DUMMY};
        enum class SearchState{AT_TARGET, NO_TARGET_FOUND,  NEW_TARGET};
        State update();
        std::string checkPathState();
        void newRandomTarget();
        void newTarget(Mat::Vector2d newT);
        void print_state(State state) const;
        std::tuple<SearchState, Mat::Vector2d> search_a_feasible_target(const Node &target, const std::map<std::string, double> &params, const Node &robot);

        // Target
        struct Target : public std::mutex
        {
            Mat::Vector2d p;
            std::atomic_bool active = false;
            std::atomic_bool blocked = true;
            std::atomic_bool humanBlock = false;
            bool operator==(const Mat::Vector2d &t) const { return p==t; };
        };
        Target current_target;
        bool robotAutoMov = false;
        bool moveRobot = false;
        bool stopMovingRobot = false;

        float KE;
        float KI;

    private:
        std::shared_ptr<DSR::DSRGraph> G;
        std::shared_ptr<Collisions> collisions;
        std::shared_ptr<DSR::InnerAPI> innerModel;
        std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;
        std::shared_ptr<DSR::RT_API> rt_api;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;

        //typedef struct { float dist; float angle;} LocalPointPol;

        //laser
        using dists = std::vector<float>;
        using angles = std::vector<float>;
        using LaserData = std::tuple<angles, dists>;
        LaserData read_laser_from_G();
        std::tuple<QPolygonF, std::vector<QPointF>> updateLaserPolygon(const LaserData &lData);

        TMap grid;
        TController controller;
        const std::string robot_name = "omnirobot";
        const std::string world_name = "world";
        const std::string floor_name = "infiniteFloor";
        const std::string laser_name = "laser";
        Grid<>::Dimensions dim;

        // Scene
        QGraphicsScene *viewer_2d;

        // ElasticBand
        std::vector<QPointF> pathPoints;
        const float ROBOT_LENGTH = 500;
        const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.9;
        bool targetBehindRobot = false;

        //Draw
        QTime reloj = QTime::currentTime();
        QPointF lastPointInPath, currentRobotNose;
        QPolygonF currentRobotPolygon;
        //laser_poly;
        //std::vector<QPointF> laser_cart;
        Mat::Vector6d currentRobotPose;
        float robotXWidth, robotZLong; //robot dimensions read from config
        Mat::Vector3d robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
        bool gridChanged = false;
        std::map<float, vector<QPolygonF>> mapCostObjects;
        std::vector<QGraphicsLineItem *> scene_road_points;
        void drawRoad(const QPolygonF &laser_poly);

        void updateFreeSpaceMap(bool drawGrid = true);
        bool findNewPath();
        bool isVisible(QPointF p, const QPolygonF &laser_poly);
        void computeForces(std::vector<QPointF> &path, const std::vector<QPointF> &laser_cart, const QPolygonF &laser_poly);
        bool isPointVisitable(QPointF point);
        void addPoints(const QPolygonF &laser_poly);
        void cleanPoints(const QPolygonF &laser_poly);
        QPolygonF getRobotPolygon();
        QPointF getRobotNose();
};

#endif //PROJECT_NAVIGATION_H
