//
// Created by robolab on 24/01/20.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <Laser.h>
#include <CommonBehavior.h>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>
#include <tuple>
#include <QVector2D>

#include <dsr/api/dsr_inner_api.h>

class Controller
{
    public:
        using retUpdate = std::tuple <bool, bool, bool, float, float, float>;
        void initialize(const std::shared_ptr<DSR::InnerAPI> &innerModel_, std::shared_ptr<RoboCompCommonBehavior::ParameterList> params_);
        retUpdate update(std::vector<QPointF> points, const RoboCompLaser::TLaserData &laserData, const QPointF &target, const QVec &robotPose, const QPointF &robotNose);

    private:
        std::shared_ptr<DSR::InnerAPI> innerModel;
        QTime time;
        int delay;
        std::vector<float> baseOffsets;

        // Constants reassigned to the params values
        float MAX_ADV_SPEED;
        float MAX_ROT_SPEED;
        float MAX_SIDE_SPEED;
        float MAX_LAG; //ms
        float ROBOT_RADIUS_MM; //mm

        const float ROBOT_LENGTH = 500;
        const float FINAL_DISTANCE_TO_TARGET = 500; //mm
        float KB = 2.0;

        float advVelx = 0, advVelz = 0, rotVel = 0;
        QVector2D bumperVel;

        // compute max de gauss(value) where gauss(x)=y  y min
        float exponentialFunction(float value, float xValue, float yValue, float min);
        float rewrapAngleRestricted(const float angle);
};

#endif //PROJECT_CONTROLLER_H
