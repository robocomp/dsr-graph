//
// Created by pbustos on 3/04/22.
//

#ifndef LOCAL_GRID_MPC_H
#define LOCAL_GRID_MPC_H

#include <casadi/casadi.hpp>
#include <casadi/core/optistack.hpp>
#include <Eigen/Dense>
#include <tuple>
#include <QtCore>
#include <QGraphicsEllipseItem>
#include <Laser.h>


namespace mpc
{
    class MPC
    {
        public:
            using Ball = std::tuple<Eigen::Vector2d, float, Eigen::Vector2d>;
            using Balls = std::vector<Ball>;
            using Result = std::optional<std::tuple<double, double, casadi::OptiSol, MPC::Balls>>;

            struct Constants
            {
                int num_steps = 10;                     // MPC steps ahead
                const float robot_radius = 300;
                double gauss_dist = 0.1;                // minimun distance to a lidar-gaussian as constraint
                double point_dist = 0.2;                // value in lidar-gaussian at lidar points (corners)
                double point_sigma = 0.07;              // variance for point (corners) gaussians
                double gauss_value_for_point = 0.3;     // minimun distance to a corner gaussian as constraint
                float min_dist_to_target = 0.9;         // min distance to target at which the robot stops
                double max_rotation_value = 0.5;        // max rotation constraint in rads/sg
                double max_advance_value = 0.9;           // max advance constraint in m/sg
                double min_advance_value = 0;           // min advance constraint in m/sg
                double xset_gaussian = 0.5;             // gaussian break x set value
                double yset_gaussian = 0.6;             // gaussian break y set value
                double min_line_dist = 0.4;
                float max_RDP_deviation = 70;
                float laser_noise_sigma  = 15;
                float max_ball_radius = 1 ;             // meters
                const float peak_threshold = 500;       // opening detector
                const float target_noise_sigma = 50;
                const int num_lidar_affected_rays_by_hard_noise = 1;
            };

            // target
            struct Target
            {
                private:
                    bool active = false;
                    QPointF pos, pos_ant=QPointF(0,0); //mm
                public:
                    void set_active(bool b) { active = b;}
                    bool is_active() const { return active;}
                    QPointF get_pos() const { return pos; }
                    QPointF get_pos_ant() const { return pos_ant; }
                    QPointF get_velocity() const {return pos-pos_ant;}
                    Eigen::Vector2d get_velocity_meters() const {auto v = get_velocity(); return Eigen::Vector2d(v.x()/1000.0, v.y()/1000.0);}
                    void set_pos(const QPointF &p) { pos_ant = pos; pos = p; }
                    Eigen::Vector2d to_eigen() const
                    { return Eigen::Vector2d(pos.x(), pos.y()); }
                    Eigen::Vector2d to_eigen_meters() const
                    { return Eigen::Vector2d(pos.x()/1000, pos.y()/1000); }
            };

            casadi::Opti initialize_differential(const int N);
            Result minimize_balls_path( const std::vector<Eigen::Vector2d> &path, const Eigen::Vector3d &current_pose_meters, const RoboCompLaser::TLaserData &ldata);

            casadi::MX pos;
            casadi::MX rot;

    private:
            Target target;
            Constants consts;
            casadi::Opti opti;
            std::vector<double> previous_values_of_solution, previous_control_of_solution;
            casadi::MX state;

            casadi::MX phi;
            casadi::MX control;
            casadi::MX adv;
            casadi::MX slack_vector;

            std::vector<double> e2v(const Eigen::Vector2d &v);
            Ball compute_free_ball(const Eigen::Vector2d &center, const std::vector<Eigen::Vector2d> &lpoints);
    };

} // mpc

#endif //LOCAL_GRID_MPC_H
