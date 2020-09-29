#include "navigation.h"
#include <QGraphicsLineItem>

template<typename TMap, typename TController>
void Navigation<TMap, TController>::initialize( const std::shared_ptr<DSR::DSRGraph> &graph,
                                                std::shared_ptr< RoboCompCommonBehavior::ParameterList > configparams_,
                                                QGraphicsScene *scene,
                                                bool read_from_file,
                                                std::string file_name)
{
    qDebug()<<"Navigation - "<< __FUNCTION__;
    G = graph;
    innerModel = G->get_inner_api();
    configparams = configparams_;
    viewer_2d = scene;
    //robot_name = configparams_->at("RobotName").value;

    stopRobot();
    //grid can't be initialized if the robot is moving
    std::optional<Node> world_node = G->get_node("world");
    if(not world_node.has_value())
    {
        qWarning() << __FILE__ << __FUNCTION__ << "World node not found in G. Aborting";
        std::terminate();
    }
    QRectF outerRegion;
    outerRegion.setLeft(G->get_attrib_by_name<OuterRegionLeft>(world_node.value()).value());
    outerRegion.setRight(G->get_attrib_by_name<OuterRegionRight>(world_node.value()).value());
    outerRegion.setBottom(G->get_attrib_by_name<OuterRegionBottom>(world_node.value()).value());
    outerRegion.setTop(G->get_attrib_by_name<OuterRegionTop>(world_node.value()).value());
    if(outerRegion.isNull())
    {
        qWarning() << __FILE__ << __FUNCTION__ << "Outer region of the scene could not be found in G. Aborting";
        std::terminate();
    }

    // if read_from_file is true we should read the parameters from the file to guarantee consistency
    dim.HMIN = std::min(outerRegion.left(), outerRegion.right());
    dim.WIDTH = std::max(outerRegion.left(), outerRegion.right()) - dim.HMIN;
    dim.VMIN = std::min(outerRegion.top(), outerRegion.bottom());
    dim.HEIGHT = std::max(outerRegion.top(), outerRegion.bottom()) - dim.VMIN;
    std::cout << __FUNCTION__ << "TileSize is " << configparams_->at("TileSize").value << std::endl;
    dim.TILE_SIZE = stoi(configparams_->at("TileSize").value);

    collisions =  std::make_shared<Collisions>();
    collisions->initialize(G, configparams);

    grid.initialize(G, collisions, dim, read_from_file, file_name);
    grid.draw(viewer_2d);

    controller.initialize(innerModel,configparams);

    robotXWidth = std::stof(configparams->at("RobotXWidth").value);
    robotZLong = std::stof(configparams->at("RobotZLong").value);
    robotBottomLeft     = QVec::vec3( - robotXWidth / 2 - 100, 0, - robotZLong / 2 - 100);
    robotBottomRight    = QVec::vec3( + robotXWidth / 2 + 100, 0, - robotZLong / 2 - 100);
    robotTopRight       = QVec::vec3( + robotXWidth / 2 + 100, 0, + robotZLong / 2 + 100);
    robotTopLeft        = QVec::vec3( - robotXWidth / 2 - 100, 0, + robotZLong / 2 + 100);
};

template<typename TMap, typename TController>
typename Navigation<TMap, TController>::State Navigation<TMap, TController>::update()
{
    //qInfo() << "Navigation - " << __FUNCTION__;
    static QTime reloj = QTime::currentTime();

    if(current_target.active.load() == false)
        return State::IDLE;

    currentRobotPose = innerModel->transform_axis("world", robot_name).value();
    auto nose_3d = innerModel->transform("world", QVec::vec3(0, 0, 250), robot_name).value();
    currentRobotNose = QPointF(nose_3d.x(), nose_3d.z());
    LaserData laser_data = read_laser_from_G();
    const auto &[laser_poly, laser_cart] = updateLaserPolygon(laser_data);
    currentRobotPolygon = getRobotPolygon();

    auto state = checkPathState();
    if(state == "PATH_NOT_FOUND")
    {
        qWarning()<< __FUNCTION__ << QString::fromStdString(state);
        return State::PATH_NOT_FOUND;
    }

    computeForces(pathPoints, laser_cart, laser_poly);
    cleanPoints(laser_poly);
    addPoints(laser_poly);
    drawRoad(laser_poly);
    auto [success, blocked, active, xVel, zVel, rotVel] = controller.update(pathPoints, laser_data, current_target.p, currentRobotPose,  currentRobotNose );

    if (blocked)
    {
        stopRobot();
        current_target.blocked.store(true);
        return State::BLOCKED;
    }
    if (!active)
    {
        stopRobot();
        current_target.active.store(false);
        pathPoints.clear();
        if(stopMovingRobot)
            moveRobot = false;
        //if(robotAutoMov)
        //    newRandomTarget();
        return State::AT_TARGET;
    }
    static float MAX_ADV_SPEED = QString::fromStdString(configparams->at("MaxZSpeed").value).toFloat();
    static float MAX_ROT_SPEED = QString::fromStdString(configparams->at("MaxRotationSpeed").value).toFloat();
    static float MAX_SIDE_SPEED = QString::fromStdString(configparams->at("MaxXSpeed").value).toFloat();
    static QMat adv_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_ADV_SPEED,MAX_ADV_SPEED}, QPointF{-20,20}}});
    static QMat rot_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_ROT_SPEED,MAX_ROT_SPEED}, QPointF{-15,15}}});
    static QMat side_conv = QMat::afinTransformFromIntervals(QList<QPair<QPointF,QPointF>>{QPair<QPointF,QPointF>{QPointF{-MAX_SIDE_SPEED,MAX_SIDE_SPEED}, QPointF{-15,15}}});

    if (!blocked and active)
        if(moveRobot)
        {
            zVel = (adv_conv * QVec::vec2(zVel,1.0))[0];
            rotVel = (rot_conv * QVec::vec2(rotVel,1.0))[0];
            xVel = (side_conv * QVec::vec2(xVel, 1.0))[0];
            auto robot_node = G->get_node(robot_name);
            G->add_or_modify_attrib_local<ref_adv_speed>(robot_node.value(),  (float)zVel);
            G->add_or_modify_attrib_local<ref_rot_speed>(robot_node.value(), (float)rotVel);
            G->add_or_modify_attrib_local<ref_side_speed>(robot_node.value(),  (float)xVel);
            G->update_node(robot_node.value());
            qInfo() << __FUNCTION__ << "xVel " << xVel << "zVel " << zVel << "rotVel" << rotVel << "Elapsed time" << reloj.restart() << "ms";
            return State::RUNNING;
        }
    return State::IDLE; // to remove warning
};

// Given a target object, search a good arriving position and orientation
// Constraints
// - in free space
// - as close to target as possible
// - robot oriented towards object
// - close to current robot pose, i.e. to the approaching path
// General procedure
// - compute object best coordinates in case it is a large object:table, shelve
// - project the coordinates on the floor
// - search a free grid point satisfying the restrictions



template<typename TMap, typename TController>
std::tuple<typename Navigation<TMap, TController>::SearchState, QVector2D> Navigation<TMap, TController>::search_a_feasible_target(const Node &target, const Node &robot, std::optional<float> x, std::optional<float> y)
{
    //  std::cout << __FUNCTION__ << " " << target.id() << " " << x.value() << " " << y.value() << std::endl;
    // if x,y not empty
    if(x.has_value() and y.has_value())
    {
        // if already in current_target return
        if (this->current_target.p == QPointF(x.value(), y.value()))
        {
            //qInfo() << __FUNCTION__  << "Exit same point";
            return std::make_tuple(SearchState::AT_TARGET, QVector2D());
        }
        // if node is floor_plane take coordinates directly
        if (target.id() == 11) // floor
        {
            this->newTarget(QPointF(x.value(), y.value()));
            return std::make_tuple(SearchState::NEW_TARGET, QVector2D(x.value(), y.value()));
        }
    }
    // get target coordinates in world
    auto tc = innerModel->transform("world", target.name()).value();
    QVector2D target_center(tc.x(), tc.z());
    // get robot coordinates in world
    auto rc = innerModel->transform("world", robot_name).value();
    QVector2D robot_center(rc.x(), rc.z());
    std::vector<QVector2D> candidates;
    std::string target_name = target.name();

    // search the whole grid. It could be

    // search in a spiral pattern away from object for the first free cell
    long int x_pos = target_center.x();
    long int y_pos = target_center.y();
    int d = dim.TILE_SIZE;
    for(int i : iter::range(grid.size()))
    {
        while (2 * x_pos * d < i)
        {
            const auto &k = Grid<>::Key(x_pos, y_pos);
            if (grid.isFree(k))
                candidates.emplace_back(QVector2D(x_pos, y_pos));
            x_pos = x_pos + d;
        }
        while (2 * y_pos * d < i)
        {
           const auto &k = Grid<>::Key(x_pos, y_pos);
           if (grid.isFree(k))
                candidates.emplace_back(QVector2D(x_pos, y_pos));
            y_pos = y_pos + d;
        }
        d = -1 * d;
        if(candidates.size() > 10)   // arbitrary number hard to fix
            break;
    }
//    for( const auto &[key, val] : grid)
//        if(val.free and grid.cellNearToOccupiedCellByObject(key, target_name))
//
//        {
//            candidates.push_back(QVector2D(key.x, key.z));
//            if(candidates.size() > 10)
//                break;
//        }
    //std::cout << __FUNCTION__ << " " << candidates.size() << std::endl;

    // sort by distances to target
    if(candidates.size() > 0)
    {
        std::sort(std::begin(candidates), std::end(candidates), [robot_center](const auto &p1, const auto &p2) {
            return (robot_center - p1).length() < (robot_center - p2).length();
        });
        this->newTarget(candidates.front().toPointF());
        return std::make_tuple(SearchState::NEW_TARGET, candidates.front());
    }
    else
        return std::make_tuple(SearchState::NO_TARGET_FOUND, QVector2D());
}


template<typename TMap, typename TController>
typename Navigation<TMap, TController>::LaserData Navigation<TMap, TController>::read_laser_from_G()
{
    qDebug() << __FUNCTION__ << "reading from DSR laser node";
    LaserData laserData;
    auto laser_node = G->get_node("laser");
    if (laser_node.has_value())
    {
        const auto lAngles = G->get_attrib_by_name<angles_att>(laser_node.value());
        const auto lDists = G->get_attrib_by_name<dists_att>(laser_node.value());
        if (lAngles.has_value() and lDists.has_value())
        {
            LaserData laserData = std::make_tuple(lAngles.value(), lDists.value());
            return laserData;
        }
        else
            qFatal("Terminate due to attributes angles or dists not found");
    }
    else
        qFatal("Terminate due to laser node not found ");
    return LaserData();
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::stopRobot()
{
    auto robot_node = G->get_node(robot_name);
    G->add_or_modify_attrib_local<ref_adv_speed>(robot_node.value(),  (float)0);
    G->add_or_modify_attrib_local<ref_rot_speed>(robot_node.value(),  (float)0);
    G->add_or_modify_attrib_local<ref_side_speed>(robot_node.value(), (float)0);
    G->update_node(robot_node.value());
    current_target.active.store(false);
}

template<typename TMap, typename TController>
bool Navigation<TMap, TController>::isCurrentTargetActive()
{
    return current_target.active.load();
}

template<typename TMap, typename TController>
std::string Navigation<TMap, TController>::checkPathState()
{
   //std::cout << __FUNCTION__ << " " << current_target.active.load() << " " <<
   //              current_target.blocked.load() << " "  << scene_road_points.size() << std::endl;
    if (current_target.active.load())
    {
        if (current_target.blocked.load())
        {
            if ( not findNewPath())
            {
                qDebug() << __FUNCTION__ << "Path not found";
                //                if(current_target.humanBlock.load()) //if the path is blocked by human the target is not deactivated
                //                    return "";
                //qDebug()<< "checkPathState - Deactivating current target";
                stopRobot();
                current_target.active.store(false);
                pathPoints.clear();
                if(robotAutoMov)
                    newRandomTarget();
                return "PATH_NOT_FOUND";;
            }
            else
            {
                // std::cout << "checkPathState - Path found" << std::endl;
                this->current_target.blocked.store(false);
                reloj.restart();
                return "PATH_FOUND";
            }
        }
        return "NOT_BLOCKED";
    }
    else
        return "NOT_ACTIVE";
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::newRandomTarget()
{
    qDebug()<<"Navigation - "<< __FUNCTION__;
    auto hmin = std::min(collisions->outerRegion.left(), collisions->outerRegion.right());
    auto width = std::max(collisions->outerRegion.left(), collisions->outerRegion.right()) - hmin;
    auto vmin = std::min(collisions->outerRegion.top(), collisions->outerRegion.bottom());
    auto height = std::max(collisions->outerRegion.top(), collisions->outerRegion.bottom()) - vmin;
    auto x = hmin + (double)rand() * width / (double)RAND_MAX;
    auto z = vmin + (double)rand() * height/ (double)RAND_MAX;
    this->current_target.lock();
    current_target.active.store(true);
    current_target.blocked.store(true);
    current_target.p = QPointF(x,z);

    this->current_target.unlock();
    qDebug()<<"New Random Target" << current_target.p;
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::newTarget(QPointF newT)
{
    std::cout << "Navigation:" << __FUNCTION__  << " arrived " << newT << std::endl;
    if(stopMovingRobot)
    {
        stopRobot();
        moveRobot = false;
    }
    current_target.active.store(true);
    current_target.blocked.store(true);
    current_target.humanBlock.store(false);
    this->current_target.lock();
        current_target.p = newT;
    this->current_target.unlock();
}


////////// GRID RELATED METHODS //////////
template<typename TMap, typename TController>
void Navigation<TMap, TController>::updateFreeSpaceMap(bool drawGrid)
{
}

/// wrapper for grif.computePath()
template<typename TMap, typename TController>
bool Navigation<TMap, TController>::findNewPath()
{
    qDebug() << __FUNCTION__;
    pathPoints.clear();

    // extract target from current_path
    this->current_target.lock();
        auto target = this->current_target.p;
    this->current_target.unlock();

    std::list<QPointF> path = grid.computePath(currentRobotNose, target);

    if (path.size() > 0)
    {
        qDebug() << __FUNCTION__ << "Path created with length " << path.size();
        pathPoints.push_back(currentRobotNose);
        for (const QPointF &p : path)
        {
            pathPoints.push_back(p);
            //printf("%.2f %.2f\n", (float)p.x(), (float)p.y());
        }
        lastPointInPath = pathPoints[pathPoints.size()-1];
        return true;
    }
    else
    {
        qDebug() << __FUNCTION__ << "Path not found from " << currentRobotNose;
        return false;
    }
}

template<typename TMap, typename TController>
bool Navigation<TMap, TController>::isVisible(QPointF p, const QPolygonF &laser_poly)
{
    std::optional<QVec> pointInLaser = innerModel->transform("laser", QVec::vec3(p.x(),0, p.y()),"world");
    return laser_poly.containsPoint(QPointF(pointInLaser.value().x(), pointInLaser.value().z()), Qt::OddEvenFill);
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::computeForces(std::vector<QPointF> &path, const std::vector<QPointF> &laser_cart, const QPolygonF &laser_poly)
{
    if (path.size() < 3)
        return;

    int nonVisiblePointsComputed = 0;

    // Go through points using a sliding windows of 3
    for (auto &&[i, group] : iter::enumerate(iter::sliding_window(path, 3)))
    {
        if (group.size() < 3)
            break; // break if too short

        //        if (group[0] == pathPoints[0])
        //            continue;

        auto p1 = QVector2D(group[0]);
        auto p2 = QVector2D(group[1]);
        auto p3 = QVector2D(group[2]);
        QPointF p = group[1];
        int index_of_p_in_path = i+1;  //index of p in path

        ////////////////////////////////
        /// INTERNAL curvature forces on p2. Stretches the path locally
        /// Approximates the angle between adjacent segments: p2->p1, p2->p3
        ////////////////////////////////
        QVector2D iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

        ////////////////////////////////////////////
        /// External forces caused by obstacles repulsion field
        ///////////////////////////////////////////7
        float min_dist;
        QVector2D eforce;

        qDebug() << __FUNCTION__  << nonVisiblePointsComputed;
        // compute forces from map on not visible points
        if ((isVisible(p, laser_poly ) == false))
        {
            auto [obstacleFound, vectorForce] = grid.vectorToClosestObstacle(p);
            if (( not obstacleFound) or (nonVisiblePointsComputed > 10))
            {
                qDebug ()  << __FUNCTION__ << "No obstacles found in map for not visible point or it is more than 10 not visible points away";
                nonVisiblePointsComputed++;
                continue;
            }
            else
            {
                qDebug()  << __FUNCTION__  << "--- Obstacle found in grid ---";
                min_dist = vectorForce.length() - (ROBOT_LENGTH / 2);   // subtract robot semi-width
                if (min_dist <= 0)    // hard limit to close obstables
                    min_dist = 0.01;
                eforce = vectorForce;
            }
            nonVisiblePointsComputed++;
        }
        // compute forces from laser on visible points
        else
        {
            // vector holding a) distance from laser tip to p, vector from laser tip to p, laser tip plane coordinates
            std::vector<std::tuple<float, QVector2D, QPointF>> distances;
            // Apply to all laser points a functor to compute the distances to point p2. laser_cart must be up to date
            std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, RL=ROBOT_LENGTH](const QPointF &laser)
                    {   // compute distance from laser measure to point minus RLENGTH/2 or 0 and keep it positive
                        float dist = (QVector2D(p) - QVector2D(laser)).length() - (RL / 2);
                        if (dist <= 0)
                            dist = 0.01;
                        return std::make_tuple(dist,  QVector2D(p)-QVector2D(laser), laser);
                    });
            // compute min of all laser to p distances
            auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b)
                    {
                        return std::get<float>(a) < std::get<float>(b);
                    });
            min_dist = std::get<float>(*min);
            eforce = std::get<QVector2D>(*min);
        }
        /// Note: instead of min, we could compute the resultant of all forces acting on the point, i.e. inside a given radius.
        /// a logarithmic law can be used to compute de force from the distance.
        /// To avoid constants, we need to compute de Jacobian of the sum of forces wrt the (x,y) coordinates of the point

        // rescale min_dist so 1 is ROBOT_LENGTH
        float magnitude = (1.f / ROBOT_LENGTH) * min_dist;
        // compute inverse square law
        magnitude = 10.f / (magnitude * magnitude);
        //if(magnitude > 25) magnitude = 25.;
        QVector2D f_force = magnitude * eforce.normalized();

        // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
        QVector2D base_line = (p1 - p3).normalized();
        const QVector2D itangential = QVector2D::dotProduct(f_force, base_line) * base_line;
        f_force = f_force - itangential;

        // update node pos. KI and KE are approximating inverse Jacobians modules. This should be CHANGED
        // Directions are taken as the vector going from p to closest obstacle.
        auto total = (KI * iforce) + (KE * f_force);
        //
        // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
        if (total.length() > 30)
            total = 8 * total.normalized();
        if (total.length() < -30)
            total = -8 * total.normalized();

        /// Compute additional restrictions to be forced in the minimization process
            // A) Check boundaries for final displacements
                // A.1) Move nodes only if it does not move inside objects
                // A.2) Does not move underneath the robot.
                // A.3) Does not exit the laser polygon
        QPointF temp_p = p + total.toPointF();
        qDebug()  << __FUNCTION__  << "Total force "<< total.toPointF()<< " New Point "<< temp_p;
        if (isPointVisitable(temp_p) and (not currentRobotPolygon.containsPoint(temp_p, Qt::OddEvenFill))
            //and (std::none_of(std::begin(intimateSpaces), std::end(intimateSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
            //and (std::none_of(std::begin(personalSpaces), std::end(personalSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
            )
        {
            path[index_of_p_in_path] = temp_p;
        }
//            if( auto it = find_if(pathPoints.begin(), pathPoints.end(), [p] (auto & s){ return (s.x() == p.x() and s.y() == p.y() );}); it != pathPoints.end())
//            {
//                int index = std::distance(pathPoints.begin(), it);
//                pathPoints[index] = temp_p;
//            }
    }
    // Check if robot nose is inside the laser polygon
    if(isVisible(currentRobotNose, laser_poly))
        pathPoints[0] = currentRobotNose;
    else
    {
        current_target.blocked.store(true);
        qWarning() << __FUNCTION__  << "Robot Nose not visible -- NEEDS REPLANNING ";
    }
}

template<typename TMap, typename TController>
bool Navigation<TMap, TController>::isPointVisitable(QPointF point)
{
    std::list<QPointF> path = grid.computePath(currentRobotNose, point);
    if (path.size() == 0)
    {
        qDebug()<< "Point not visitable -----";
        return false;
    }
    else
        return true;
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::addPoints(const QPolygonF &laser_poly)
{
    // qDebug()<<"Navigation - "<< __FUNCTION__;
    std::vector<std::tuple<int, QPointF>> points_to_insert;
    for (auto &&[k, group] : iter::enumerate(iter::sliding_window(pathPoints, 2)))
    {
        auto &p1 = group[0];
        auto &p2 = group[1];

        if (isVisible(p1, laser_poly) == false or isVisible(p2, laser_poly) == false) //not visible
            continue;

        float dist = QVector2D(p1 - p2).length();
        if (dist > ROAD_STEP_SEPARATION)
        {
            float l = 0.9 * ROAD_STEP_SEPARATION /
                      dist; //Crucial que el punto se ponga mas cerca que la condición de entrada
            QLineF line(p1, p2);
            points_to_insert.push_back(std::make_tuple(k + 1, QPointF{line.pointAt(l)}));
        }
    }
    for (const auto &[l, p] : iter::enumerate(points_to_insert))
    {
        if (!currentRobotPolygon.containsPoint(std::get<QPointF>(p), Qt::OddEvenFill))
            pathPoints.insert(pathPoints.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
    }
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::cleanPoints(const QPolygonF &laser_poly)
{
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

    std::vector<QPointF> points_to_remove;
    for (const auto &group : iter::sliding_window(pathPoints, 2))
    {
        const auto &p1 = group[0];
        const auto &p2 = group[1];

        if ((!isVisible(p1, laser_poly)) or (!isVisible(p2, laser_poly))) //not visible
            continue;

        if (p2 == lastPointInPath)
            break;
        // check if p1 was marked to erase in the previous iteration
        if (std::find(std::begin(points_to_remove), std::end(points_to_remove), p1) != std::end(points_to_remove))
            continue;

        float dist = QVector2D(p1 - p2).length();
        if (dist < 0.5 * ROAD_STEP_SEPARATION)
            points_to_remove.push_back(p2);

        else if(currentRobotPolygon.containsPoint(p2, Qt::OddEvenFill))
        {
            qDebug()<<"-------------" << __FUNCTION__ << "------------- Removing point inside robot ";
            points_to_remove.push_back(p2);
        }

    }


    for (auto &&p : points_to_remove)
    {
        pathPoints.erase(std::remove_if(pathPoints.begin(), pathPoints.end(), [p](auto &r) { return p == r; }), pathPoints.end());

    }
}

template<typename TMap, typename TController>
QPolygonF Navigation<TMap, TController>::getRobotPolygon()
{
    QPolygonF robotP;
    auto bLWorld = innerModel->transform("world", robotBottomLeft ,robot_name);
    auto bRWorld = innerModel->transform("world", robotBottomRight ,robot_name);
    auto tRWorld = innerModel->transform("world", robotTopRight ,robot_name);
    auto tLWorld = innerModel->transform("world", robotTopLeft ,robot_name);
    robotP << QPointF(bLWorld.value().x(),bLWorld.value().z());
    robotP << QPointF(bRWorld.value().x(),bRWorld.value().z());
    robotP << QPointF(tRWorld.value().x(),tRWorld.value().z());
    robotP << QPointF(tLWorld.value().x(),tLWorld.value().z());
    return robotP;
}

template<typename TMap, typename TController>
std::tuple<QPolygonF, std::vector<QPointF>> Navigation<TMap, TController>::updateLaserPolygon(const std::tuple<std::vector<float>, std::vector<float>> &lData)
{
    QPolygonF laser_poly;
    std::vector<QPointF> laser_cart;

    //laser_poly.clear(); //stores the points of the laser in laser reference system
    //laser_cart.clear();

    const auto &[angles, dists] = lData;
    for (const auto &[angle, dist] : iter::zip(angles, dists))
    {
        //convert laser polar coordinates to cartesian
        float x = dist*sin(angle); float z = dist*cos(angle);
        QVec laserWorld = innerModel->transform("world", QVec::vec3(x, 0, z), "laser").value(); //OJO CON LOS NOMBRES
        laser_poly << QPointF(x, z);
        laser_cart.push_back(QPointF(laserWorld.x(),laserWorld.z()));
    }
    return std::make_tuple(laser_poly, laser_cart);
}

template<typename TMap, typename TController>
QPointF Navigation<TMap, TController>::getRobotNose()
{
    //        qDebug()<<"Navigation - "<< __FUNCTION__;
    auto robot = QPointF(currentRobotPose.x(),currentRobotPose.z());

    //    return (robot + QPointF( (robotZLong/2 + 200) * sin(currentRobotPose.ry()), (robotZLong/2 + 200) * cos(currentRobotPose.ry())));
    return (robot + QPointF(250*sin(currentRobotPose.ry()),250*cos(currentRobotPose.ry())));

}


template<typename TMap, typename TController>
void Navigation<TMap, TController>::drawRoad(const QPolygonF &laser_poly)
{
    qDebug() << "Navigation - "<< __FUNCTION__;
    ///////////////////////
    // Preconditions
    ///////////////////////
    if (pathPoints.size() == 0)
        return;

    //clear previous points
    for (QGraphicsLineItem* item : scene_road_points)
        viewer_2d->removeItem((QGraphicsItem*)item);
    scene_road_points.clear();
//
    /// Draw all points
    QGraphicsLineItem *line1, *line2;
    std::string color;
    for (unsigned int i = 1; i < pathPoints.size(); i++)
    {
        QPointF &w = pathPoints[i];
        QPointF &wAnt = pathPoints[i - 1];
        if (w == wAnt) //avoid calculations on equal points
            continue;

        QLine2D li(QVec::vec2(wAnt.x(), wAnt.y()), QVec::vec2(w.x(), w.y()));
        QLine2D lp = li.getPerpendicularLineThroughPoint(QVec::vec2(w.x(), w.y()));
        QVec p1 = lp.pointAlongLineStartingAtP1AtLanda(QVec::vec2(w.x(),w.y()), 200);
        QVec p2 = lp.pointAlongLineStartingAtP1AtLanda(QVec::vec2(w.x(),w.y()), -200);
        QLineF qli(wAnt, w);
        QLineF qli_perp(p1.toQPointF(), p2.toQPointF());

        if(i == 1 or i == pathPoints.size()-1)
            color = "#FF0000"; //Red
        else
            if (isVisible(w, laser_poly))
                color = "#00FFF0"; //Blue
            else
                color = "#A200FF"; //Purple

        line1 = viewer_2d->addLine(qli, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
        line2 = viewer_2d->addLine(qli_perp, QPen(QBrush(QColor(QString::fromStdString(color))), 20));
        line1->setZValue(2000);
        line2->setZValue(2000);
        scene_road_points.push_back(line1);
        scene_road_points.push_back(line2);
    }
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::print_state( State state) const
{
    static State state_ant = State::DUMMY;
    if( state == state_ant)
    {
        state_ant = state;
        return;
    }
    switch (state)
    {
        case State::BLOCKED: qInfo() << "Nav state: BLOCKED"; break;
        case State::PATH_NOT_FOUND: qInfo() << "Nav state: PATH_NOT_FOUND"; break;
        case State::RUNNING: qInfo() << "Nav state: RUNNING"; break;
        case State::IDLE: qInfo() << "Nav state: IDLE"; break;
        case State::DUMMY: break;
        case State::AT_TARGET:
            auto p = QPointF(currentRobotPose.x(), currentRobotPose.z());
            qInfo() << "Nav state: *** TARGET ACHIEVED ***  Target -> " << current_target.p << " Current pose -> "
                    <<  p << "Error -> " << QVector2D(p - current_target.p).length();
            break;
    }
    state_ant = state;
}




//template<typename TMap, typename TController>
//void Navigation<TMap, TController>::drawRoad()
//{
    //        qDebug()<<"Navigation - "<< __FUNCTION__;
    /*
        ///////////////////////
        // Preconditions
        ///////////////////////
        if (pathPoints.size() == 0)
            return;


        try	{ viewer->removeNode("points");} catch(const QString &s){	qDebug() <<"drawRoad" <<s; };
        try	{ viewer->addTransform_ignoreExisting("points","world");} catch(const QString &s){qDebug()<<"drawRoad" << s; };

        try
        {
            ///////////////////
            //Draw all points
            //////////////////
            for (int i = 1; i < pathPoints.size(); i++)
            {
                QPointF &w = pathPoints[i];
                QPointF &wAnt = pathPoints[i - 1];
                if (w == wAnt) //avoid calculations on equal points
                    continue;

                QLine2D l(QVec::vec2(wAnt.x(),wAnt.y()), QVec::vec2(w.x(),w.y()));
                QLine2D lp = l.getPerpendicularLineThroughPoint(QVec::vec2(w.x(), w.y()));
                QVec normal = lp.getNormalForOSGLineDraw();  //3D vector
                QString item = "p_" + QString::number(i);
                viewer->addTransform_ignoreExisting(item, "points", QVec::vec6(w.x(), 10, w.y(), 0, 0, 0));


                if(i == 1)
                {
                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#FF0000");  //Rojo
                }
                else if (i == pathPoints.size()-1)
                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#FF0000");  //Rojo

                else if (isVisible(w))
                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#00FFF0");
                else
                    viewer->drawLine(item + "_point", item, QVec::zeros(3), normal, 500, 40, "#A200FF");  //Morado
            }
        }
        catch(const QString &s){qDebug()<<"drawRoad" << s;}
    //        qDebug()<<"END "<<__FUNCTION__;*/
//};

//ESPIRAL

//    for(auto &&i : iter::range(grid.size()))
//    {
//        while (2 * x_pos * d < m)
//        {
//            const auto &k = Grid<>::Key(x_pos, y_pos);
//            if (grid.isFree(k) and grid.isNearOccupied(k, target_name))
//                candidates.push_back(QVector2D(x_pos, y_pos));
//            x_pos = x_pos + d;
//        }
//        while (2 * y_pos * d < m)
//        {
//            const auto &k = Grid<>::Key(x_pos, y_pos);
//            if (grid.isFree(k) and grid.isNearOccupied(k, target_name))
//                candidates.push_back(QVector2D(x_pos, y_pos));
//            y_pos = y_pos + d;
//        }
//        d = -1 * d;
//        m = m + 1;
//    }


// search in a spiral pattern away from object for free spots near an occupied spot by target object
//    long int x_pos = target_center.x();
//    long int y_pos = target_center.y();
//    int d = dim.TILE_SIZE;
//    for(int i : iter::range(grid.size()))
//    {
//        while (2 * x_pos * d < i)
//        {
//            const auto &k = Grid<>::Key(x_pos, y_pos);
//            if (grid.isFree(k) and grid.cellNearToOccupiedCellByObject(k, target_name))
//                candidates.push_back(QVector2D(x_pos, y_pos));
//            x_pos = x_pos + d;
//        }
//        while (2 * y_pos * d < i)
//        {
//           const auto &k = Grid<>::Key(x_pos, y_pos);
//           if (grid.isFree(k) and grid.cellNearToOccupiedCellByObject(k, target_name))
//                candidates.push_back(QVector2D(x_pos, y_pos));
//            y_pos = y_pos + d;
//        }
//        d = -1 * d;
//        if(candidates.size() > 10)   // arbitrary number hard to fix
//            break;
//    }