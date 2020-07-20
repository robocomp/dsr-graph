#include "navigation.h"
#include <QGraphicsLineItem>

template<typename TMap, typename TController>
void Navigation<TMap, TController>::initialize( const std::shared_ptr<DSR::DSRGraph> &graph,
                 std::shared_ptr< RoboCompCommonBehavior::ParameterList > configparams_,
                 QGraphicsScene *scene,
                 std::string file_name)
{
    qDebug()<<"Navigation - "<< __FUNCTION__;
    G = graph;
    innerModel = G->get_inner_api();
    configparams = configparams_;
    viewer_2d = scene;

    stopRobot();
    //grid can't be initialized if the robot is moving
    collisions =  std::make_shared<Collisions>();
    collisions->initialize(G, configparams);
    grid.initialize(collisions, file_name);
    grid.draw(viewer_2d);
    controller.initialize(innerModel,configparams);

    robotXWidth = std::stof(configparams->at("RobotXWidth").value);
    robotZLong = std::stof(configparams->at("RobotZLong").value);
    robotBottomLeft     = QVec::vec3( - robotXWidth / 2 - 100, 0, - robotZLong / 2 - 100);
    robotBottomRight    = QVec::vec3( + robotXWidth / 2 + 100, 0, - robotZLong / 2 - 100);
    robotTopRight       = QVec::vec3( + robotXWidth / 2 + 100, 0, + robotZLong / 2 + 100);
    robotTopLeft        = QVec::vec3( - robotXWidth / 2 - 100, 0, + robotZLong / 2 + 100);

    //    reloj.restart();
};

template<typename TMap, typename TController>
void Navigation<TMap, TController>::update(const RoboCompLaser::TLaserData &laserData_, bool needsReplaning)
{
    qDebug() << "Navigation - " << __FUNCTION__;
    // static QTime reloj = QTime::currentTime();

    //    if (gridChanged)
    //    {
    //        updateFreeSpaceMap();
    //        gridChanged = false;
    //    }

    //TODO:
    //    currentRobotPose = innerModel->transformS6D("world","robot");
    //    qDebug()<< "Updated Robot pose " << reloj.restart();

    // if(needsReplaning)
    // {
    //     for (auto p: pathPoints)
    //     {
    //         if(std::any_of(std::begin(socialSpaces), std::end(socialSpaces),[p](const auto &poly) { return poly.containsPoint(p, Qt::OddEvenFill);})
    //         or std::any_of(std::begin(personalSpaces), std::end(personalSpaces),[p](const auto &poly) { return poly.containsPoint(p, Qt::OddEvenFill);})
    //         or std::any_of(std::begin(totalAffordances), std::end(totalAffordances),[p](const auto &poly) { return poly.containsPoint(p, Qt::OddEvenFill);})
    //        )
    //         {
    //             stopRobot();

    //             this->current_target.lock();
    //                 current_target.blocked.store(true);
    //             this->current_target.unlock();

    //             break;
    //         }
    //     }
    // }

    if (checkPathState() == false)
        return;

    computeForces(pathPoints, laserData_);
    cleanPoints();
    addPoints();
    auto [blocked, active, xVel,zVel,rotVel] = controller.update(pathPoints, laserData_, current_target.p, currentRobotPose);
    //    qDebug()<< "xVel "<<xVel << "zVel "<<zVel << "rotVel" << rotVel;

    //    if (blocked)
    //    {
    //        stopRobot();
    //
    //        this->current_target.lock();
    //            current_target.blocked.store(true);
    //        this->current_target.unlock();
    //    }
    //
    //    if (!active)
    //    {
    //        stopRobot();
    //        this->current_target.lock();
    //            current_target.active.store(false);
    //        this->current_target.unlock();
    //        pathPoints.clear();
    //        if(stopMovingRobot) moveRobot = false;
    //        if(robotAutoMov) newRandomTarget();
    //    }
    //
    //    if (!blocked and active)
    //    {
    // //       if(moveRobot) omnirobot_proxy->setSpeedBase(xVel,zVel,rotVel);
    //    }
    // //   drawRoad();
};

template<typename TMap, typename TController>
void Navigation<TMap, TController>::stopRobot()
{
    qDebug()<<"Navigation - "<< __FUNCTION__;
    //TODO
    //    omnirobot_proxy->setSpeedBase(0,0,0);
}

template<typename TMap, typename TController>
bool Navigation<TMap, TController>::isCurrentTargetActive()
{
    return current_target.active.load();
}

template<typename TMap, typename TController>
bool Navigation<TMap, TController>::checkPathState()
{
   //std::cout << __FUNCTION__ << current_target.active.load() << " " << current_target.blocked.load() << std::endl;
    if (current_target.active.load())
    {
        if (current_target.blocked.load())
        {
            if ( not findNewPath())
            {
                qDebug() << __FUNCTION__ << "Path not found";
                if(current_target.humanBlock.load()) //if the path is blocked by human the target is not deactivated
                    return false;
                qDebug()<< "checkPathState - Deactivating current target";
                stopRobot();
                current_target.active.store(false);
                pathPoints.clear();
                if(robotAutoMov)
                    newRandomTarget();
                return false;
            }
            else
            {
                qDebug()<< "checkPathState - Path found";
                this->current_target.lock();
                this->current_target.blocked.store(false);
                this->current_target.unlock();
                drawRoad();
                // reloj.restart();
            }
        }
        return true;
    }
    else
        return false;
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
    std::cout << __FUNCTION__  << " New Target arrived " << newT << std::endl;
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

////////// CONTROLLER RELATED METHODS //////////
template<typename TMap, typename TController>
RoboCompLaser::TLaserData Navigation<TMap, TController>::computeLaser(RoboCompLaser::TLaserData laserData)
{
    return RoboCompLaser::TLaserData();
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
        //printf("%.2f %.2f\n", (float)currentRobotNose.x(), (float)currentRobotNose.y());
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
bool Navigation<TMap, TController>::isVisible(QPointF p)
{
    std::optional<QVec> pointInLaser = innerModel->transform("laser", QVec::vec3(p.x(),0,p.y()),"world");
    return laser_poly.containsPoint(QPointF(pointInLaser.value().x(),pointInLaser.value().z()), Qt::OddEvenFill);
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::computeForces(const std::vector<QPointF> &path, const RoboCompLaser::TLaserData &lData)
{
    if (path.size() < 3) {
        return;
    }

    int pointIndex = 0;
    int nonVisiblePointsComputed = 0;

    // Go through points using a sliding windows of 3
    for (auto &group : iter::sliding_window(path, 3))
    {
        if (group.size() < 3)
            break; // break if too short


        //        if (group[0] == pathPoints[0])
        //            continue;

        auto p1 = QVector2D(group[0]);
        auto p2 = QVector2D(group[1]);
        auto p3 = QVector2D(group[2]);
        auto p = group[1];

        float min_dist;
        QVector2D force;

        qDebug()<< nonVisiblePointsComputed;
        if ((isVisible(p) == false))// if not visible (computed before) continue
        {

            auto [obstacleFound, vectorForce] = grid.vectorToClosestObstacle(p);

            if ((!obstacleFound) or (nonVisiblePointsComputed > 10))
            {
                qDebug () << "No obstacles found ";
                nonVisiblePointsComputed++;

                continue;
            }
            else
            {
                qDebug()<< "--- Obstacle found in grid ---";
                min_dist = vectorForce.length() - (ROBOT_LENGTH / 2);
                if (min_dist <= 0)
                    min_dist = 0.01;
                force = vectorForce;
            }
            nonVisiblePointsComputed++;
        }
        else
        {
            std::vector<std::tuple<float, QVector2D, QPointF>> distances;
            // Apply to all laser points a functor to compute the distances to point p2
            std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, this](QPointF &t) { //lasercart is updated in UpdateLaserPolygon
                // compute distante from laser tip to point minus RLENGTH/2 or 0 and keep it positive
                float dist = (QVector2D(p) - QVector2D(t)).length() - (ROBOT_LENGTH / 2);
                if (dist <= 0)
                    dist = 0.01;
                return std::make_tuple(dist,  QVector2D(p)-QVector2D(t), t);
            });

            // compute min distance
            auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b) {
                return std::get<float>(a) < std::get<float>(b);
            });
            min_dist = std::get<float>(*min);
            //QPointF min_angle = std::get<QPointF>(*min);
            //          qDebug()<< "Point "<< p << " --  min dist " << min_dist << "--- min angle "<< min_angle;
            force = std::get<QVector2D>(*min);
        }


        // INTERNAL curvature forces on p2
        QVector2D iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

        // EXTERNAL forces. We need the minimun distance from each point to the obstacle(s). we compute the shortest laser ray to each point in the path
        // compute minimun distances to each point within the laser field

        // rescale min_dist so 1 is ROBOT_LENGTH
        float magnitude = (1.f / ROBOT_LENGTH) * min_dist;
        // compute inverse square law
        magnitude = 10.f / (magnitude * magnitude);
        //if(magnitude > 25) magnitude = 25.;
        QVector2D f_force = magnitude * force.normalized();
        //qDebug() << magnitude << f_force;

        // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
        QVector2D base_line = (p1 - p3).normalized();
        const QVector2D itangential = QVector2D::dotProduct(f_force, base_line) * base_line;
        f_force = f_force - itangential;

        //        qDebug()<< "[NAVIGATION]"<< __FUNCTION__<< " --- i force " << iforce << "f force "<< f_force;
        // update node pos
        auto total = (KI * iforce) + (KE * f_force);
        //
        // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
        if (total.length() > 30)
            total = 8 * total.normalized();
        if (total.length() < -30)
            total = -8 * total.normalized();

        //        qDebug()<< "[NAVIGATION]"<< __FUNCTION__<< "---total forces = " << total;
        // move node only if they do not exit the laser polygon and do not get inside objects or underneath the robot.
        QPointF temp_p = p + total.toPointF();

        qDebug() << "Total force "<< total.toPointF()<< " New Point "<< temp_p;

        //        if (isVisible(temp_p)
        if (isPointVisitable(temp_p)
            and (!currentRobotPolygon.containsPoint(temp_p, Qt::OddEvenFill))
            //and (std::none_of(std::begin(intimateSpaces), std::end(intimateSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
            //and (std::none_of(std::begin(personalSpaces), std::end(personalSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
                )
        {

            auto it = find_if(pathPoints.begin(), pathPoints.end(), [p] (auto & s) {
                return (s.x() == p.x() and s.y() == p.y() );
            } );

            if (it != pathPoints.end())
            {
                int index = std::distance(pathPoints.begin(), it);
                pathPoints[index] = temp_p;
            }
        }
        pointIndex++;
    }

    if(isVisible(currentRobotNose))
    {
        pathPoints[0] = currentRobotNose;
        drawRoad();
    }
    else
    {
        this->current_target.lock();
        current_target.blocked.store(true);
        this->current_target.unlock();

        qDebug()<< "Robot Nose not visible -- NEEDS REPLANNING ";
    }

    FILE *fd1 = fopen("calculatedPoints.txt", "w");

    for (const QPointF &p : pathPoints)
    {
        fprintf(fd1, "%.2f %.2f\n", (float)p.x(), (float)p.y());
    }

    fclose(fd1);

    qDebug()<< endl;
    qDebug()<< endl;
    return;
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
void Navigation<TMap, TController>::addPoints()
{
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

    std::vector<std::tuple<int, QPointF>> points_to_insert;
    for (auto &&[k, group] : iter::enumerate(iter::sliding_window(pathPoints, 2)))
    {
        auto &p1 = group[0];
        auto &p2 = group[1];

        if (isVisible(p1) == false or isVisible(p2) == false) //not visible
            continue;

        float dist = QVector2D(p1 - p2).length();

        if (dist > ROAD_STEP_SEPARATION)
        {
            float l = 0.9 * ROAD_STEP_SEPARATION / dist; //Crucial que el punto se ponga mas cerca que la condición de entrada
            QLineF line(p1, p2);
            points_to_insert.push_back(std::make_tuple(k + 1, QPointF{line.pointAt(l)}));
        }
        //qDebug() << __FUNCTION__ << k;
    }
    for (const auto &[l, p] : iter::enumerate(points_to_insert))
    {
        if(!currentRobotPolygon.containsPoint(std::get<QPointF>(p), Qt::OddEvenFill))
        {
            //                qDebug()<< "Add points  " << std::get<QPointF>(p);

            pathPoints.insert(pathPoints.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
        }

    }
    //        qDebug() << __FUNCTION__ << "points inserted " << points_to_insert.size();
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::cleanPoints()
{
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

    std::vector<QPointF> points_to_remove;
    for (const auto &group : iter::sliding_window(pathPoints, 2))
    {
        const auto &p1 = group[0];
        const auto &p2 = group[1];

        if ((!isVisible(p1)) or (!isVisible(p2))) //not visible
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
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

    QPolygonF robotP;

    auto bLWorld = innerModel->transform ("world", robotBottomLeft ,"base_mesh");
    auto bRWorld = innerModel->transform ("world", robotBottomRight ,"base_mesh");
    auto tRWorld = innerModel->transform ("world", robotTopRight ,"base_mesh");
    auto tLWorld = innerModel->transform ("world", robotTopLeft ,"base_mesh");


    robotP << QPointF(bLWorld.value().x(),bLWorld.value().z());
    robotP << QPointF(bRWorld.value().x(),bRWorld.value().z());
    robotP << QPointF(tRWorld.value().x(),tRWorld.value().z());
    robotP << QPointF(tLWorld.value().x(),tLWorld.value().z());

    FILE *fd = fopen("robot.txt", "w");
    for (const auto &r: robotP)
    {
        fprintf(fd, "%d %d\n", (int)r.x(), (int)r.y());
    }

    fprintf(fd, "%d %d\n", (int)robotP[0].x(), (int)robotP[0].y());

    fclose(fd);
    return robotP;
}

template<typename TMap, typename TController>
void Navigation<TMap, TController>::updateLaserPolygon(const RoboCompLaser::TLaserData &lData)
{
    //        qDebug()<<"Navigation - "<< __FUNCTION__;

    laser_poly.clear(); //stores the points of the laser in lasers refrence system
    laser_cart.clear();
    /*   auto lasernode = innerModel->getNode<InnerModelLaser>(QString("laser"));

       for (const auto &l : lData)
       {
           //convert laser polar coordinates to cartesian
           QVec laserc = lasernode->laserTo(QString("laser"),l.dist, l.angle);
           QVec laserWord = lasernode->laserTo(QString("world"),l.dist, l.angle);
   //        QVec laserWorld = innerModel->transform("world",QVec::vec3(laserc.x(),0,laserc.y()),"laser");

           laser_poly << QPointF(laserc.x(),laserc.z());

           laser_cart.push_back(QPointF(laserWord.x(),laserWord.z()));

       }


       FILE *fd = fopen("laserPoly.txt", "w");
       for (const auto &lp : laser_poly)
       {
           QVec p = innerModel->transform("world",QVec::vec3(lp.x(),0,lp.y()),"laser");
           fprintf(fd, "%d %d\n", (int)p.x(), (int)p.z());
       }
       fclose(fd);
   */

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
void Navigation<TMap, TController>::drawRoad()
{
    qDebug()<<"Navigation - "<< __FUNCTION__;
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

        QLine2D li(QVec::vec2(wAnt.x(),wAnt.y()), QVec::vec2(w.x(),w.y()));
        QLine2D lp = li.getPerpendicularLineThroughPoint(QVec::vec2(w.x(), w.y()));
        QVec p1 = lp.pointAlongLineStartingAtP1AtLanda(QVec::vec2(w.x(),w.y()), 200);
        QVec p2 = lp.pointAlongLineStartingAtP1AtLanda(QVec::vec2(w.x(),w.y()), -200);
        QLineF qli(wAnt, w);
        QLineF qli_perp(p1.toQPointF(), p2.toQPointF());

        if(i == 1 or i == pathPoints.size()-1)
            color = "#FF0000"; //Red
        else
            if (isVisible(w))
                color = "#00FFF0";
            else
                color = "#A200FF";

        line1 = viewer_2d->addLine(qli, QPen(QString::fromStdString(color)));
        line2 = viewer_2d->addLine(qli_perp, QPen(QString::fromStdString(color)));
        line1->setZValue(2000);
        line2->setZValue(2000);
        scene_road_points.push_back(line1);
        scene_road_points.push_back(line2);
    }
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

