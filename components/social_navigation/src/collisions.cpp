//
// Created by pbustos on 20/7/20.
//
#include "collisions.h"

void Collisions::initialize(const std::shared_ptr<DSR::DSRGraph> &graph_,const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &params_)
{
    qDebug()<<"Collisions - " <<__FUNCTION__;
    G = graph_;
    //read from World (DSR node)
    std::optional<Node> world_node = G->get_node("world");
    if(world_node.has_value())
    {
        outerRegion.setLeft(G->get_attrib_by_name<int>(world_node.value(), "OuterRegionLeft").value());
        outerRegion.setRight(G->get_attrib_by_name<int>(world_node.value(), "OuterRegionRight").value());
        outerRegion.setBottom(G->get_attrib_by_name<int>(world_node.value(), "OuterRegionBottom").value());
        outerRegion.setTop(G->get_attrib_by_name<int>(world_node.value(), "OuterRegionTop").value());
    }
    robot_name = params_->at("RobotName").value;
    if(outerRegion.isNull())
    {
        qDebug()<<"[ERROR] OUTER REGION IS NULL";
    }
    QStringList ls = QString::fromStdString(params_->at("ExcludedObjectsInCollisionCheck").value).replace(" ", "" ).split(',');
    std::cout << __FILE__ << __FUNCTION__ << " " << ls.size() << "objects read for exclusion list" << std::endl;
    foreach(const QString &s, ls)
        excludedNodes.insert(s.toStdString());

    // Compute the list of meshes that correspond to robot, world and possibly some additionally excluded ones
    robotNodes.clear(); restNodes.clear();
    recursiveIncludeMeshes(G->get_node_root().value(), robot_name, false, robotNodes, restNodes, excludedNodes);
    std::cout<<"lists"<<std::endl;
    std::cout<<"robot"<<robotNodes<<std::endl;
    std::cout<<"rest"<<restNodes<<std::endl;
    qsrand( QTime::currentTime().msec() );
}

bool Collisions::checkRobotValidStateAtTargetFast(DSR::DSRGraph& G_copy, const std::vector<float> &targetPos, const std::vector<float> &targetRot)
{
    //First we move the robot in our copy of innermodel to its current coordinates
    std::optional<Node> world = G_copy.get_node("world");
    std::optional<Edge> edge = G_copy.get_edge("world", robot_name, "RT");
    G_copy.modify_attrib_local(edge.value(), "translation", targetPos);
    G_copy.modify_attrib_local(edge.value(), "rotation_euler_xyz", targetRot);
    G_copy.update_node(world.value());
    std::shared_ptr<DSR::InnerAPI> inner = G_copy.get_inner_api();

    G_copy.get_edge_RT_as_RTMat(edge.value()).print("rt_orig");
    std::optional<Edge> e = G_copy.get_edge("world", robot_name, "RT");
    G_copy.get_edge_RT_as_RTMat(e.value()).print("rt");

    ///////////////////////
    //// Check if the robot at the target collides with any know object
    ///////////////////////
    bool collision = false;
    for ( const std::string& in : robotNodes )
        for ( const std::string&  out : restNodes )
        {
            try
            {
                collision = collide(inner, in, out);
            }
            catch (QString &s)
            {
                qDebug() << __FUNCTION__ << s;
                qFatal("Collision");
            }
            if (collision)
            {
                std::cout << "COLLISION: " << in << " to " << out << " pos: (" << targetPos[0] << "," << targetPos[1]
                          << "," << targetPos[2] << ")" << std::endl;
                return false;
            }
        }
    return true;
}

bool Collisions::collide(std::shared_ptr<DSR::InnerAPI> inner, const std::string &node_a_name, const std::string &node_b_name)
{
    //std::cout << "collide " << node_a_name << " to "<< node_b_name << std::endl;
    QMat r1q = inner->getTransformationMatrixS("world", node_a_name).value();
    fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
    fcl::Vec3f T1( r1q(0,3), r1q(1,3), r1q(2,3) );

    QMat r2q = inner->getTransformationMatrixS("world", node_b_name).value();
    fcl::Matrix3f R2( r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2) );
    fcl::Vec3f T2( r2q(0,3), r2q(1,3), r2q(2,3) );

    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    fcl::CollisionObject* n1 = get_collision_object(inner, node_a_name);
    fcl::CollisionObject* n2 = get_collision_object(inner, node_b_name);

    if (n1 != nullptr and n2 != nullptr)
    {
        n1->setTransform(R1, T1);
        n1->computeAABB();
        n2->setTransform(R2, T2);
        n2->computeAABB();
        fcl::collide(n1, n2, request, result);
        return result.isCollision();
    }
    else
    {
        return false;
    }
}

void Collisions::recursiveIncludeMeshes(Node node, std::string robot_name, bool inside, std::vector<std::string> &in, std::vector<std::string> &out, std::set<std::string> &excluded)
{
    if (node.name() == robot_name)
    {
        inside = true;
    }
    if (node.type() == "mesh" or node.type() == "plane")
    {
        if( std::find(excluded.begin(), excluded.end(), node.name()) == excluded.end() )
        {
            if (inside)
            {
                in.push_back(node.name());
            }
            else
            {
                out.push_back(node.name());
            }
        }
    }

    for(auto &edge: G->get_node_edges_by_type(node, "RT"))
    {
        auto child = G->get_node(edge.to());
        recursiveIncludeMeshes(child.value(), robot_name, inside, in, out, excluded);
    }
}


//return collison object, creates it if does not exist
fcl::CollisionObject* Collisions::get_collision_object(std::shared_ptr<DSR::InnerAPI> inner, std::string node_name)
{
    if (collision_objects.find(node_name) == collision_objects.end())
    {
        // object creation
        std::optional<Node> node = G->get_node(node_name);
        if (node.has_value())
        {
            if( node.value().type() == "plane" )
            {
                collision_objects[node_name] = create_plane_collision_object(inner, node.value());
            }
            else
            {
                if( node.value().type() == "mesh")
                {
                    collision_objects[node_name] = create_mesh_collision_object(inner, node.value());
                }
                else
                {
                    collision_objects[node_name] = nullptr;
                }
            }
        }
    }
    return collision_objects[node_name];
}

fcl::CollisionObject* Collisions::create_mesh_collision_object(std::shared_ptr<DSR::InnerAPI> inner, const Node &node)
{
    fcl::CollisionObject* collision_object = nullptr;
    std::optional<std::string> meshPath = G->get_attrib_by_name<std::string>(node, "path");
    osg::ref_ptr<osg::Node> osgnode_ = osgDB::readNodeFile(meshPath.value());
    if (osgnode_ != NULL)
    {
        std::vector<fcl::Vec3f> vertices;
        std::vector<fcl::Triangle> triangles;
        CalculateTriangles calcTriangles(&vertices, &triangles);
        osgnode_->accept(calcTriangles);
        // Get the internal transformation matrix of the mesh
        std::optional<Node> parent = G->get_parent_node(node);
        if(not parent.has_value())
            return collision_object;
        std::optional<QVec> pose = inner->transformS6D(node.name(), parent.value().name());
        RTMat rtm(pose.value().rx(), pose.value().ry(), pose.value().rz(), pose.value().x(), pose.value().y(), pose.value().z());
        // Transform each of the read vertices
        std::optional<int> scalex = G->get_attrib_by_name<int>(node, "scalex");
        std::optional<int> scaley = G->get_attrib_by_name<int>(node, "scaley");
        std::optional<int> scalez = G->get_attrib_by_name<int>(node, "scalez");
        if(not (scalex.has_value() and scaley.has_value() and scalez.has_value()))
            return collision_object;
        for (size_t i=0; i<vertices.size(); i++)
        {
            fcl::Vec3f v = vertices[i];
            const QMat v2 = (rtm * QVec::vec3(v[0]*scalex.value(), v[1]*scaley.value(), -v[2]*scalez.value()).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
            vertices[i] = fcl::Vec3f(v2(0), v2(1), v2(2));
        }
        // Associate the read vertices and triangles vectors to the FCL collision model object
        FCLModelPtr fclMesh = FCLModelPtr(new FCLModel());
        fclMesh->beginModel();
        fclMesh->addSubModel(vertices, triangles);
        fclMesh->endModel();
        collision_object = new fcl::CollisionObject(fclMesh);
    }
    return collision_object;
}

fcl::CollisionObject* Collisions::create_plane_collision_object(std::shared_ptr<DSR::InnerAPI> inner, const Node &node)
{
    fcl::CollisionObject* collision_object = nullptr;
    std::optional<int> width = G->get_attrib_by_name<int>(node, "width");
    std::optional<int> height = G->get_attrib_by_name<int>(node, "height");
    std::optional<int> depth = G->get_attrib_by_name<int>(node, "depth");
    if(not (width.has_value() and height.has_value() and depth.has_value()))
        return collision_object;
    std::vector<fcl::Vec3f> vertices;
    vertices.push_back(fcl::Vec3f(-width.value()/2., +height.value()/2., -depth.value()/2.)); // Front NW
    vertices.push_back(fcl::Vec3f(+width.value()/2., +height.value()/2., -depth.value()/2.)); // Front NE
    vertices.push_back(fcl::Vec3f(-width.value()/2., -height.value()/2., -depth.value()/2.)); // Front SW
    vertices.push_back(fcl::Vec3f(+width.value()/2., -height.value()/2., -depth.value()/2.)); // Front SE
    vertices.push_back(fcl::Vec3f(-width.value()/2., +height.value()/2., +depth.value()/2.)); // Back NW
    vertices.push_back(fcl::Vec3f(+width.value()/2., +height.value()/2., +depth.value()/2.)); // Back NE
    vertices.push_back(fcl::Vec3f(-width.value()/2., -height.value()/2., +depth.value()/2.)); // Back SW
    vertices.push_back(fcl::Vec3f(+width.value()/2., -height.value()/2., +depth.value()/2.)); // Back SE

    std::optional<Node> parent = G->get_parent_node(node);
    if(not parent.has_value())
        return collision_object;
    std::optional<QVec> pose = inner->transformS6D(node.name(), parent.value().name());
    osg::Matrix r;
    r.makeRotate(osg::Vec3(0, 0, 1), osg::Vec3(pose.value().rx(), pose.value().ry(), -pose.value().rz()));
    QMat qmatmat(4,4);
    for (int rro=0; rro<4; rro++)
    {
        for (int cco=0; cco<4; cco++)
        {
            qmatmat(rro,cco) = r(rro,cco);
        }
    }

    for (size_t i=0; i<vertices.size(); i++)
    {
        fcl::Vec3f v = vertices[i];
        const QVec rotated = (qmatmat*(QVec::vec3(v[0], v[1], v[2]).toHomogeneousCoordinates())).fromHomogeneousCoordinates();
        vertices[i] = fcl::Vec3f(rotated(0)+pose.value().x(), rotated(1)+pose.value().y(), rotated(2)+pose.value().z());
    }

    std::vector<fcl::Triangle> triangles;
    triangles.push_back(fcl::Triangle(0,1,2)); // Front
    triangles.push_back(fcl::Triangle(1,2,3));
    triangles.push_back(fcl::Triangle(4,5,6)); // Back
    triangles.push_back(fcl::Triangle(5,6,7));
    triangles.push_back(fcl::Triangle(4,0,6)); // Left
    triangles.push_back(fcl::Triangle(0,6,2));
    triangles.push_back(fcl::Triangle(5,1,7)); // Right
    triangles.push_back(fcl::Triangle(1,7,3));
    triangles.push_back(fcl::Triangle(5,1,4)); // Top
    triangles.push_back(fcl::Triangle(1,4,0));
    triangles.push_back(fcl::Triangle(2,3,6)); // Bottom
    triangles.push_back(fcl::Triangle(3,6,7));

    FCLModelPtr fclMesh = FCLModelPtr(new FCLModel());
    fclMesh->beginModel();
    fclMesh->addSubModel(vertices, triangles);
    fclMesh->endModel();

    collision_object = new fcl::CollisionObject(fclMesh);

    return collision_object;
}
