//
// Created by pbustos on 20/7/20.
//
#include "collisions.h"

void Collisions::initialize(const std::shared_ptr<DSR::DSRGraph> &graph_,const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &params_)
{
    qDebug() << "Collisions - " <<__FUNCTION__;
    G = graph_;
    //read from World (DSR node)
    std::optional<Node> world_node = G->get_node(world_name);
    if(world_node.has_value())
    {
        outerRegion.setLeft(G->get_attrib_by_name<OuterRegionLeft_att>(world_node.value()).value());
        outerRegion.setRight(G->get_attrib_by_name<OuterRegionRight_att>(world_node.value()).value());
        outerRegion.setBottom(G->get_attrib_by_name<OuterRegionBottom_att>(world_node.value()).value());
        outerRegion.setTop(G->get_attrib_by_name<OuterRegionTop_att>(world_node.value()).value());
    }
    if(outerRegion.isNull())
    {
        qDebug()<<"[ERROR] OUTER REGION IS NULL";
        std::terminate();
    }
    std::cout << __FILE__ << __FUNCTION__ << std::endl;
    QStringList ls = QString::fromStdString(params_->at("excluded_objects_in_collision_check").value).replace(" ", "" ).split(',');
    std::cout << __FILE__ << __FUNCTION__ << " " << ls.size() << "objects read for exclusion list" << std::endl;
    foreach(const QString &s, ls)
        excludedNodes.insert(s.toStdString());

    // Compute the list of meshes that correspond to robot, world and possibly some additionally excluded ones
    robotNodes.clear(); restNodes.clear();
    recursiveIncludeMeshes(G->get_node_root().value(), robot_name, false, robotNodes, restNodes, excludedNodes);
//    std::cout << __FUNCTION__ << " RESTNODES:" << std::endl;
//    for(auto n : restNodes)
//        std::cout << n << std::endl;
//    std::cout << __FUNCTION__ << " ROBOT:" ;
//    for(auto r : robotNodes)
//        std::cout << r << std::endl;
//    std::cout << __FUNCTION__ << " EXCLUDED:" ;
//    for(auto r : excludedNodes)
//        std::cout << r << std::endl;
    qsrand( QTime::currentTime().msec() );

}

std::tuple<bool, std::string> Collisions::checkRobotValidStateAtTargetFast(DSR::DSRGraph *G_copy, const std::vector<float> &targetPos, const std::vector<float> &targetRot)
{
    //First we move the robot in G_copy to the target coordinates
    std::optional<Node> world = G_copy->get_node(world_name);
    std::optional<int> robot_id = G_copy->get_id_from_name(robot_name);
    std::unique_ptr<RT_API> rt = G_copy->get_rt_api();
    rt->insert_or_assign_edge_RT(world.value(), robot_id.value(), targetPos, targetRot);
    std::shared_ptr<DSR::InnerEigenAPI> inner_eigen = G_copy->get_inner_eigen_api();

    //// Check if the robot at the target collides with any object in restNodes
    bool collision = false;
    for ( const std::string& in : robotNodes )
        for ( const std::string&  out : restNodes )
        {
            try
            {
                collision = collide(inner_eigen, in, out);
            }
            catch (QString &s)
            {
                std::cout << __FUNCTION__ << " " << s.toStdString() << " between " << in << " and " << out << std::endl;
                qFatal("Collision");
            }
            if (collision)
            {
                std::cout << "COLLISION: " << in << " to " << out << " pos: (" << targetPos[0] << "," << targetPos[1]
                          << "," << targetPos[2] << ")" << std::endl;
                return std::make_tuple(false, out);
            }
        }
    return std::make_tuple(true, "");;
}

bool Collisions::collide(std::shared_ptr<DSR::InnerEigenAPI> inner_eigen, const std::string &node_a_name, const std::string &node_b_name)
{
    //std::cout << "collide " << node_a_name << " to "<< node_b_name << std::endl;
    Mat::RTMat r1q = inner_eigen->get_transformation_matrix(world_name, node_a_name).value();
    fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
    fcl::Vec3f T1( r1q(0,3), r1q(1,3), r1q(2,3) );
    Mat::RTMat r2q = inner_eigen->get_transformation_matrix(world_name, node_b_name).value();
    fcl::Matrix3f R2( r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2) );
    fcl::Vec3f T2( r2q(0,3), r2q(1,3), r2q(2,3) );

    fcl::CollisionRequest request;
    fcl::CollisionResult result;
    fcl::CollisionObject* n1 = get_collision_object(node_a_name);
    fcl::CollisionObject* n2 = get_collision_object(node_b_name);

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
        return false;
}

void Collisions::recursiveIncludeMeshes(Node node, std::string robot_name, bool inside, std::vector<std::string> &in, std::vector<std::string> &out, std::set<std::string> &excluded)
{
    if (node.name() == robot_name)
        inside = true;
    if (node.type() == "mesh" or node.type() == "plane")
    {
        if (std::find(excluded.begin(), excluded.end(), node.name()) == excluded.end())  //not found in excluded
        {
            if (inside)
                in.push_back(node.name());
            else
                out.push_back(node.name());
        }
    }
    for(auto &edge: G->get_node_edges_by_type(node, "RT"))
    {
        auto child = G->get_node(edge.to());
        recursiveIncludeMeshes(child.value(), robot_name, inside, in, out, excluded);
    }
}

//returns collison object, creates it if does not exist
fcl::CollisionObject* Collisions::get_collision_object(const std::string& node_name)
{
    if (collision_objects.find(node_name) == collision_objects.end())
    {
        // object creation
        std::optional<Node> node = G->get_node(node_name);
        if (node.has_value())
        {
            if( node.value().type() == "plane" )
                collision_objects[node_name] = create_plane_collision_object(node.value());
            else
            {
                if( node.value().type() == "mesh")
                    collision_objects[node_name] = create_mesh_collision_object(node.value());
                else
                    collision_objects[node_name] = nullptr;
            }
        }
    }
    return collision_objects[node_name];
}

fcl::CollisionObject* Collisions::create_mesh_collision_object(const Node &node)
{
    fcl::CollisionObject* collision_object = nullptr;
    std::optional<std::string> meshPath = G->get_attrib_by_name<path_att>(node);
    osg::ref_ptr<osg::Node> osgnode_ = osgDB::readNodeFile(meshPath.value());
    if (osgnode_ != NULL)
    {
        std::vector<fcl::Vec3f> vertices;
        std::vector<fcl::Triangle> triangles;
        CalculateTriangles calcTriangles(&vertices, &triangles);
        osgnode_->accept(calcTriangles);

        // Transform each of the read vertices
        std::optional<int> scalex = G->get_attrib_by_name<scalex_att>(node);
        std::optional<int> scaley = G->get_attrib_by_name<scaley_att>(node);
        std::optional<int> scalez = G->get_attrib_by_name<scalez_att>(node);
        if(not (scalex.has_value() and scaley.has_value() and scalez.has_value()))
        {
            qWarning() << __FUNCTION__ << "scale attributes not found in object " << QString::fromStdString(node.name()) << " returning nullptr";
            return collision_object;
        }
        for(auto &v : vertices)
        {
            v[0] *= scalex.value();
            v[1] *= scaley.value();
            v[2] *= -scalez.value();
        }

        // Associate the vertices and triangles vectors to the FCL collision model object
        FCLModelPtr fclMesh = FCLModelPtr(new FCLModel());
        fclMesh->beginModel();
            fclMesh->addSubModel(vertices, triangles);
        fclMesh->endModel();
        collision_object = new fcl::CollisionObject(fclMesh);
    }
    return collision_object;
}

fcl::CollisionObject* Collisions::create_plane_collision_object(const Node &node)
{
    fcl::CollisionObject* collision_object = nullptr;
    std::optional<int> width = G->get_attrib_by_name<width_att>(node);
    std::optional<int> height = G->get_attrib_by_name<height_att>(node);
    std::optional<int> depth = G->get_attrib_by_name<depth_att>(node);
    if(not (width.has_value() and height.has_value() and depth.has_value()))
    {
        qWarning() << __FUNCTION__ << "size attributes not found in object " << QString::fromStdString(node.name()) << " returning nullptr";
        return collision_object;
    }
    std::vector<fcl::Vec3f> vertices;
    vertices.push_back(fcl::Vec3f(-width.value()/2., +height.value()/2., -depth.value()/2.)); // Front NW
    vertices.push_back(fcl::Vec3f(+width.value()/2., +height.value()/2., -depth.value()/2.)); // Front NE
    vertices.push_back(fcl::Vec3f(-width.value()/2., -height.value()/2., -depth.value()/2.)); // Front SW
    vertices.push_back(fcl::Vec3f(+width.value()/2., -height.value()/2., -depth.value()/2.)); // Front SE
    vertices.push_back(fcl::Vec3f(-width.value()/2., +height.value()/2., +depth.value()/2.)); // Back NW
    vertices.push_back(fcl::Vec3f(+width.value()/2., +height.value()/2., +depth.value()/2.)); // Back NE
    vertices.push_back(fcl::Vec3f(-width.value()/2., -height.value()/2., +depth.value()/2.)); // Back SW
    vertices.push_back(fcl::Vec3f(+width.value()/2., -height.value()/2., +depth.value()/2.)); // Back SE

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
