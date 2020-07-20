//
// Created by robolab on 13/01/20.
//

#ifndef COLLISIONS_H
#define COLLISIONS_H

#include "dsr/api/dsr_api.h"
#include <dsr/api/dsr_inner_api.h>
#include <CommonBehavior.h>

#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/ccd/motion.h>
#include <fcl/BV/BV.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/traversal/traversal_node_bvh_shape.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <osg/TriangleFunctor>
#include <osg/io_utils>
#include <osg/Geode>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
typedef fcl::BVHModel<fcl::OBBRSS> FCLModel;
typedef std::shared_ptr<FCLModel> FCLModelPtr;


struct IncludeTrianglesInFCL_functor
{
	IncludeTrianglesInFCL_functor()
	{
		vertices = NULL;
		triangles = NULL;
	}

	std::vector<fcl::Vec3f> *vertices;
	std::vector<fcl::Triangle> *triangles;
	osg::Matrix tm;

	void set(std::vector<fcl::Vec3f> *vertices_, std::vector<fcl::Triangle> *triangles_, osg::Matrix transformMatrix)
	{
		vertices = vertices_;
		triangles = triangles_;
		tm = transformMatrix;
	}
	void clear()
	{
		if (vertices == NULL or triangles == NULL)
		{
			fprintf(stderr, "IncludeTrianglesInFCL_functor not initialized!\n");
			throw "IncludeTrianglesInFCL_functor not initialized!";
		}
		vertices->clear();
		triangles->clear();
	}
	void operator() (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool /* treatVertexDataAsTemporary */)
	{
		if (vertices == NULL or triangles == NULL)
		{
			fprintf(stderr, "IncludeTrianglesInFCL_functor not initialized!\n");
			throw "IncludeTrianglesInFCL_functor not initialized!";
		}
		osg::Vec3 v1p = tm * v1;
		osg::Vec3 v2p = tm * v2;
		osg::Vec3 v3p = tm * v3;
		vertices->push_back(fcl::Vec3f(v1p.x(), v1p.y(), v1p.z()));
		vertices->push_back(fcl::Vec3f(v2p.x(), v2p.y(), v2p.z()));
		vertices->push_back(fcl::Vec3f(v3p.x(), v3p.y(), v3p.z()));
		triangles->push_back(fcl::Triangle(vertices->size()-3, vertices->size()-2, vertices->size()-1));
	}
};

class CalculateTriangles : public osg::NodeVisitor
{
	public:
		CalculateTriangles(std::vector<fcl::Vec3f> *vertices_, std::vector<fcl::Triangle> *triangles_) : NodeVisitor( NodeVisitor::TRAVERSE_ALL_CHILDREN )
		{
			vertices = vertices_;
			triangles = triangles_;
			transformMatrix.makeIdentity();
		}

		virtual void apply(osg::Geode &geode)
		{
			// Use an OSG triangle functor to gather the vertices and triangles
			std::vector<fcl::Vec3f> vs;
			std::vector<fcl::Triangle> ts;
			osg::TriangleFunctor<IncludeTrianglesInFCL_functor> tri;
			tri.set(&vs, &ts, transformMatrix);
			tri.clear();
			int D = geode.getNumDrawables();
			for (int d=0; d<D; d++)
			{
				geode.getDrawable(d)->accept(tri);
			}
			// Append new points
			vertices->insert(vertices->end(), vs.begin(), vs.end());
			for (uint t=0; t<ts.size(); t++)
			{
				ts[t].set(ts[t][0]+triangles->size(), ts[t][1]+triangles->size(), ts[t][2]+triangles->size());
			}
			triangles->insert(triangles->end(), ts.begin(), ts.end());
			// recursion
			traverse( geode );
		}

		virtual void apply(osg::MatrixTransform &node)
		{
			// update matrix
			transformMatrix *= node.getMatrix();
			// recursion
			traverse( node );
		}
	protected:
		// Pointers that we will be given and that we have to fill with the output data
		std::vector<fcl::Vec3f> *vertices;
		std::vector<fcl::Triangle> *triangles;
		// Transformation matrix
		osg::Matrix transformMatrix;
};

 //TODO: update objects if node parameters values changed
 //Also update on new/delete/changed nodes
class Collisions {

public:
    std::vector<std::string> robotNodes;
    std::vector<std::string> restNodes;
    std::set<std::string> excludedNodes;
    QRectF outerRegion;
    std::string robot_name;

    void initialize(const std::shared_ptr<DSR::DSRGraph> &graph_,const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &params_) 
    {
        qDebug()<<"Collisions - " <<__FUNCTION__;
        G = graph_;
        innerModel = G->get_inner_api();

//qDebug()<<"collide"<<collide("viriato_mesh", "tableA_tabletop");
//exit(0);
        /// Processing configuration parameters
 /*       try
        {
            outerRegion.setLeft(std::stof(params_->at("OuterRegionLeft").value));
            outerRegion.setRight(std::stof(params_->at("OuterRegionRight").value));
            outerRegion.setBottom(std::stof(params_->at("OuterRegionBottom").value));
            outerRegion.setTop(std::stof(params_->at("OuterRegionTop").value));
        }
        catch(const std::exception &e)
        {
            std::cout << "Exception " << e.what() << " Collisions::initialize(). OuterRegion parameters not found in config file" << std::endl;
            //robocomp::exception ex("OuterRegion parameters not found in config file");
            throw e;
        }*/
        //read from World (DSR node)
        std::optional<Node> world_node = G->get_node("world");
        if(world_node.has_value()) {
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
        qDebug() << __FILE__ << __FUNCTION__ << ls.size() << "objects read for exclusion list";

        foreach(const QString &s, ls)
            excludedNodes.insert(s.toStdString());

        // Compute the list of meshes that correspond to robot, world and possibly some additionally excluded ones
        robotNodes.clear(); restNodes.clear();
        recursiveIncludeMeshes(G->get_node_root().value(), robot_name, false, robotNodes, restNodes, excludedNodes);
//std::cout<<"lists"<<std::endl;
//std::cout<<"robot"<<robotNodes<<std::endl;
//std::cout<<"rest"<<restNodes<<std::endl;
        qsrand( QTime::currentTime().msec() );
    }

    bool checkRobotValidStateAtTargetFast(const std::vector<float> &targetPos, const std::vector<float> &targetRot)    
    {
    //First we move the robot in our copy of innermodel to its current coordinates
        std::optional<Edge> edge = G->get_edge("world", robot_name, "RT");
        G->modify_attrib_local(edge.value(), "translation", targetPos);
        G->modify_attrib_local(edge.value(), "rotation_euler_xyz", targetRot);
//          innerModel->updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());

        ///////////////////////
        //// Check if the robot at the target collides with any know object
        ///////////////////////

        bool collision = false;

        for ( std::string in : robotNodes )
        {
            for ( std::string  out : restNodes )
            {
                try
                {
                    collision = collide(in, out);
                }
                catch(QString s) {qDebug()<< __FUNCTION__ << s;}
                if (collision)
                {
                    std::cout<<"COLLISION: "<<in<<" to "<<out<<" pos: ("<<targetPos[0]<<","<<targetPos[1]<<","<<targetPos[2]<<")"<<std::endl;
                    return false;
                }
            }
        }
        return true;
    }

    void recursiveIncludeMeshes(Node node, std::string robot_name, bool inside, std::vector<std::string> &in, std::vector<std::string> &out, std::set<std::string> &excluded)
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

    bool collide(const std::string &node_a_name, const std::string &node_b_name)
    {
//std::cout << "collide " << node_a_name << " to "<< node_b_name << std::endl;
        
        QMat r1q = innerModel->getTransformationMatrixS("world", node_a_name).value();
        fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
        fcl::Vec3f T1( r1q(0,3), r1q(1,3), r1q(2,3) );


        QMat r2q = innerModel->getTransformationMatrixS("world", node_b_name).value();
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
        {
            return false;
        }
        
    }

    //return collison object, creates it if does not exist
    fcl::CollisionObject* get_collision_object(std::string node_name)
    {
        if (collision_objects.find(node_name) == collision_objects.end())
        { 
            // object creation
            std::optional<Node> node = G->get_node(node_name);
            if (node.has_value())
            {
                if( node.value().type() == "plane" )
                {
                    collision_objects[node_name] = create_plane_collision_object(node.value());
                }
                else
                {
                    if( node.value().type() == "mesh")
                    {
                        collision_objects[node_name] = create_mesh_collision_object(node.value());
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

    fcl::CollisionObject* create_mesh_collision_object(Node node)
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
            std::optional<QVec> pose = innerModel->transformS6D(node.name(), parent.value().name());
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

    fcl::CollisionObject* create_plane_collision_object(Node node)
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
        std::optional<QVec> pose = innerModel->transformS6D(node.name(), parent.value().name());
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


private:

    std::shared_ptr<DSR::InnerAPI> innerModel;
    std::shared_ptr<DSR::DSRGraph> G;

    std::map<std::string, fcl::CollisionObject*> collision_objects;

};

#endif //COLLISIONS_H
