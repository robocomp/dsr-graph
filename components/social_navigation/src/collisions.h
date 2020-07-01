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

class Collisions {

public:
    std::vector<QString> robotNodes;
    std::vector<QString> restNodes;
    std::set<QString> excludedNodes;
    QRectF outerRegion;

    void initialize(const std::shared_ptr<CRDT::CRDTGraph> &graph_,const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &params_) {


        qDebug()<<"Collisions - " <<__FUNCTION__;
        G = graph_;
        innerModel = G->get_inner_api();

//qDebug()<<"collide"<<collide("fridge_mesh", "viriato_mesh");
//exit(0);
        /// Processing configuration parameters
        try
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
        }

        if(outerRegion.isNull())
        {
            qDebug()<<"[ERROR] OUTER REGION IS NULL";
        }

        QStringList ls = QString::fromStdString(params_->at("ExcludedObjectsInCollisionCheck").value).replace(" ", "" ).split(',');
        qDebug() << __FILE__ << __FUNCTION__ << ls.size() << "objects read for exclusion list";

                foreach(const QString &s, ls)
                excludedNodes.insert(s);

        // Compute the list of meshes that correspond to robot, world and possibly some additionally excluded ones
        robotNodes.clear(); restNodes.clear();
//        recursiveIncludeMeshes(innerModel->getRoot(), "robot", false, robotNodes, restNodes, excludedNodes);
        qsrand( QTime::currentTime().msec() );

    }

    bool checkRobotValidStateAtTargetFast(const QVec &targetPos, const QVec &targetRot) const   {
        //First we move the robot in our copy of innermodel to its current coordinates

  //          innerModel->updateTransformValues("robot", targetPos.x(), targetPos.y(), targetPos.z(), targetRot.x(), targetRot.y(), targetRot.z());

            ///////////////////////
            //// Check if the robot at the target collides with any know object
            ///////////////////////

            bool collision = false;

            for ( auto &in : robotNodes )
            {
                for ( auto &out : restNodes )
                {
                    try {
//                        collision = innerModel->collide(in, out);
                    }

                    catch(QString s) {qDebug()<< __FUNCTION__ << s;}

                    if (collision)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

/*
    void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out, std::set<QString> &excluded) {

        if (node->id == robotId)
        {
            inside = true;
        }

        InnerModelMesh *mesh;
        InnerModelPlane *plane;
        InnerModelTransform *transformation;

        if ((transformation = dynamic_cast<InnerModelTransform *>(node)))
        {
            for (int i=0; i<node->children.size(); i++)
            {
                recursiveIncludeMeshes(node->children[i], robotId, inside, in, out, excluded);

            }

        }

        else if ((mesh = dynamic_cast<InnerModelMesh *>(node)) or (plane = dynamic_cast<InnerModelPlane *>(node)))
        {
            if( std::find(excluded.begin(), excluded.end(), node->id) == excluded.end() )
            {
                if (inside)
                {
                    in.push_back(node->id);
                }
                else
                if(mesh or plane)
                    out.push_back(node->id);
            }
        }

    }
*/
    bool collide(const std::string &node_a_name, const std::string &node_b_name)
    {
        std::cout << "collide " << node_a_name << node_b_name << std::endl;
        
        QMat r1q = innerModel->getTransformationMatrixS("world", node_a_name).value();
        fcl::Matrix3f R1( r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2) );
        fcl::Vec3f T1( r1q(0,3), r1q(1,3), r1q(2,3) );


        QMat r2q = innerModel->getTransformationMatrixS("world", node_b_name).value();
        fcl::Matrix3f R2( r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2) );
        fcl::Vec3f T2( r2q(0,3), r2q(1,3), r2q(2,3) );
        

        fcl::CollisionRequest request;
        fcl::CollisionResult result;

        fcl::CollisionObject* n1 = get_collision_object(node_a_name);
        n1->setTransform(R1, T1);
        n1->computeAABB();

        fcl::CollisionObject* n2 = get_collision_object(node_b_name);
        n2->setTransform(R2, T2);
        n2->computeAABB();

        fcl::collide(n1, n2, request, result);
        return result.isCollision();
    }
    //TODO: update objects if node parameters values changed

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
            std::optional<QVec> pose = innerModel->transformS6D(parent.value().name(), node.name());
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

        return collision_object;
    }


private:

    std::shared_ptr<CRDT::InnerAPI> innerModel;
    std::shared_ptr<CRDT::CRDTGraph> G;

    std::map<std::string, fcl::CollisionObject*> collision_objects;

};

#endif //COLLISIONS_H
