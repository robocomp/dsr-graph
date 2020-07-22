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

class Collisions
{
    public:
        void initialize(const std::shared_ptr<DSR::DSRGraph> &graph_, const std::shared_ptr< RoboCompCommonBehavior::ParameterList > &params_);
        bool checkRobotValidStateAtTargetFast(DSR::DSRGraph &G_copy, const std::vector<float> &targetPos, const std::vector<float> &targetRot);
        QRectF outerRegion;

    private:
        std::shared_ptr<DSR::DSRGraph> G;
        std::map<std::string, fcl::CollisionObject*> collision_objects;
        std::vector<std::string> robotNodes;
        std::vector<std::string> restNodes;
        std::set<std::string> excludedNodes;

        std::string robot_name;

        void recursiveIncludeMeshes(Node node, std::string robot_name, bool inside, std::vector<std::string> &in, std::vector<std::string> &out, std::set<std::string> &excluded);
        bool collide(std::shared_ptr<DSR::InnerAPI> innerModel, const std::string &node_a_name, const std::string &node_b_name);
        //return collison object, creates it if does not exist
        fcl::CollisionObject* get_collision_object(std::shared_ptr<DSR::InnerAPI> inner, std::string node_name);
        fcl::CollisionObject* create_mesh_collision_object(std::shared_ptr<DSR::InnerAPI> inner, const Node &node);
        fcl::CollisionObject* create_plane_collision_object(std::shared_ptr<DSR::InnerAPI> inner, const Node &node);
};

#endif //COLLISIONS_H
