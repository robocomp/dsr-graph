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

        fcl::CollisionObject* n1 = create_collision_object(node_a_name);
        n1->setTransform(R1, T1);
        n1->computeAABB();
        fcl::AABB a1 = n1->getAABB();
        fcl::Vec3f v1 = a1.center();

        fcl::CollisionObject* n2 = create_collision_object(node_b_name);
        n2->setTransform(R2, T2);
        n2->computeAABB();
        fcl::AABB a2 = n2->getAABB();
        fcl::Vec3f v2 = a2.center();

        fcl::collide(n1, n2, request, result);
        return result.isCollision();
    }

    fcl::CollisionObject* create_collision_object(std::string node_name)
    {



    }

private:

    std::shared_ptr<CRDT::InnerAPI> innerModel;
    std::shared_ptr<CRDT::CRDTGraph> G;
};

#endif //COLLISIONS_H
