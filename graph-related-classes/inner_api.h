#ifndef INNER_API
#define INNER_API

#include "../../../graph-related-classes/CRDT.h"
#include <qmat/QMatAll>

class InnerAPI
{
    using ListsPtr = std::tuple<std::list<EdgeAttribs>, std::list<EdgeAttribs> >; 
    public:
        InnerAPI(std::shared_ptr<CRDT::CRDTGraph> _G);
        
        /////////////////////////////////////////////////
        /// Kinematic transformation methods
        ////////////////////////////////////////////////
        QVec transform(const QString & destId, const QVec &origVec, const QString & origId);
        QVec transform( const QString &destId, const QString & origId);
        QVec transformS( const std::string &destId, const QVec &origVec, const std::string & origId);
        QVec transformS( const std::string &destId, const std::string &origId);
        QVec transform6D(const QString &destId, const QVec &origVec, const QString & origId); 
        QVec transform6D(const QString &destId, const QString & origId);
        QVec transformS6D(const std::string &destId, const std::string & origId);
        QVec transformS6D(const std::string &destId, const QVec &origVec, const std::string & origId);
            
        ////////////////////////////////////////////////
        /// Transformation matrix retrieval methods
        ////////////////////////////////////////////////
        RTMat getTransformationMatrix(const QString &destId, const QString &origId);
        RTMat getTransformationMatrixS(const std::string &destId, const std::string &origId);
        QMat getRotationMatrixTo(const QString &to, const QString &from);
        QVec getTranslationVectorTo(const QString &to, const QString &from);
        QVec rotationAngles(const QString & destId, const QString & origId);
    
    private:
        std::shared_ptr<CRDT::CRDTGraph> G;
        ListsPtr setLists(const std::string &origId, const std::string &destId);
       
};

#endif