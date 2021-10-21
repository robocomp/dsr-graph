//
// Created by juancarlos on 13/10/21.
//

#ifndef QT3D_PRUEBAS_CUBOID_H
#define QT3D_PRUEBAS_CUBOID_H

#include <Qt3DRender>
#include <QObject>

class QCuboidGeom;

class QCuboidGeometryPrivate
{
public:
    QCuboidGeometryPrivate();
    void init(QCuboidGeom *q);

    // Dimensions
    float m_xExtent;
    float m_yExtent;
    float m_zExtent;

    // Resolutions of faces with normal x, y, and z
    QSize m_yzFaceResolution;
    QSize m_xzFaceResolution;
    QSize m_xyFaceResolution;

    Qt3DRender::QAttribute *m_positionAttribute;
    Qt3DRender::QAttribute *m_normalAttribute;
    Qt3DRender::QAttribute *m_texCoordAttribute;
    Qt3DRender::QAttribute *m_tangentAttribute;
    Qt3DRender::QAttribute *m_indexAttribute;
    Qt3DRender::QBuffer *m_vertexBuffer;
    Qt3DRender::QBuffer *m_indexBuffer;

};


class QCuboidGeom : public Qt3DRender::QGeometry
{
    Q_OBJECT
    Q_PROPERTY(float xExtent WRITE setXExtent )
    Q_PROPERTY(float yExtent WRITE setYExtent )
    Q_PROPERTY(float zExtent WRITE setZExtent )
    Q_PROPERTY(QSize xyMeshResolution WRITE setXYMeshResolution )
    Q_PROPERTY(QSize yzMeshResolution WRITE setYZMeshResolution )
    Q_PROPERTY(QSize xzMeshResolution WRITE setXZMeshResolution )

    public:
    explicit QCuboidGeom(QNode *parent = nullptr);
    ~QCuboidGeom() override;



    void init();

    public Q_SLOTS:
    void setXExtent(float xExtent);
    void setYExtent(float yExtent);
    void setZExtent(float zExtent);
    void setYZMeshResolution(const QSize &resolution);
    void setXZMeshResolution(const QSize &resolution);
    void setXYMeshResolution(const QSize &resolution);


    Q_SIGNALS:
    void VertexBufferLoaded();

    private:
    QCuboidGeometryPrivate * d_ptr;
    Q_DECLARE_PRIVATE(QCuboidGeometry)
};


#endif //QT3D_PRUEBAS_CUBOID_H
