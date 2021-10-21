//
// Created by juancarlos on 13/10/21.
//

//Es la implementación de Qt3DExtras pero envía una señal cual termina el job de la generación de los vértices.

#include "Cuboid.h"
#include <Qt3DRender/qattribute.h>
#include <Qt3DRender/qbuffer.h>
#include <Qt3DRender/qbufferdatagenerator.h>
#include <QObject>

namespace {

    enum PlaneNormal {
        PositiveX,
        NegativeX,
        PositiveY,
        NegativeY,
        PositiveZ,
        NegativeZ
    };

    void createPlaneVertexData(float w, float h, const QSize &resolution,
                               PlaneNormal normal, float planeDistance,
                               float *vertices)
    {
        const float a0 = -w / 2.0f;
        const float b0 = -h / 2.0f;
        const float da = w / (resolution.width() - 1);
        const float db = h / (resolution.height() - 1);
        const float du = 1.0f / (resolution.width() - 1);
        const float dv = 1.0f / (resolution.height() - 1);

        switch (normal) {
            case NegativeX:
                // Iterate over z
                for (int j = resolution.height() - 1; j >= 0; --j) {
                    const float b = b0 + static_cast<float>(j) * db;
                    const float v = static_cast<float>(j) * dv;

                    // Iterate over y
                    for (int i = 0; i < resolution.width(); ++i) {
                        const float a = a0 + static_cast<float>(i) * da;
                        const float u = static_cast<float>(i) * du;

                        // position
                        *vertices++ = planeDistance;
                        *vertices++ = a;
                        *vertices++ = b;

                        // texture coordinates
                        *vertices++ = v;
                        *vertices++ = u;

                        // normal
                        *vertices++ = -1.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;

                        // tangent
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 1.0f;
                        *vertices++ = 1.0f;
                    }
                }
                break;

            case PositiveX: {
                // Iterate over z
                for (int j = 0; j < resolution.height(); ++j) {
                    const float b = b0 + static_cast<float>(j) * db;
                    const float v = static_cast<float>(j) * dv;

                    // Iterate over y
                    for (int i = 0; i < resolution.width(); ++i) {
                        const float a = a0 + static_cast<float>(i) * da;
                        const float u = static_cast<float>(i) * du;

                        // position
                        *vertices++ = planeDistance;
                        *vertices++ = a;
                        *vertices++ = b;

                        // texture coordinates
                        *vertices++ = 1.0f - v;
                        *vertices++ = u;

                        // normal
                        *vertices++ = 1.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;

                        // tangent
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = -1.0f;
                        *vertices++ = 1.0f;
                    }
                }
                break;
            }

            case NegativeY:
                // Iterate over z
                for (int j = 0; j < resolution.height(); ++j) {
                    const float b = b0 + static_cast<float>(j) * db;
                    const float v = static_cast<float>(j) * dv;

                    // Iterate over x
                    // This iterates in the opposite sense to the other directions
                    // so that the winding order is correct
                    for (int i = 0; i < resolution.width(); ++i) {
                        const float a = a0 + static_cast<float>(i) * da;
                        const float u = static_cast<float>(i) * du;

                        // position
                        *vertices++ = a;
                        *vertices++ = planeDistance;
                        *vertices++ = b;

                        // texture coordinates
                        *vertices++ = u;
                        *vertices++ = v;

                        // normal
                        *vertices++ = 0.0f;
                        *vertices++ = -1.0f;
                        *vertices++ = 0.0f;

                        // tangent
                        *vertices++ = 1.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 1.0f;
                    }
                }
                break;

            case PositiveY: {
                // Iterate over z
                for (int j = resolution.height() - 1; j >= 0; --j) {
                    const float b = b0 + static_cast<float>(j) * db;
                    const float v = static_cast<float>(j) * dv;

                    // Iterate over x
                    // This iterates in the opposite sense to the other directions
                    // so that the winding order is correct
                    for (int i = 0; i < resolution.width(); ++i) {
                        const float a = a0 + static_cast<float>(i) * da;
                        const float u = static_cast<float>(i) * du;

                        // position
                        *vertices++ = a;
                        *vertices++ = planeDistance;
                        *vertices++ = b;

                        // texture coordinates
                        *vertices++ = u;
                        *vertices++ = 1.0f - v;

                        // normal
                        *vertices++ = 0.0f;
                        *vertices++ = 1.0f;
                        *vertices++ = 0.0f;

                        // tangent
                        *vertices++ = 1.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 1.0f;
                    }
                }
                break;
            }

            case NegativeZ:
                // Iterate over y
                for (int j = 0; j < resolution.height(); ++j) {
                    const float b = b0 + static_cast<float>(j) * db;
                    const float v = static_cast<float>(j) * dv;

                    // Iterate over x
                    for (int i = resolution.width() - 1; i >= 0; --i) {
                        const float a = a0 + static_cast<float>(i) * da;
                        const float u = static_cast<float>(i) * du;

                        // position
                        *vertices++ = a;
                        *vertices++ = b;
                        *vertices++ = planeDistance;

                        // texture coordinates
                        *vertices++ = 1.0f - u;
                        *vertices++ = v;

                        // normal
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = -1.0f;

                        // tangent
                        *vertices++ = -1.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 1.0f;
                    }
                }
                break;

            case PositiveZ: {
                // Iterate over y
                for (int j = 0; j < resolution.height(); ++j) {
                    const float b = b0 + static_cast<float>(j) * db;
                    const float v = static_cast<float>(j) * dv;

                    // Iterate over x
                    for (int i = 0; i < resolution.width(); ++i) {
                        const float a = a0 + static_cast<float>(i) * da;
                        const float u = static_cast<float>(i) * du;

                        // position
                        *vertices++ = a;
                        *vertices++ = b;
                        *vertices++ = planeDistance;

                        // texture coordinates
                        *vertices++ = u;
                        *vertices++ = v;

                        // normal
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 1.0f;

                        // tangent
                        *vertices++ = 1.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 0.0f;
                        *vertices++ = 1.0f;
                    }
                }
                break;
            }
        } // switch (normal)
    }

    void createPlaneIndexData(const QSize &resolution, quint16 *indices, quint16 &baseVertex)
    {
        // Populate indices taking care to get correct CCW winding on all faces
        // Iterate over v direction (rows)
        for (int j = 0; j < resolution.height() - 1; ++j) {
            const int rowStartIndex = j * resolution.width() + baseVertex;
            const int nextRowStartIndex = (j + 1) * resolution.width() + baseVertex;

            // Iterate over u direction (columns)
            for (int i = 0; i < resolution.width() - 1; ++i) {
                // Split quad into two triangles
                *indices++ = rowStartIndex + i;
                *indices++ = rowStartIndex + i + 1;
                *indices++ = nextRowStartIndex + i;

                *indices++ = nextRowStartIndex + i;
                *indices++ = rowStartIndex + i + 1;
                *indices++ = nextRowStartIndex + i + 1;
            }
        }
        baseVertex += resolution.width() * resolution.height();
    }

    QByteArray createCuboidVertexData(float xExtent,
                                      float yExtent,
                                      float zExtent,
                                      const QSize &yzResolution,
                                      const QSize &xzResolution,
                                      const QSize &xyResolution)
    {
        Q_ASSERT(xExtent > 0.0f && yExtent > 0.0f && zExtent > 0.0);
        Q_ASSERT(yzResolution.width() >= 2 && yzResolution.height() >=2);
        Q_ASSERT(xzResolution.width() >= 2 && xzResolution.height() >=2);
        Q_ASSERT(xyResolution.width() >= 2 && xyResolution.height() >=2);

        const int yzVerts = yzResolution.width() * yzResolution.height();
        const int xzVerts = xzResolution.width() * xzResolution.height();
        const int xyVerts = xyResolution.width() * xyResolution.height();
        const int nVerts = 2 * (yzVerts + xzVerts + xyVerts);

        const quint32 elementSize = 3 + 3 + 2 + 4;
        const quint32 stride = elementSize * sizeof(float);
        QByteArray vertexBytes;
        vertexBytes.resize(stride * nVerts);
        float* vertices = reinterpret_cast<float*>(vertexBytes.data());

        createPlaneVertexData(yExtent, zExtent, yzResolution, PositiveX, xExtent * 0.5f, vertices);
        vertices += yzVerts * elementSize;
        createPlaneVertexData(yExtent, zExtent, yzResolution, NegativeX, -xExtent * 0.5f, vertices);
        vertices += yzVerts * elementSize;
        createPlaneVertexData(xExtent, zExtent, xzResolution, PositiveY, yExtent * 0.5f, vertices);
        vertices += xzVerts * elementSize;
        createPlaneVertexData(xExtent, zExtent, xzResolution, NegativeY, -yExtent * 0.5f, vertices);
        vertices += xzVerts * elementSize;
        createPlaneVertexData(xExtent, yExtent, xyResolution, PositiveZ, zExtent * 0.5f, vertices);
        vertices += xyVerts * elementSize;
        createPlaneVertexData(xExtent, yExtent, xyResolution, NegativeZ, -zExtent * 0.5f, vertices);

        return vertexBytes;
    }

    QByteArray createCuboidIndexData(const QSize &yzResolution,
                                     const QSize &xzResolution,
                                     const QSize &xyResolution)
    {
        Q_ASSERT(yzResolution.width() >= 2 && yzResolution.height() >= 2);
        Q_ASSERT(xzResolution.width() >= 2 && xzResolution.height() >= 2);
        Q_ASSERT(xyResolution.width() >= 2 && xyResolution.height() >= 2);

        const int yzIndices = 2 * 3 * (yzResolution.width() - 1) * (yzResolution.height() - 1);
        const int xzIndices = 2 * 3 * (xzResolution.width() - 1) * (xzResolution.height() - 1);
        const int xyIndices = 2 * 3 * (xyResolution.width() - 1) * (xyResolution.height() - 1);
        const int indexCount = 2 * (yzIndices + xzIndices + xyIndices);

        QByteArray indexData;
        indexData.resize(indexCount * sizeof(quint16));
        quint16 *indices = reinterpret_cast<quint16 *>(indexData.data());
        quint16 baseIndex = 0;

        createPlaneIndexData(yzResolution, indices, baseIndex);
        indices += yzIndices;
        createPlaneIndexData(yzResolution, indices, baseIndex);
        indices += yzIndices;
        createPlaneIndexData(xzResolution, indices, baseIndex);
        indices += xzIndices;
        createPlaneIndexData(xzResolution, indices, baseIndex);
        indices += xzIndices;
        createPlaneIndexData(xyResolution, indices, baseIndex);
        indices += xyIndices;
        createPlaneIndexData(xyResolution, indices, baseIndex);

        return indexData;
    }

} // anonymous

class CuboidVertexBufferFunctor : public Qt3DRender::QBufferDataGenerator
    {
    public:
        explicit CuboidVertexBufferFunctor(float xExtent,
                                           float yExtent,
                                           float zExtent,
                                           const QSize &yzResolution,
                                           const QSize &xzResolution,
                                           const QSize &xyResolution,
                                           QCuboidGeometryPrivate *geom)
                : m_xExtent(xExtent)
                , m_yExtent(yExtent)
                , m_zExtent(zExtent)
                , m_yzFaceResolution(yzResolution)
                , m_xzFaceResolution(xzResolution)
                , m_xyFaceResolution(xyResolution)
                , g(geom)
        {}

        ~CuboidVertexBufferFunctor() {}

        QByteArray operator()() final
        {
            auto tmp = createCuboidVertexData(m_xExtent, m_yExtent, m_zExtent,
                                          m_yzFaceResolution, m_xzFaceResolution, m_xyFaceResolution);
            return tmp;
        }

        bool operator ==(const QBufferDataGenerator &other) const final
        {
            const CuboidVertexBufferFunctor *otherFunctor = functor_cast<CuboidVertexBufferFunctor>(&other);
            if (otherFunctor != nullptr)
                return (otherFunctor->m_xExtent == m_xExtent &&
                        otherFunctor->m_yExtent == m_yExtent &&
                        otherFunctor->m_zExtent == m_zExtent &&
                        otherFunctor->m_yzFaceResolution == m_yzFaceResolution &&
                        otherFunctor->m_xzFaceResolution == m_xzFaceResolution &&
                        otherFunctor->m_xyFaceResolution == m_xyFaceResolution);
            return false;
        }

        QT3D_FUNCTOR(CuboidVertexBufferFunctor)

    private:
        float m_xExtent;
        float m_yExtent;
        float m_zExtent;
        QSize m_yzFaceResolution;
        QSize m_xzFaceResolution;
        QSize m_xyFaceResolution;
        QCuboidGeometryPrivate *g;
    };

class CuboidIndexBufferFunctor : public Qt3DRender::QBufferDataGenerator
{
    public:
        explicit CuboidIndexBufferFunctor(const QSize &yzResolution,
                                          const QSize &xzResolution,
                                          const QSize &xyResolution,
                                          QCuboidGeometryPrivate *geom)
                : m_yzFaceResolution(yzResolution)
                , m_xzFaceResolution(xzResolution)
                , m_xyFaceResolution(xyResolution)
                , g(geom)
        {}

        ~CuboidIndexBufferFunctor() {}

        QByteArray operator()() final
        {

            auto tmp =  createCuboidIndexData(m_yzFaceResolution, m_xzFaceResolution, m_xyFaceResolution);
            return tmp;
        }

        bool operator ==(const QBufferDataGenerator &other) const final
        {
            const CuboidIndexBufferFunctor *otherFunctor = functor_cast<CuboidIndexBufferFunctor>(&other);
            if (otherFunctor != nullptr)
                return (otherFunctor->m_yzFaceResolution == m_yzFaceResolution &&
                        otherFunctor->m_xzFaceResolution == m_xzFaceResolution &&
                        otherFunctor->m_xyFaceResolution == m_xyFaceResolution);
            return false;
        }

        QT3D_FUNCTOR(CuboidIndexBufferFunctor)

    private:
        QSize m_yzFaceResolution;
        QSize m_xzFaceResolution;
        QSize m_xyFaceResolution;
        QCuboidGeometryPrivate *g;
};

QCuboidGeometryPrivate::QCuboidGeometryPrivate() :
          m_xExtent(1.0f)
        , m_yExtent(1.0f)
        , m_zExtent(1.0f)
        , m_yzFaceResolution(2, 2)
        , m_xzFaceResolution(2, 2)
        , m_xyFaceResolution(2, 2)
        , m_positionAttribute(nullptr)
        , m_normalAttribute(nullptr)
        , m_texCoordAttribute(nullptr)
        , m_tangentAttribute(nullptr)
        , m_indexAttribute(nullptr)
        , m_vertexBuffer(nullptr)
        , m_indexBuffer(nullptr)
{
}

void QCuboidGeometryPrivate::init(QCuboidGeom *q)
{
    m_positionAttribute = new Qt3DRender::QAttribute(q);
    m_normalAttribute = new Qt3DRender::QAttribute(q);
    m_texCoordAttribute = new Qt3DRender::QAttribute(q);
    m_tangentAttribute = new Qt3DRender::QAttribute(q);
    m_indexAttribute = new Qt3DRender::QAttribute(q);
    m_vertexBuffer = new Qt3DRender::QBuffer(q);
    m_indexBuffer = new Qt3DRender::QBuffer(q);

    // vec3 pos vec2 tex vec3 normal vec4 tangent
    const quint32 stride = (3 + 2 + 3 + 4) * sizeof(float);
    const int yzIndices = 2 * 3 * (m_yzFaceResolution.width() - 1) * (m_yzFaceResolution.height() - 1);
    const int xzIndices = 2 * 3 * (m_xzFaceResolution.width() - 1) * (m_xzFaceResolution.height() - 1);
    const int xyIndices = 2 * 3 * (m_xyFaceResolution.width() - 1) * (m_xyFaceResolution.height() - 1);
    const int yzVerts = m_yzFaceResolution.width() * m_yzFaceResolution.height();
    const int xzVerts = m_xzFaceResolution.width() * m_xzFaceResolution.height();
    const int xyVerts = m_xyFaceResolution.width() * m_xyFaceResolution.height();

    const int nVerts = 2 * (yzVerts + xzVerts + xyVerts);
    const int indexCount = 2 * (yzIndices + xzIndices + xyIndices);

    m_positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    m_positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    m_positionAttribute->setVertexSize(3);
    m_positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    m_positionAttribute->setBuffer(m_vertexBuffer);
    m_positionAttribute->setByteStride(stride);
    m_positionAttribute->setCount(nVerts);

    m_texCoordAttribute->setName(Qt3DRender::QAttribute::defaultTextureCoordinateAttributeName());
    m_texCoordAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    m_texCoordAttribute->setVertexSize(2);
    m_texCoordAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    m_texCoordAttribute->setBuffer(m_vertexBuffer);
    m_texCoordAttribute->setByteStride(stride);
    m_texCoordAttribute->setByteOffset(3 * sizeof(float));
    m_texCoordAttribute->setCount(nVerts);

    m_normalAttribute->setName(Qt3DRender::QAttribute::defaultNormalAttributeName());
    m_normalAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    m_normalAttribute->setVertexSize(3);
    m_normalAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    m_normalAttribute->setBuffer(m_vertexBuffer);
    m_normalAttribute->setByteStride(stride);
    m_normalAttribute->setByteOffset(5 * sizeof(float));
    m_normalAttribute->setCount(nVerts);

    m_tangentAttribute->setName(Qt3DRender::QAttribute::defaultTangentAttributeName());
    m_tangentAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    m_tangentAttribute->setVertexSize(4);
    m_tangentAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    m_tangentAttribute->setBuffer(m_vertexBuffer);
    m_tangentAttribute->setByteStride(stride);
    m_tangentAttribute->setByteOffset(8 * sizeof(float));
    m_tangentAttribute->setCount(nVerts);

    m_indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    m_indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedShort);
    m_indexAttribute->setBuffer(m_indexBuffer);

    m_indexAttribute->setCount(indexCount);

    m_vertexBuffer->setSyncData(true);
    m_indexBuffer->setSyncData(true);


    m_vertexBuffer->setDataGenerator(QSharedPointer<CuboidVertexBufferFunctor>::create(m_xExtent, m_yExtent, m_zExtent,  m_yzFaceResolution,
                                                                                       m_xzFaceResolution, m_xyFaceResolution, this));
    m_indexBuffer->setDataGenerator(QSharedPointer<CuboidIndexBufferFunctor>::create(m_yzFaceResolution, m_xzFaceResolution, m_xyFaceResolution, this));




    QObject::connect(m_vertexBuffer, &Qt3DRender::QBuffer::dataChanged,
            q, [q](const QByteArray& data) {
                emit qobject_cast<QCuboidGeom *>(q)->VertexBufferLoaded();
            },
            Qt::QueuedConnection);



    q->addAttribute(m_positionAttribute);
    q->addAttribute(m_texCoordAttribute);
    q->addAttribute(m_normalAttribute);
    q->addAttribute(m_tangentAttribute);
    q->addAttribute(m_indexAttribute);

}


QCuboidGeom::QCuboidGeom(QNode *parent)
    : QGeometry(parent)
{
    d_ptr = new QCuboidGeometryPrivate();
}

void QCuboidGeom::init()
{
    Q_D(QCuboidGeometry);
    d->init(this);
}

QCuboidGeom::~QCuboidGeom()
{
    delete d_ptr;
}

void QCuboidGeom::setXExtent(float xExtent)
{
    Q_D(QCuboidGeometry);
    d->m_xExtent = xExtent;
}

void QCuboidGeom::setYExtent(float yExtent)
{
    Q_D(QCuboidGeometry);
    d->m_yExtent = yExtent;
}

void QCuboidGeom::setZExtent(float zExtent)
{
    Q_D(QCuboidGeometry);
    d->m_zExtent = zExtent;
}

void QCuboidGeom::setYZMeshResolution(const QSize &resolution)
{
    Q_D(QCuboidGeometry);
    d->m_yzFaceResolution = resolution;
}

void QCuboidGeom::setXZMeshResolution(const QSize &resolution)
{
    Q_D(QCuboidGeometry);
    d->m_xzFaceResolution = resolution;
}

void QCuboidGeom::setXYMeshResolution(const QSize &resolution)
{
    Q_D(QCuboidGeometry);
    d->m_xyFaceResolution = resolution;
}



