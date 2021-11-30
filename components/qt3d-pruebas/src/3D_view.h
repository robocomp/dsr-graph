//
// Created by juancarlos on 26/8/21.
//

#ifndef DSR_GRAPH_3D_VIEW_H
#define DSR_GRAPH_3D_VIEW_H
#include <numeric>

#include <Qt3DCore/QEntity>
#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>
#include <Qt3DCore/QTransform>
#include <Qt3DCore/QAspectEngine>

#include <Qt3DInput/QInputAspect>

#include <Qt3DRender/QRenderAspect>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QPlaneGeometry>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QCuboidGeometry>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras>
#include <Qt3DRender/QMesh>


#include <Qt3DRender>
#include <Qt3DCore/qtransform.h>
#include <Qt3DInput>

#include <QObject>
#include <QWidget>

#include <QMatrix4x4>
#include <Qt3DWindow>

#include <dsr/api/dsr_api.h>

#include "WireFrameMat.h"
#include "WireFrameEffect.h"

//#include<PointCloud.h>
//#include<TriangleMesh.h>

#include "KdTree.h"

#include <immintrin.h>


#include <dsr/api/GeometryInfo.h>
#include "Cuboid.h"



inline static Qt3DCore::QEntity* drawSphere(const QVector3D& position, float radius , const QColor& color, Qt3DCore::QEntity *_rootEntity)
{
    auto *Entity = new Qt3DCore::QEntity( _rootEntity);

    //Entity->setObjectName(QString(node.name().data()));
    auto *material = new Qt3DExtras::QPhongMaterial();
    material->setAmbient(color);
    material->setSpecular(color);
    auto *sphMesh = new Qt3DExtras::QSphereMesh;

    sphMesh->setRadius(radius);


    Entity->addComponent(sphMesh);

    auto *planeTransform = new Qt3DCore::QTransform;
    planeTransform->setScale3D(QVector3D(1.0, 1.0, 1.0));
    planeTransform->setTranslation(position);

    Entity->addComponent(material);
    Entity->addComponent(planeTransform);

    return Entity;
}

inline static Qt3DCore::QEntity* drawLine(const QVector3D& start, const QVector3D& end, const QColor& color, Qt3DCore::QEntity *_rootEntity)
{
    auto *geometry = new Qt3DRender::QGeometry(_rootEntity);

    // position vertices (start and end)
    QByteArray bufferBytes;
    bufferBytes.resize(3 * 2 * sizeof(float)); // start.x, start.y, start.end + end.x, end.y, end.z
    auto *positions = reinterpret_cast<float*>(bufferBytes.data());
    *positions++ = start.x();
    *positions++ = start.y();
    *positions++ = start.z();
    *positions++ = end.x();
    *positions++ = end.y();
    *positions++ = end.z();

    auto *buf = new Qt3DRender::QBuffer(geometry);
    buf->setData(bufferBytes);

    auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
    positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(buf);
    positionAttribute->setByteStride(3 * sizeof(float));
    positionAttribute->setCount(2);
    geometry->addAttribute(positionAttribute); // We add the vertices in the geometry

    // connectivity between vertices
    QByteArray indexBytes;
    indexBytes.resize(2 * sizeof(unsigned int)); // start to end
    auto *indices = reinterpret_cast<unsigned int*>(indexBytes.data());
    *indices++ = 0;
    *indices++ = 1;

    auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
    indexBuffer->setData(indexBytes);

    auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
    indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
    indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(2);
    geometry->addAttribute(indexAttribute); // We add the indices linking the points in the geometry

    // mesh
    auto *line = new Qt3DRender::QGeometryRenderer(_rootEntity);
    line->setGeometry(geometry);
    line->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);
    auto *material = new Qt3DExtras::QPhongMaterial(_rootEntity);
    material->setAmbient(color);

    // entity
    auto *lineEntity = new Qt3DCore::QEntity(_rootEntity);
    lineEntity->addComponent(line);
    lineEntity->addComponent(material);

    return lineEntity;
}

class World : public QObject
{
Q_OBJECT

    /*
    Posibilidad de tener varias cámaras. Una en la cabeza del robot, otra cenital y otra en orbital que pueda moverse. (mirar viewports)
     * */
public:

    void insert_point(std::tuple<float, float, float> p)
    {
        auto [x, y, z] = p;
        drawSphere(QVector3D(x, y, z), 1.0, QColor(0xfff000), rootEntity);
    }


    explicit World(DSR::DSRGraph *g_) : g(g_), inner(g->get_inner_eigen_api()), geom_info(TempSingleton<GeomInfo>::get()) {

        view = new Qt3DExtras::Qt3DWindow;
        widget = createWidget();
        widget->setMinimumSize(QSize(500, 400));

        connect(g, &DSR::DSRGraph::update_node_signal, this, &World::updated_node);
        connect(g, &DSR::DSRGraph::update_edge_signal, this, &World::updated_edge);
        connect(g, &DSR::DSRGraph::update_node_attr_signal, this, &World::updated_node_attr);
        connect(g, &DSR::DSRGraph::update_edge_attr_signal, this, &World::updated_edge_attr);
        connect(g, &DSR::DSRGraph::del_edge_signal, this, &World::deleted_edge);
        connect(g, &DSR::DSRGraph::del_node_signal, this, &World::deleted_node);

        initialize();

    }

    ~World() override {
        if (!only_one_widget) {
            delete view;
        }
    }

    QWidget *createWidget() {
        if (!only_one_widget) {
            only_one_widget = true;
            return QWidget::createWindowContainer(view);
        }
        return nullptr;
    }

    QWidget *getWidget() {
        return widget;
    }

    void show() { view->show(); }

    static float normalize( const float value, float start = 0.f, float end = 360.f)
    {
        const float width       = end - start   ;
        const float offsetValue = value - start ;

        return ( offsetValue - ( floor( offsetValue / width ) * width ) ) + start ;
    }


    void addGeomInfo(Qt3DCore::QEntity* Entity)
    {

        auto c_root = rootEntity->components();
        auto world_tr = std::find_if(c_root.begin(), c_root.end(),
                               [](auto &p) {
                                   return qobject_cast<Qt3DCore::QTransform *>(p);
                               });

        auto identity = QMatrix4x4();
        identity.setToIdentity();
        auto world_matrix = (world_tr != c_root.end() && qobject_cast<Qt3DCore::QTransform *>(*world_tr)) ?
                qobject_cast<Qt3DCore::QTransform *>(*world_tr)->matrix() : identity;

        auto components = Entity->components();
        auto c = std::find_if(components.begin(), components.end(),
                              [](auto &p) {
                                  return qobject_cast<Qt3DRender::QGeometryRenderer *>(p);
                              });
        if (c != components.end()) {
            auto renderer = qobject_cast<Qt3DRender::QGeometryRenderer *>(*c);
            if (renderer != nullptr && renderer->geometry()) {
                auto atts = renderer->geometry()->attributes();
                auto indexAtt =
                        std::find_if(atts.begin(), atts.end(), [](Qt3DRender::QAttribute *&p) {
                            return p->attributeType() == Qt3DRender::QAttribute::IndexAttribute;
                        });

                auto posAtt =
                        std::find_if(atts.begin(), atts.end(), [](Qt3DRender::QAttribute *&p) {
                            return p->name() == Qt3DRender::QAttribute::defaultPositionAttributeName();
                        });

                auto tr = std::find_if(components.begin(), components.end(),
                                      [](auto &p) {
                                          return qobject_cast<Qt3DCore::QTransform *>(p);
                                      });

                if (indexAtt != atts.end() && posAtt != atts.end() && (*posAtt)->buffer()->type() == Qt3DRender::QBuffer::VertexBuffer && tr != components.end()) {

                    auto *transform = qobject_cast<Qt3DCore::QTransform *>(*tr);
                    auto nverts = (*posAtt)->count(); // Número de vértices.
                    auto stride = (*posAtt)->byteStride(); //Incremento en cada iteración. En memoria va contiguo posiciones, normales, etc.
                    auto byteOffset = (*posAtt)->byteOffset(); //Offset para el inicio.


                    auto idxCount = (*indexAtt)->count(); // Número de índices

                    std::cout << Entity->objectName().toStdString() << " - VERTICES: " << nverts << " Offset " << byteOffset << " Stride " << stride << " idxcount " << idxCount << std::endl;


                    //////////////////////////////////////////
                    /// Indices
                    //////////////////////////////////////////
                    QByteArray idxBufferData = (*indexAtt)->buffer()->data();//indexDataBuffer->data();
                    auto *rawIdxArray = reinterpret_cast<unsigned short *>(idxBufferData.data());

                    std::vector<unsigned short> indices; //(rawIdxArray, rawIdxArray + idxCount);

                    //////////////////////////////////////////

                    auto *vertexDataBuffer = (*posAtt)->buffer();

                    QByteArray vertexBufferData = vertexDataBuffer->data();
                    std::vector<std::tuple<float, float, float>> positions;
                    positions.reserve(nverts);

                    std::cout << "----------------- " << Entity->objectName().toStdString() << " --------------------" << std::endl;

                    auto matrix =  transform->matrix();

                    std::cout << "--------------------------------------------------" << std::endl;


                    for (unsigned int i = 0; i < idxCount ; i+=6)
                    {
                        int idxPos1  = rawIdxArray[i] * stride +  byteOffset;
                        int idxPos2  = rawIdxArray[i+1] * stride +  byteOffset;
                        int idxPos3  = rawIdxArray[i+2] * stride +  byteOffset;
                        int idxPos4  = rawIdxArray[i+3] * stride +  byteOffset;
                        int idxPos5  = rawIdxArray[i+4] * stride +  byteOffset;
                        int idxPos6  = rawIdxArray[i+5] * stride +  byteOffset;

                        //Ignore tangents (3, 6)
                        int idx = 0;
                        for (auto idxPos : {idxPos1, idxPos2, idxPos3, idxPos4, idxPos5, idxPos6}) {
                            QByteArray posx = vertexBufferData.mid(idxPos + 0 * sizeof(float), sizeof(float));
                            QByteArray posy = vertexBufferData.mid(idxPos + 1 * sizeof(float), sizeof(float));
                            QByteArray posz = vertexBufferData.mid(idxPos + 2 * sizeof(float), sizeof(float));


                            float x = *reinterpret_cast<const float *>(posx.data());
                            float y = *reinterpret_cast<const float *>(posy.data());
                            float z = *reinterpret_cast<const float *>(posz.data());

                            //std::cout << "idx: " << idx << " Vertex: " << x << " " << y << " " << z << std::endl;
                            QVector4D tmp = world_matrix * matrix * QVector4D(x, y, z, 1.0);
                            auto v3 = tmp.toVector3D();

                            positions.emplace_back(v3.x(), v3.y(), v3.z());
                            indices.push_back(positions.size()-1);
                            idx++;
                        }

                        //std::cout << " ------------------------------- "<< std::endl;

                    }

                    //Entity->setEnabled(false);


                    std::cout << "positions : " << positions.size() << " indices: " << indices.size() << std::endl;

                    if (!Entity->objectName().isEmpty()) {
                        geom_info->addQtGeom(Entity, std::move(positions), std::move(indices));
                    } else {
                        std::cout << "Skip entity without name" << std::endl;
                    }
                } else {
                    std::cout << "No idx att or no indice att" << std::endl;
                }

            } else {
                std::cout << "No geometry or no renderer" << std::endl;
            }
        } else {
            std::cout << "No geometry renderer" << std::endl;
        }
    }
private:

    void initialize() {
        rootEntity = new Qt3DCore::QEntity;

        auto tmp = g->get_nodes_by_type(rgbd_type_str.data());
        if (!tmp.empty()) {
            cam = g->get_camera_api(tmp.at(0));
        } else {
            qFatal("Cannot find camera");
        }

        widget->setMinimumSize(QSize((int)cam->get_width(), (int)cam->get_height()));
        widget->setMaximumSize(QSize((int)cam->get_width(), (int)cam->get_height()));


        auto *lightEntity = new Qt3DCore::QEntity(rootEntity);
        auto *light = new Qt3DRender::QPointLight(lightEntity);
        light->setColor("white");
        light->setIntensity(0.6);
        lightEntity->addComponent(light);
        auto *lightTransform = new Qt3DCore::QTransform(lightEntity);
        lightTransform->setTranslation(QVector3D(1000, -1000.0, 0));
        lightEntity->addComponent(lightTransform);

        Qt3DRender::QCamera *camera = view->camera();
        camera->lens()->setPerspectiveProjection(55.0f, 16.0f/9.0f, 0.01f, 10000.0);
        camera->setPosition(QVector3D(0, -500.0, 500.0));
        camera->setViewCenter(QVector3D(0, 0, 0));


        auto mat_head = inner->get_transformation_matrix("world", "viriato_head_camera_sensor").value();

        auto tr_head = mat_head.translation();
        auto rot = mat_head.rotation();

        first_person_camera = new  Qt3DRender::QCamera(rootEntity);

        first_person_camera->lens()->setPerspectiveProjection(55.0f, cam->get_focal_x()/cam->get_focal_y(), 0.01f, 10000.0);
        first_person_camera->setPosition(QVector3D(tr_head.x()/10, tr_head.y()/10, tr_head.z()/10));
        first_person_camera->setViewCenter(QVector3D(tr_head.x()/10 + (tr_head.x()/10 > 0.0 ? 0.1 : -0.1 ), tr_head.y(), tr_head.z()/10 + (tr_head.z()/10 > 0.0 ? 0.1 : -0.1 )));

        Eigen::Matrix<float, 3, 3> mcopy = rot.cast<float>();
        QMatrix3x3 rot_(mcopy.data());

        first_person_camera->transform()->setRotation(QQuaternion::fromRotationMatrix(rot_));


        // Framegraph root node
        auto surfaceSelector = new Qt3DRender::QRenderSurfaceSelector();
        auto mainViewPort = new Qt3DRender::QViewport(surfaceSelector);

        //clear buffers
        auto clearBuffers = new Qt3DRender::QClearBuffers(mainViewPort);
        clearBuffers->setBuffers(Qt3DRender::QClearBuffers::ColorDepthBuffer);
        clearBuffers->setClearColor(Qt::white);
        [[maybe_unused]] auto noDraw = new Qt3DRender::QNoDraw(clearBuffers);

        // viewport
        auto viewPort1 = new Qt3DRender::QViewport(mainViewPort);
        viewPort1->setNormalizedRect(QRectF(0.0f, 0.0f, 1.0f, 1.0f));
        auto cameraSelector1 = new Qt3DRender::QCameraSelector(viewPort1);
        cameraSelector1->setCamera(/*camera*/ first_person_camera);


        view->setActiveFrameGraph(surfaceSelector);


        globalLayer = new Qt3DRender::QLayer(rootEntity);
        globalLayer->setRecursive(false);
        rootEntity->addComponent(globalLayer);

        view->renderSettings()->pickingSettings()->setPickMethod(Qt3DRender::QPickingSettings::PrimitivePicking);
        view->renderSettings()->pickingSettings()->setFaceOrientationPickingMode(Qt3DRender::QPickingSettings::FrontAndBackFace);
        //view->renderSettings()->pickingSettings()->setWorldSpaceTolerance(50.0);
        view->renderSettings()->pickingSettings()->setPickResultMode(Qt3DRender::QPickingSettings::NearestPick);

        for (auto i = 0; i < (int)(cam->get_width()* cam->get_height())/30 ;i++) {
            auto l_raycaster =  /*new Qt3DRender::QRayCaster(rootEntity); //*/new Qt3DRender::QScreenRayCaster(rootEntity);
            l_raycaster->setFilterMode(Qt3DRender::QAbstractRayCaster::AcceptAllMatchingLayers);
            l_raycaster->setRunMode(Qt3DRender::QAbstractRayCaster::SingleShot);
            l_raycaster->addLayer(globalLayer);
            l_raycaster->setObjectName(QString(("raycaster_" + std::to_string(i)).data()));
            rootEntity->addComponent(l_raycaster);

            static std::atomic_int counter = 0;
            connect(l_raycaster, &Qt3DRender::QRayCaster::hitsChanged, this,
                    [&, created_entities = std::unordered_set<std::tuple<float, float, float>, hash_tuple>{}, num_elements = 0, object_name = ("raycaster_" + std::to_string(i))](
                            auto hits) mutable {

                        std::vector<Eigen::Vector3f> points(hits.size());
                        std::vector<Qt3DCore::QEntity *> ents(hits.size());

                        counter++;

                        auto windowViewport = [](const QSize &area, const QRectF &relativeViewport) {
                            if (area.isValid()) {
                                const int areaWidth = area.width();
                                const int areaHeight = area.height();
                                return QRect(relativeViewport.x() * areaWidth,
                                             (1.0 - relativeViewport.y() - relativeViewport.height()) * areaHeight,
                                             relativeViewport.width() * areaWidth,
                                             relativeViewport.height() * areaHeight);
                            }
                            return relativeViewport.toRect();
                        };



                        //set = true;
                        auto topleft = widget->mapToGlobal(widget->geometry().topLeft());
                        auto bottomright = widget->mapToGlobal(widget->geometry().bottomRight());

                        //auto hstart = 0, wstart = 0;
                        auto hend = bottomright.x() - topleft.x(), wend = bottomright.y() - topleft.y();


                        QSize area(hend, wend);
                        QRect relativeViewport(0.0f, 0.0f, 1.0f, 1.0f);
                        relativeViewport = windowViewport(area, relativeViewport);

                        auto mt = first_person_camera->transform()->matrix();
                        QVector4D position = mt * QVector4D(0.0, 0.0, 0.0, 1.0);
                        QVector4D viewDirection = mt * QVector4D(0.0f, 0.0f, -1.0f, 0.0f);
                        QVector4D upVector = mt * QVector4D(0.0f, 1.0f, 0.0f, 0.0f);
                        QMatrix4x4 viewMatrix;
                        viewMatrix.lookAt(position.toVector3D(),
                                          (position + viewDirection).toVector3D(),
                                          upVector.toVector3D());

                        QVector3D nearPos(0.0, 0.0, 0.0);
                        nearPos = nearPos.unproject(viewMatrix,
                                                    first_person_camera->projectionMatrix(), QRect(0, 0, 1, 1));


                        //std::cout << hits.size() << std::endl;
                        for (const auto& i : hits) {

                            auto inter = i.worldIntersection();
                            std::cout << " Intersección en:  (" ") " << inter.x() << ", " << inter.y() << ", "
                                      << inter.z() << " Entity" << i.entity()->objectName().toStdString() << std::endl;

                            points.push_back(Eigen::Vector3f(inter.x(), inter.y(), inter.z()));
                            ents.push_back(i.entity());


                            QVector3D origin = nearPos;
                            QVector3D dest = inter;
                            //QVector3D direction = (inter - nearPos).normalized();
                            //size_t len = (inter - nearPos).length();



                            //std::cout << " DRAW LINE BETWEEN (" <<glCorrectPos.x() << " " << glCorrectPos.y() <<") " << np.x() << ", " << np.y() << ", "  << np.z() << " <|> " << fp.x() << ", " << fp.y() << ", "  << fp.z() << std::endl;
                            /*auto ent = */drawLine(origin, dest, QColor(0x0000ff), rootEntity);


                            if (not created_entities.contains(std::make_tuple(inter.x(), inter.y(), inter.z()))) {
                                num_elements += 1;
                                auto *Entity = new Qt3DCore::QEntity(rootEntity);

                                //Entity->setObjectName(QString(node.name().data()));
                                auto *material = new Qt3DExtras::QNormalDiffuseMapAlphaMaterial();
                                material->setDiffuse(gray);
                                material->setNormal(gray);
                                material->setAmbient(QColor(QRgb(0x00ff00)));
                                material->setSpecular(QColor(QRgb(0x00ff00)));
                                auto *sphMesh = new Qt3DExtras::QSphereMesh;

                                sphMesh->setRadius(1.5);


                                Entity->addComponent(sphMesh);

                                auto *planeTransform = new Qt3DCore::QTransform;
                                planeTransform->setScale3D(QVector3D(1.0, 1.0, 1.0));
                                planeTransform->setTranslation(QVector3D(inter.x(), inter.y(), inter.z()));

                                Entity->addComponent(material);
                                Entity->addComponent(planeTransform);


                                created_entities.insert(std::make_tuple(inter.x(), inter.y(), inter.z()));
                            }

                            break;
                        }

                        /*
                        auto dist = [](auto &a, auto &&b) -> double { return std::sqrt( (b.x() - a.x() )* (b.x() - a.x()) + std::hypot( b.y() - a.y(), b.z() - a.z()) ); };
                        auto[idx_vecs, dist_vecs]  = pc.KNN(points, 30);
                        for (size_t id1 = 0; id1 < idx_vecs.size(); id1++) {
                            for (size_t id2 = 0; id2 < idx_vecs[id1].size(); id2++) {
                                auto [val_x, val_y, val_z] = pc.get_pos(idx_vecs[id1][id2]);
                                std::cout << "i, j (" <<  ii << jj <<") idx: " << idx_vecs[id1][id2] <<
                                " intersected coord: "<< points[id1].x() << " " <<  points[id1].y() << " " << points[id1].z() <<
                                " nearest coord: "  <<  val_x << " " << val_y << " " << val_z <<
                                " dist: " << dist_vecs[id1][id2]  <<  " other dist " << dist( points[id1], Eigen::Vector3d(val_x, val_y, val_z) )<< std::endl;

                                if (auto[it, ok] = point_entity.emplace(idx_vecs[id1][id2],
                                                                        std::vector<std::pair<Qt3DCore::QEntity *, float>>{{ents[id1], dist_vecs[id1][id2]}}); !ok) {
                                    it->second.push_back({ents[id1], dist_vecs[id1][id2]});
                                }

                            }
                        }
                        */

                    }, Qt::DirectConnection);

            raycaster.push_back(l_raycaster);
        }
        gray = new Qt3DRender::QTextureLoader;
        gray->setSource(QUrl("qrc:/gris.png"));

        for (const auto& node : g->get_nodes_by_types({"plane", "mesh"}))
        {
            insert_node(node);
            //addGeomInfo(Entity);
        }


        //auto [vertices, indices] = geom_info.get_vertex_and_borders();

        connect(geom_info.get(), &GeomInfo::geometry_added,
                this, [this](auto entity)
                {
                    auto [positions, indices] = *geom_info->getQtGeom(entity);
                    for (auto i = 0; i < (int)indices.size() -1 ; i+=1 )
                    {

                        //auto [x1, y1, z1] = positions[indices[i]];
                        //auto [x2, y2, z2] = positions[indices[i+1]];

                        //auto orig = QVector3D(x1, y1, z1), dest = QVector3D(x2, y2, z2);

                        //std::cout << " DRAW LINE BETWEEN [" << x1 << " " <<  y1<< " " <<  z1 << "] and [" <<  x2 << " " <<  y2 << " " <<  z2 <<"]" <<std::endl;
                        //drawSphere(orig, 1.5, QColor(QRgb(0x00ff00)), rootEntity);
                        //drawSphere(dest, 1.5, QColor(QRgb(0xff0000)), rootEntity);
                        //drawLine(orig, dest, QColor(0x0000ff), rootEntity);
                    }

                    auto vertex_vec = geom_info->getGeomBbox(entity);
                    auto color = QColor(166, 232, 63);
                    for (auto [x, y, z] : vertex_vec)
                    {
                        drawSphere(QVector3D(x, y, z), 3, color, rootEntity);
                    }
                },
                Qt::QueuedConnection);

        view->setRootEntity(rootEntity);
    }

    void updated_node(uint64_t id, const std::string &type)
    {

        if (auto node = g->get_node(id); (type == "plane"sv || type == "mesh"sv) && node.has_value())
        {
            insert_node(node.value());
        } else {
            //TODO: remove
        }
    }

    void updated_node_attr(uint64_t id, const std::vector<std::string> &att_names)
    {
        //return;
        /*
        static bool set = false;
        static auto last = std::chrono::steady_clock::now();

        if (!set and std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last) >= 1000ms &&
            cam &&
            std::find(att_names.begin(), att_names.end(), cam_rgb_str) != att_names.end() ) {

            const auto strtglobal = std::chrono::steady_clock::now();

            auto pointcloud = cam->get_pointcloud("floor", 5);
            auto [img, rgb_width, rgb_height] = g->get_attribs_by_name<cam_rgb_att, cam_rgb_width_att, cam_rgb_height_att>(id);
            if (img.has_value() and rgb_width.has_value() and rgb_height.has_value() and pointcloud) {

                std::cout << "Elements: " << pointcloud->size() << std::endl;
                const auto strt = std::chrono::steady_clock::now();

                auto* pc_ptr = (float*) pointcloud->data();
                auto elements = pointcloud->size()*3;
                auto iters = elements / 8;

                const __m256 scalar = _mm256_set1_ps(10.0);

                for (auto idx = 0; idx < iters; idx+=8) {
                    __m256 mem = _mm256_loadu_ps(pc_ptr + idx);
                    __m256 result = _mm256_div_ps(mem, scalar);
                    _mm256_storeu_ps(pc_ptr + idx, result);
                }

                for (auto idx = iters; idx < elements ; idx++)
                {
                    *pc_ptr /= 10.0;
                }

                const auto nd1 = std::chrono::steady_clock::now();

                KdTree pc (std::move(pointcloud.value()));

                const auto nd = std::chrono::steady_clock::now();


                std::cout
                        << "kdtree created in "
                        << std::chrono::duration_cast<std::chrono::milliseconds>(nd - strt).count() << std::endl;


                auto s = 0;

                auto topleft = widget->mapToGlobal(widget->geometry().topLeft());
                auto bottomright = widget->mapToGlobal(widget->geometry().bottomRight());

                auto hstart = 0 , wstart=0;
                auto hend = bottomright.x() - topleft.x(), wend=  bottomright.y() - topleft.y() ;


                std::unordered_map<Qt3DRender::QBuffer*, QByteArray> cache;
                auto now = std::chrono::steady_clock::now();

                [[maybe_unused]] auto triangulo = 0;
                [[maybe_unused]] auto line = 0;
                [[maybe_unused]] auto punto = 0;
                [[maybe_unused]] auto enti = 0;

                if (!set){
                    auto i = 0;
                     for (auto ii = hstart; ii < hend; ii+=30) {
                    //for (auto ii = hend; hend > 0; hend-=4) {
                        for (auto jj = wstart; jj < wend; jj += 30) {
                        //for (auto jj = wend; wend > 0; wend -= 4) {


                            auto windowViewport = [](const QSize &area, const QRectF &relativeViewport)
                            {
                                if (area.isValid()) {
                                    const int areaWidth = area.width();
                                    const int areaHeight = area.height();
                                    return QRect(relativeViewport.x() * areaWidth,
                                                 (1.0 - relativeViewport.y() - relativeViewport.height()) * areaHeight,
                                                 relativeViewport.width() * areaWidth,
                                                 relativeViewport.height() * areaHeight);
                                }
                                return relativeViewport.toRect();
                            };



                            //set = true;
                            QSize area (hend, wend);
                            QRect relativeViewport(0.0f, 0.0f, 1.0f, 1.0f);
                            relativeViewport = windowViewport(area, relativeViewport);
                            const QPoint glCorrectPos = QPoint(ii, area.height() - jj); // In GL the y is inverted compared to Qt

                            auto mt = first_person_camera->transform()->matrix();
                            QVector4D position = mt * QVector4D(0.0, 0.0, 0.0, 1.0);
                            QVector4D viewDirection = mt * QVector4D(0.0f, 0.0f, -1.0f, 0.0f);
                            QVector4D upVector = mt * QVector4D(0.0f, 1.0f, 0.0f, 0.0f);
                            QMatrix4x4 viewMatrix;
                            viewMatrix.lookAt(position.toVector3D(),
                                     (position + viewDirection).toVector3D(),
                                     upVector.toVector3D());

                            QVector3D nearPos (glCorrectPos.x(), glCorrectPos.y(), 0.0);
                            nearPos = nearPos.unproject(viewMatrix,
                                                        first_person_camera->projectionMatrix(), relativeViewport);


                            QVector3D farPos (glCorrectPos.x(), glCorrectPos.y(), 1.0);
                            farPos = farPos.unproject(viewMatrix,
                                                      first_person_camera->projectionMatrix(), relativeViewport);


                            QVector3D origin = nearPos;
                            QVector3D dest = farPos;
                            QVector3D direction = (farPos - nearPos).normalized();
                            size_t len = (farPos - nearPos).length();


                            //raycaster->trigger(origin, direction, len);
                            //raycaster[i]->trigger(QPoint(ii, jj));


                            //std::cout << "hits: " << raycaster[i]->hits().size() << std::endl;

                            //std::cout << " DRAW LINE BETWEEN (" <<glCorrectPos.x() << " " << glCorrectPos.y() <<") " << np.x() << ", " << np.y() << ", "  << np.z() << " <|> " << fp.x() << ", " << fp.y() << ", "  << fp.z() << std::endl;
                            //auto ent = drawLine(origin, dest, QColor(0xff0000), rootEntity);

                            i++;
                            set = true;

                            //std::this_thread::sleep_for(15ms);
                            //goto loop1;
                            std::vector<uint> vertex_indices;
                            switch (i.type()) {
                                case Qt3DRender::QRayCasterHit::TriangleHit: {
                                    triangulo += 1;
                                    vertex_indices = {i.vertex1Index(), i.vertex2Index(), i.vertex3Index()};
                                    break;
                                }
                                case Qt3DRender::QRayCasterHit::LineHit: {
                                    line += 1;
                                    vertex_indices = {i.vertex1Index(), i.vertex2Index()};
                                    break;
                                }
                                case Qt3DRender::QRayCasterHit::PointHit:
                                    punto += 1;
                                    continue;
                                case Qt3DRender::QRayCasterHit::EntityHit: {
                                    enti += 1;
                                    QVector3D local_pos = i.localIntersection();
                                    continue;
                                }
                            }


                            Qt3DCore::QEntity *entity = i.entity();
                            auto components = entity->components();
                            auto c = std::find_if(components.begin(), components.end(),
                                                  [](auto &p) {
                                                      return qobject_cast<Qt3DRender::QGeometryRenderer *>(p);
                                                  });
                            auto renderer = qobject_cast<Qt3DRender::QGeometryRenderer *>(*c);
                            auto atts = renderer->geometry()->attributes();
                            auto colorAttribute =
                                    std::find_if(atts.begin(), atts.end(), [](Qt3DRender::QAttribute *&p) {
                                        return p->name() == Qt3DRender::QAttribute::defaultColorAttributeName();
                                    });


                            }





                        }
                    }
                   // std::cout << "Entidades creadas " << created_entities.size() << std::endl;


                }





                //std::cout << "By type : triangulos -> " << triangulo << " lineas -> " << line << " puntos -> " << punto << " entities -> "<< enti << std::endl;
                //std::cout << "Cache size: " << cache.size() << std::endl;
                //for (auto &[buff, bytearr] : cache){
                //    buff->setData(bytearr);
                //}

                //const auto end = std::chrono::steady_clock::now();
                //std::cout
                //        << "complete raycast took "
                //        << std::chrono::duration_cast<std::chrono::milliseconds>(end - now).count() << std::endl;  // using milliseconds and seconds accordingly

                //std::cout << "RAYCAST COLISIONES:  " << s << std::endl;

                //get entity renderer
                //get color attribute. se llama -> Qt3DCore::QAttribute::defaultColorAttributeName()
                //Cambiar los vértices que sean.
                //Cambiar o añadir material?
                //Qt3DRender::QMaterial *material = new Qt3DExtras::QPerVertexColorMaterial(rootEntity);
                //Bordear lo que se ve en la cámara del robot para otros viewports.

            }

            const auto endglobal = std::chrono::steady_clock::now();
            std::cout
                    << "raycast and kdtree done in "
                    << std::chrono::duration_cast<std::chrono::milliseconds>(endglobal - strtglobal).count() << std::endl;  // using milliseconds and seconds accordingly

        }*/
    }

    void updated_edge(uint64_t from, uint64_t to, const std::string &type)
    {

        if (auto node = g->get_node(to); type == "RT"sv  && node.has_value())
        {
            insert_node(node.value());
        } else {
            //TODO: remove
        }
    }


    void updated_edge_attr(uint64_t from, uint64_t to, const std::string &type,
                                 const std::vector<std::string> &att_name)
    {
        if (type == "RT"sv) {
            auto mat = inner->get_transformation_matrix("world", "viriato_head_camera_sensor").value();
            auto tr_head = mat.translation();
            auto rot = mat.rotation();

            if (!qFuzzyCompare(QVector3D(tr_head.x() / 10, tr_head.y() / 10, tr_head.z() / 10), first_person_camera->position()))
                first_person_camera->translateWorld(QVector3D(tr_head.x() / 10, tr_head.y() / 10, tr_head.z() / 10)- first_person_camera->position());

            Eigen::Matrix<float, 3, 3> mcopy = rot.cast<float>();
            QMatrix3x3 rot_(mcopy.data());


            auto euler = QQuaternion::fromRotationMatrix(rot_).toEulerAngles();
            //std::cout << first_person_camera->transform()->rotationX()  << " " << first_person_camera->transform()->rotationY() << " " << first_person_camera->transform()->rotationZ() << std::endl;

            first_person_camera->transform()->setRotationX(-(-euler.z() /*first_person_camera->transform()->rotationZ()*/ - 90));
            first_person_camera->transform()->setRotationY( euler.y() /*first_person_camera->transform()->rotationY()*/ + 90);
            first_person_camera->transform()->setRotationZ(90);
            //std::cout << "-------------------------------------\n";

        }
    }

    void deleted_edge(uint64_t from, uint64_t to, const std::string &edge_tag){

    }
    void deleted_node(uint64_t id){

    }

    void insert_node (const DSR::Node &node) {
        auto y = g->get_attrib_by_name<height_att>(node);
        auto x = g->get_attrib_by_name<width_att>(node);
        auto z = g->get_attrib_by_name<depth_att>(node);
        auto path = g->get_attrib_by_name<path_att>(node);

        auto mat = inner->get_transformation_matrix("world", node.name());

        if (((x.has_value() && y.has_value() && z.has_value() )|| path.has_value()) && mat.has_value()) {

            auto *Entity = new Qt3DCore::QEntity(rootEntity);

            Entity->setObjectName(QString(node.name().data()));
            auto *material = new Qt3DExtras::QNormalDiffuseMapAlphaMaterial();
            material->setDiffuse(gray);
            material->setNormal(gray);

            QVector3D scale (1, 1, 1);

            if (node.name() == "Wall9") return;
            if (node.name() == "floor") {
                material->setAmbient(QColor(QRgb(0x474747)));
                material->setSpecular(QColor(QRgb(0x474747)));
                material->setShininess(15.f);
            } else {

                material->setAmbient(QColor(QRgb(0xfcfcfc)));
                material->setSpecular(QColor(QRgb(0xfcfcfc)));
                material->setShininess(60.f);
            }

            auto plane_mesh = [&](auto& x, auto& y, auto& z) {

                //Qt3DExtras::QCuboidMesh *planeMesh = new Qt3DExtras::QCuboidMesh;

                auto *planeGeom = new QCuboidGeom();
                auto *planeMesh = new Qt3DRender::QMesh(rootEntity);

                connect(planeGeom, &QCuboidGeom::VertexBufferLoaded, this , [this, Entity]() {
                    addGeomInfo(Entity);
                });

                auto n_x = (*x / 10.f != 0) ?  *x / 10.f : 0.00001;
                auto n_y = (*y / 10.f != 0) ?  *y / 10.f : 0.00001;
                auto n_z = (*z / 10.f != 0) ?  *z / 10.f : 0.00001;

                planeGeom->setXExtent(n_x);
                planeGeom->setYExtent(n_y);
                planeGeom->setZExtent(n_z);
                planeGeom->setXYMeshResolution(QSize(2, 2));
                planeGeom->setXZMeshResolution(QSize(2, 2));
                planeGeom->setYZMeshResolution(QSize(2, 2));

                planeGeom->init();

                planeMesh->setGeometry(planeGeom);
                Entity->addComponent(planeMesh);

                Entity->addComponent(globalLayer);
                //Entity->addComponent(raycaster);


                //toadd.push_back(Entity);

            };
            if (auto [path, scalex, scaley, scalez] = g->get_attribs_by_name<DSR::Node, path_att, scalex_att, scaley_att, scalez_att>(node); path.has_value())
            {

                auto *mesh = new Qt3DRender::QMesh(rootEntity);

                connect(mesh, &Qt3DRender::QMesh::statusChanged, this , [this, Entity](Qt3DRender::QMesh::Status status) {
                    if (status == Qt3DRender::QMesh::Status::Ready) {
                        addGeomInfo(Entity);
                    }
                });
                QUrl meshpath;
                QString pathqstr (path->data());
                pathqstr.replace(".ive", ".obj");
                pathqstr.replace(".3ds", ".obj");
                QFileInfo check_file(pathqstr);
                pathqstr = check_file.canonicalFilePath();
                meshpath.setScheme("file");
                meshpath.setPath(pathqstr);
                mesh->setSource(meshpath);

                std::cout << pathqstr.toStdString() << ", children nodes=" << mesh->childNodes().count() << " file exist?: "<< std::boolalpha<< (check_file.exists() && check_file.isFile())  << std::endl;

                qWarning() << "Add mesh" ;
                if (!pathqstr.endsWith(".obj") )
                {
                    plane_mesh(x, y, z);
                }
                else if ( node.name() != "viriato_mesh"){
                    std::cout << "Set mesh. scale  " <<scalex.value_or(1.0)<< " " << scaley.value_or(1.0)<< " " << scalez.value_or(1.0) << std::endl;
                    Entity->addComponent(mesh);
                    //Entity->addComponent(raycaster);
                    Entity->addComponent(globalLayer);

                    scale = QVector3D(scalex.value_or(1.0)/10, scaley.value_or(1.0)/10, scalez.value_or(1.0)/10);
                    scale = QVector3D(scalex.value_or(1.0)/10, scaley.value_or(1.0)/10, scalez.value_or(1.0)/10);
                }

            } else {
                plane_mesh(x, y, z);
            }

            auto wire = new WireframeMaterial(rootEntity);
            wire->setAmbient(QColor( 255*0.2, 0.0, 0.0, 1.0 ));
            wire->setDiffuse(QColor( 255*0.8, 0.0, 0.0, 1.0 ));

            wire->setEffect(new WireEffect(rootEntity));

            Qt3DCore::QTransform *planeTransform = new Qt3DCore::QTransform;
            planeTransform->setScale3D(scale);

            //Sacar transformación de Eigen.

            auto tr = mat->translation();
            Eigen::Matrix<float, 3, 3> mcopy = mat->rotation().cast<float>();
            QMatrix3x3 rot(mcopy.data());


            planeTransform->setTranslation(QVector3D(tr.x() / 10, tr.y() / 10, tr.z() / 10));

            planeTransform->setRotation(QQuaternion::fromRotationMatrix(rot));

            Entity->addComponent(material);
            Entity->addComponent(planeTransform);
            Entity->addComponent(wire);


    }
}

bool only_one_widget;
DSR::DSRGraph *g; //We don't own this pointer.
std::unique_ptr<DSR::InnerEigenAPI> inner;
std::unique_ptr<DSR::CameraAPI> cam;
Qt3DExtras::Qt3DWindow *view; //We don't manage this pointer object as a widget will take it's ownership.
Qt3DCore::QEntity *rootEntity;
Qt3DRender::QCamera *first_person_camera;
Qt3DRender::QTextureLoader *gray;
Qt3DRender::QLayer* globalLayer;
std::vector<Qt3DRender::QScreenRayCaster *>raycaster;

QWidget * widget;

std::shared_ptr<GeomInfo> geom_info;

};

#endif //DSR_GRAPH_3D_VIEW_H
