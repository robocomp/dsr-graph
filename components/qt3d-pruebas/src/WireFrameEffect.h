//
// Created by juancarlos on 17/9/21.
//

#ifndef QT3D_PRUEBAS_WIREFRAMEEFFECT_H
#define QT3D_PRUEBAS_WIREFRAMEEFFECT_H

#include <Qt3DRender>
#include <Qt3DCore>
#include <Qt3DExtras>

class WireEffect : public Qt3DRender::QEffect {
    Q_OBJECT
public:
    WireEffect(QNode *parent = nullptr)
    {
        auto ka = new Qt3DRender::QParameter();
        ka->setName("ka"); ka->setValue(QVector3D(0.1, 0.1, 0.1 ));
        addParameter(ka);
        auto kd = new Qt3DRender::QParameter();
        kd->setName("kd"); kd->setValue(QVector3D(0.7, 0.7, 0.7 ));
        addParameter(kd);
        auto ks = new Qt3DRender::QParameter();
        ks->setName("ks"); ks->setValue(QVector3D(0.95, 0.95, 0.95));
        addParameter(ks);
        auto shininess = new Qt3DRender::QParameter();
        shininess->setName("shininess"); shininess->setValue(150.0);
        addParameter(shininess);

        auto tec = new Qt3DRender::QTechnique;
        tec->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
        tec->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::CoreProfile);
        tec->graphicsApiFilter()->setMajorVersion(3);
        tec->graphicsApiFilter()->setMinorVersion(1);


        auto lp = new Qt3DRender::QParameter();
        lp->setName("light.position"); lp->setValue(QVector4D( 0.0, 0.0, 0.0, 1.0 ));
        addParameter(lp);
        auto li = new Qt3DRender::QParameter();
        li->setName("light.intensity"); li->setValue(QVector3D( 1.0, 1.0, 1.0  ));
        addParameter(li);
        auto lw = new Qt3DRender::QParameter();
        lw->setName("line.width"); lw->setValue(1.0);
        addParameter(lw);
        auto lc = new Qt3DRender::QParameter();
        lc->setName("line.color"); shininess->setValue(QVector4D( 1.0, 1.0, 1.0, 1.0 ));
        addParameter(lc);


        auto fkey = new Qt3DRender::QFilterKey;
        fkey->setName("renderingStyle"); fkey->setValue("forward");
        tec->addFilterKey(fkey);



        auto rpass = new Qt3DRender::QRenderPass;

        auto sp = new Qt3DRender::QShaderProgram;
        sp->setVertexShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl("qrc:/shaders/robustwireframe.vert")));
        sp->setGeometryShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl("qrc:/shaders/robustwireframe.geom")));
        sp->setFragmentShaderCode(Qt3DRender::QShaderProgram::loadSource(QUrl("qrc:/shaders/robustwireframe.frag")));
        rpass->setShaderProgram(sp);
        tec->addRenderPass(rpass);

        addTechnique(tec);
    };

    ~WireEffect() {};
};
#endif //QT3D_PRUEBAS_WIREFRAMEEFFECT_H
