//
// Created by juancarlos on 17/9/21.
//

#ifndef QT3D_PRUEBAS_WIREFRAMEMAT_H
#define QT3D_PRUEBAS_WIREFRAMEMAT_H

#include <Qt3DRender>
#include <Qt3DCore>
#include <Qt3DExtras>

class WireframeMaterialPrivate;

class WireframeMaterial : public Qt3DRender::QMaterial
{
    Q_OBJECT

    Q_PROPERTY(QColor ambient READ ambient WRITE setAmbient NOTIFY ambientChanged)
    Q_PROPERTY(QColor diffuse READ diffuse WRITE setDiffuse NOTIFY diffuseChanged)
    Q_PROPERTY(QColor specular READ specular WRITE setSpecular NOTIFY specularChanged)
    Q_PROPERTY(float shininess READ shininess WRITE setShininess NOTIFY shininessChanged)
    Q_PROPERTY(float alpha READ alpha WRITE setAlpha NOTIFY alphaChanged)
    Q_PROPERTY(float lineWidth READ lineWidth WRITE setLineWidth NOTIFY lineWidthChanged)
    Q_PROPERTY(QColor lineColor READ lineColor WRITE setLineColor NOTIFY lineColorChanged)
public:

    WireframeMaterial(QNode *parent = nullptr)
    {
        auto ka = new Qt3DRender::QParameter();
        ka->setName("ka"); ka->setValue(QVector3D(m_ambient.redF(), m_ambient.greenF(), m_ambient.blueF()));
        addParameter(ka);
        auto kd = new Qt3DRender::QParameter();
        kd->setName("kd"); kd->setValue(QVector3D(m_diffuse.redF(), m_diffuse.greenF(), m_diffuse.blueF()));
        addParameter(kd);
        auto ksp = new Qt3DRender::QParameter();
        ksp->setName("ksp"); ksp->setValue(QVector3D(m_specular.redF(), m_specular.greenF(), m_specular.blueF()));
        addParameter(ksp);
        auto shininess = new Qt3DRender::QParameter();
        shininess->setName("shininess"); shininess->setValue(m_shininess);
        addParameter(shininess);
        auto linewidth = new Qt3DRender::QParameter();
        linewidth->setName("line.width"); linewidth->setValue(m_lineWidth);
        addParameter(linewidth);
        auto linecolor = new Qt3DRender::QParameter();
        linecolor->setName("line.color"); linecolor->setValue(m_lineColor);
        addParameter(linecolor);
    };
    ~WireframeMaterial() {};

    QColor ambient() const   { return m_ambient; };
    QColor diffuse() const   { return m_diffuse; };
    QColor specular() const  { return m_specular; };
    float shininess() const  { return m_shininess; };
    float alpha() const      { return m_alpha; };
    float lineWidth() const  { return m_lineWidth; };
    QColor lineColor() const { return m_lineColor; };

public Q_SLOTS:
    void setAmbient(const QColor &ambient)     { m_ambient = ambient; emit ambientChanged(ambient); };
    void setDiffuse(const QColor &diffuse)     { m_diffuse = diffuse; emit diffuseChanged(diffuse); };
    void setSpecular(const QColor &specular)   { m_specular = specular; emit specularChanged(specular); };
    void setShininess(float shininess)         { m_shininess = shininess; emit shininessChanged(shininess); };
    void setAlpha(float alpha)                 { m_alpha = alpha; emit alphaChanged(alpha); };
    void setLineColor(const QColor &lineColor) { m_lineColor = lineColor; emit lineColorChanged(lineColor); };
    void setLineWidth(float lineWidth)         { m_lineWidth = lineWidth; emit lineWidthChanged(lineWidth); };

Q_SIGNALS:
    void ambientChanged(const QColor &ambient);
    void diffuseChanged(const QColor &diffuse);
    void specularChanged(const QColor &specular);
    void shininessChanged(float shininess);
    void alphaChanged(float alpha);
    void lineColorChanged(const QColor &lineColor);
    void lineWidthChanged(float lineWidth);

private:

    QColor m_ambient = QColor(255*0.05, 255*0.05, 255*0.05);
    QColor m_diffuse = QColor(255*0.7, 255*0.7, 255*0.7);
    QColor m_specular = QColor(255*0.95, 255*0.95, 255*0.95);
    float m_shininess = 150.0f;
    float m_alpha = 1.0f;
    float m_lineWidth = 0.8f;
    QColor m_lineColor = QColor(0, 0, 0);

    };
#endif //QT3D_PRUEBAS_WIREFRAMEMAT_H
