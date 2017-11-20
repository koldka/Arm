#ifndef ARM_PROJECT_DISPLAYWIDGET_H
#define ARM_PROJECT_DISPLAYWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QString>
#include <QKeyEvent>

#include "geometryloader.h"

class DisplayWidget : public QOpenGLWidget, protected QOpenGLFunctions {

public:
    DisplayWidget(QString path, QWidget *parent = 0);
    ~DisplayWidget();

    void test();
    GeometryLoader *geometries;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void initShaders();

    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void keyPressEvent(QKeyEvent *e) override;
    void timerEvent(QTimerEvent *e) override;

private:
    QBasicTimer timer;
    QOpenGLShaderProgram program;

    QMatrix4x4 projection;

    QVector2D mousePressPosition;
    qreal angularSpeed;
    qreal zoom;
    QQuaternion rotation;
    float m_offsetX;
    float m_offsetY;
    QVector3D translation;

    QPoint lastPosition;

    QString objPath;
};


#endif //ARM_PROJECT_DISPLAYWIDGET_H
