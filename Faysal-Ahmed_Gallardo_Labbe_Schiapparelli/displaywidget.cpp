#include "displaywidget.h"
#include <QMouseEvent>
#include <iostream>
#include <iomanip>

#include "mainwindow.h"
#include "ui_mainwindow.h"

DisplayWidget::DisplayWidget(QString path, QWidget *parent) :
        QOpenGLWidget(parent)
{
    zoom = 0.1;

    this->objPath = path;

    m_offsetX = 0;
    m_offsetY = 0;
    setFocusPolicy(Qt::StrongFocus);
}

DisplayWidget::~DisplayWidget()
{
    makeCurrent();
    delete geometries;
    doneCurrent();
}

void
DisplayWidget::mousePressEvent(QMouseEvent *e)
{
    if(e->buttons() == Qt::LeftButton){
        lastPosition = e->pos();
    }
    // Save mouse press position
    mousePressPosition = QVector2D(e->localPos());
}

void
DisplayWidget::mouseReleaseEvent(QMouseEvent *e)
{
    // Mouse release position - mouse press position
    QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;

    // Accelerate angular speed relative to the length of the mouse sweep
    qreal acc = diff.length() / 100.0;
    // Increase angular speed
    angularSpeed += acc;
}

void
DisplayWidget::mouseMoveEvent(QMouseEvent *e)
{
    int x = e->x() - lastPosition.x();
    int y = e->y() - lastPosition.y();

    if(e->buttons() == Qt::LeftButton)
    {
        // Calculate new rotation axis as weighted sum
        rotation = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), x)
                * QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), y)
                *rotation;
        lastPosition = e->pos();

    }
}

void
DisplayWidget::wheelEvent(QWheelEvent *event)
{
    if (event->delta() > 0)
        zoom *= 2;
    else
        zoom /= 2;

    if (zoom <= 0.01)
        zoom = 0.01;

    if (zoom >= 10)
        zoom = 10;

    update();

    event->accept();
}

void DisplayWidget::keyPressEvent(QKeyEvent *e){
    switch(e->key()){
    case Qt::Key_Z:
        m_offsetY -= .5f;
        break;

    case Qt::Key_Q:
        m_offsetX += .5f;
        break;

    case Qt::Key_S:
        m_offsetY += .5f;
        break;

    case Qt::Key_D:
        m_offsetX -= .5f;
        break;
    case Qt::Key_E:
        geometries->setEdge();
        break;
    }

    update();
    e->accept();
}

void
DisplayWidget::timerEvent(QTimerEvent *)
{
    // Decrease angular speed (friction)
    angularSpeed *= 0.99;

    // Stop rotation when speed goes below threshold
    if (angularSpeed < 0.01) {
        angularSpeed = 0.0;
    } else {
        // Update rotation
        update();
    }
}

void DisplayWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    initShaders();

    test();

    glDisable(GL_CULL_FACE);


    geometries = new GeometryLoader(this->objPath);

    // Use QBasicTimer because its faster than QTimer
    timer.start(12, this);
}

void DisplayWidget::test()
{
    //If surfacic
    if ( ((MainWindow *)this->parent()->parent()->parent())->isSurfacique() )
    {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    }
    else //If volumic
    {
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
}

void DisplayWidget::initShaders()
{
    // Compile vertex shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vertex.glsl"))
        close();

    // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fragment.glsl"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();
}

void DisplayWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const qreal zNear = 0.01, zFar = 1000.0, fov = 45.0;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}

void DisplayWidget::paintGL()
{
    //projection.scale(0.01);
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Calculate model view transformation
    QMatrix4x4 matrix;
    matrix.translate(-m_offsetX, -m_offsetY, -5.0);
    matrix.rotate(rotation);
    matrix.scale(zoom);

    // Set modelview-projection matrix
    program.setUniformValue("mvp_matrix", projection * matrix);

    // Draw cube geometry

    int alpha;

    if ( ((MainWindow *)this->parent()->parent()->parent())->isSurfacique() ) {

        ((MainWindow *)this->parent()->parent()->parent())->setSliderValue(100);
        alpha = ((MainWindow *)this->parent()->parent()->parent())->getSliderValue();
    }


    else if ( ((MainWindow *)this->parent()->parent()->parent())->isVolumique() ) {

        ((MainWindow *)this->parent()->parent()->parent())->setSliderValue(10);
        alpha = ((MainWindow *)this->parent()->parent()->parent())->getSliderValue();
    }

    else
        alpha = ((MainWindow *)this->parent()->parent()->parent())->getSliderValue();


    geometries->drawGeometry(&program, alpha);
}
