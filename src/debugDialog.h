#pragma once

#include <reactphysics3d/reactphysics3d.h>
#include <maya/MQtUtil.h>
#include <QtWidgets/qdialog.h>
#include <QtWidgets/QVBoxLayout>
#include <QtOpenGL/qglfunctions.h>
#include <QtGui/QOpenGLFunctions>
#include <QtWidgets/QOpenGLWidget>
#include <GL/gl.h>
#include <QtGui/qopenglext.h>


class DebugRenderer : public QOpenGLWidget, protected QOpenGLFunctions {  // Updated inheritance
public:
    DebugRenderer(rp3d::PhysicsWorld* physicsWorld, QWidget* parent = nullptr);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    rp3d::PhysicsWorld* physicsWorld;
};

class DebugDialog : public QDialog {
public:
    DebugDialog(rp3d::PhysicsWorld* physicsWorld, QWidget* parent = nullptr);
};
