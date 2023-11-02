#pragma once

#include <reactphysics3d/reactphysics3d.h>
#include <maya/MQtUtil.h>
#include <QtWidgets/qdialog.h>
#include <QtWidgets/QVBoxLayout>
#include <QtOpenGL/qglfunctions.h>
#include <QtOpenGL/QGLWidget>

class DebugRenderer : public QGLWidget, protected QOpenGLFunctions {
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
