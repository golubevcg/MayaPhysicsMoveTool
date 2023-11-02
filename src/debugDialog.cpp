#include <reactphysics3d/reactphysics3d.h>
#include <maya/MQtUtil.h>
#include <QtWidgets/qdialog.h>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QOpenGLWidget>
#include <QtGui/QOpenGLFunctions>
#include <GL/gl.h>
#include <QtGui/qopenglext.h>

#include "debugDialog.h"

DebugRenderer::DebugRenderer(rp3d::PhysicsWorld* physicsWorld, QWidget* parent)
    : QOpenGLWidget(parent),
    physicsWorld(physicsWorld) {
    physicsWorld->setIsDebugRenderingEnabled(true);
    rp3d::DebugRenderer& debugRenderer = physicsWorld->getDebugRenderer();
    debugRenderer.setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::CONTACT_POINT, true);
    debugRenderer.setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::CONTACT_NORMAL, true);
}

void DebugRenderer::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void DebugRenderer::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void DebugRenderer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    rp3d::DebugRenderer& debugRenderer = physicsWorld->getDebugRenderer();
    rp3d::uint32 nbLines = debugRenderer.getNbLines();
    const rp3d::DebugRenderer::DebugLine* lines = debugRenderer.getLinesArray();

    for (rp3d::uint32 i = 0; i < nbLines; ++i) {
        const rp3d::DebugRenderer::DebugLine& line = lines[i];
        int color = line.color1;
        float r = ((color >> 16) & 0xFF) / 255.0f;
        float g = ((color >> 8) & 0xFF) / 255.0f;
        float b = (color & 0xFF) / 255.0f;
        glColor3f(r, g, b);
        glBegin(GL_LINES);
        glVertex3f(line.point1.x, line.point1.y, line.point1.z);
        glVertex3f(line.point2.x, line.point2.y, line.point2.z);
        glEnd();
    }
}

DebugDialog::DebugDialog(rp3d::PhysicsWorld* physicsWorld, QWidget* parent)
    : QDialog(parent) {
    QVBoxLayout* layout = new QVBoxLayout(this);
    DebugRenderer* debugRenderer = new DebugRenderer(physicsWorld, this);
    layout->addWidget(debugRenderer);
}
