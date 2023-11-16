#include "BulletOpenGLWidget.h"
//#include <QOpenGLFunctions>

#include <QtOpenGL/qglfunctions.h>

BulletOpenGLWidget::BulletOpenGLWidget(QWidget* parent) : QOpenGLWidget(parent) {
    // Set OpenGL version info
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    setFormat(format);
}

void BulletOpenGLWidget::initializeGL() {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    // Initialize your Bullet world and OpenGL resources here
}

void BulletOpenGLWidget::resizeGL(int w, int h) {
    // Update viewport and projection matrix here
}

void BulletOpenGLWidget::paintGL() {
    QOpenGLFunctions* f = QOpenGLContext::currentContext()->functions();
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render your Bullet world here
    if (m_bulletWorld) {
        // Iterate over Bullet objects and render them
        for (int i = 0; i < m_bulletWorld->getNumCollisionObjects(); ++i) {
            btCollisionObject* obj = m_bulletWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) {
                // Extract position, orientation, etc., and render
            }
        }
    }
}

BulletDialog::BulletDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Bullet OpenGL"));
    glWidget = BulletOpenGLWidget::getInstance(this); // Get the singleton instance
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(glWidget);
    setLayout(layout);
}

// This is a global function that creates and shows the dialog
void showBulletDialog() {
    BulletDialog* dialog = new BulletDialog();
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->show();
}


