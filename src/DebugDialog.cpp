#include "BulletOpenGLWidget.h"
#include <QOpenGLFunctions>

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
}

BulletDialog::BulletDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Bullet OpenGL"));
    glWidget = new BulletOpenGLWidget();
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(glWidget);
    setLayout(layout);
}

// This is a global function that creates and shows the dialog
void showBulletDialog() {
    // Parent the QDialog to Maya's main window
    QWidget* parent = MQtUtil::mainWindow();
    BulletDialog* dialog = new BulletDialog(parent);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->show();
}
