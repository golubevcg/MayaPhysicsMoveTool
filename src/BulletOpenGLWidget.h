#ifndef BULLET_OPENGL_WIDGET_H
#define BULLET_OPENGL_WIDGET_H

#include <MayaIncludes.h>
#include <QtWidgets/qdialog.h>
#include <QtWidgets/qopenglwidget.h>
#include <QtWidgets/qboxlayout.h>

class BulletOpenGLWidget : public QOpenGLWidget {
public:
    explicit BulletOpenGLWidget(QWidget* parent = nullptr);
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
};

class BulletDialog : public QDialog {
public:
    explicit BulletDialog(QWidget* parent = nullptr);
private:
    BulletOpenGLWidget* glWidget;
};

void showBulletDialog();

#endif // BULLET_OPENGL_WIDGET_H
