#ifndef BULLET_OPENGL_WIDGET_H
#define BULLET_OPENGL_WIDGET_H

#include <MayaIncludes.h>

#include <QQDialog>
#include <QOpenGLWidget>
#include <QVBoxLayout>
#include <maya/MQtUtil.h>

class BulletOpenGLWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit BulletOpenGLWidget(QWidget* parent = nullptr);
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
};

class BulletDialog : public QDialog {
    Q_OBJECT
public:
    explicit BulletDialog(QWidget* parent = nullptr);
private:
    BulletOpenGLWidget* glWidget;
};

#endif // BULLET_OPENGL_WIDGET_H
