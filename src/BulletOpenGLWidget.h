#ifndef BULLET_OPENGL_WIDGET_H
#define BULLET_OPENGL_WIDGET_H

#include <MayaIncludes.h>
#include <QtWidgets/qdialog.h>
#include <QtWidgets/qopenglwidget.h>
#include <QtWidgets/qboxlayout.h>
#include "btBulletDynamicsCommon.h"


class BulletOpenGLWidget : public QOpenGLWidget {
private:
    explicit BulletOpenGLWidget(QWidget* parent = nullptr);
    BulletOpenGLWidget(const BulletOpenGLWidget&) = delete;
    BulletOpenGLWidget& operator=(const BulletOpenGLWidget&) = delete;
    btDiscreteDynamicsWorld* m_bulletWorld;

public:
    static BulletOpenGLWidget* getInstance(QWidget* parent = nullptr) {
        static BulletOpenGLWidget* instance = nullptr;
        if (!instance) {
            instance = new BulletOpenGLWidget(parent ? parent : MQtUtil::mainWindow());
        }
        return instance;
    }

    void setBulletWorld(btDiscreteDynamicsWorld* world) {
        m_bulletWorld = world;
    }

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
