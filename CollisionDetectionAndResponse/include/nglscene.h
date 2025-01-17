#ifndef NGLSCENE_H
#define NGLSCENE_H

#include <ngl/Vec3.h>
#include <ngl/Mat4.h>
#include <ngl/Camera.h>
#include <ngl/Transformation.h>
#include "collision.h"
#include <QOpenGLWidget>

class NGLScene: public QOpenGLWidget
{
    Q_OBJECT
public:
    NGLScene();
    ~NGLScene();

    float m_rColor;
    ngl::Transformation m_transform;
    Collision::Sphere myball;
    static float e;
    bool startSimPressed;

protected:
    void initializeGL ();
    void resizeGL (QResizeEvent *_event);
    void resizeGL (int _w, int _h );
    void loadMatricesToShader(ngl::Transformation &_transform,const ngl::Mat4 &_globalTx, ngl::Camera *_cam) const;
    void paintGL ();    
    void createPrimitives();
    void updateBall();
    ngl::Vec3 calculateCollisionResponse(const ngl::Vec3 &normal);


    void timerEvent( QTimerEvent *_event );
    //----------------------------------------------------------------------------------------------------------------------
    void mouseMoveEvent (QMouseEvent * _event);
    //----------------------------------------------------------------------------------------------------------------------
    void mousePressEvent ( QMouseEvent * _event);
    //----------------------------------------------------------------------------------------------------------------------
    void mouseReleaseEvent ( QMouseEvent * _event );
    //----------------------------------------------------------------------------------------------------------------------
    void wheelEvent(QWheelEvent *_event);
    //----------------------------------------------------------------------------------------------------------------------

    void keyPressEvent(QKeyEvent *_event);

private:

    //----------------------------------------------------------------------------------------------------------------------
    /// @brief used to store the x rotation mouse value
    //----------------------------------------------------------------------------------------------------------------------
    int m_spinXFace;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief used to store the y rotation mouse value
    //----------------------------------------------------------------------------------------------------------------------
    int m_spinYFace;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief flag to indicate if the mouse button is pressed when dragging
    //----------------------------------------------------------------------------------------------------------------------
    bool m_rotate;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief flag to indicate if the Right mouse button is pressed when dragging
    //----------------------------------------------------------------------------------------------------------------------
    bool m_translate;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the previous x mouse value
    //----------------------------------------------------------------------------------------------------------------------
    int m_origX;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the previous y mouse value
    //----------------------------------------------------------------------------------------------------------------------
    int m_origY;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the previous x mouse value for Position changes
    //----------------------------------------------------------------------------------------------------------------------
    int m_origXPos;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the previous y mouse value for Position changes
    //----------------------------------------------------------------------------------------------------------------------
    int m_origYPos;
    //----------------------------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the model position for mouse movement
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Vec3 m_modelPos;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief used to store the global mouse transforms
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_mouseGlobalTX;
    int updateTimer;
    /// @brief flag to indicate if animation is active or not
    bool m_animate;
    ngl::Camera *m_cam;

    ngl::Mat4 matrixFromAxisAngle(ngl::Vec3 axis, float angle);





signals:
    void clicked(bool);

public slots:
  void testButtonClicked(bool);

};

#endif // NGLSCENE_H
