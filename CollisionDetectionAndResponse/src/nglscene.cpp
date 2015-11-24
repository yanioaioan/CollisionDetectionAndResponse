#include <QMouseEvent>
#include <QGuiApplication>
#include "nglscene.h"
#include <iostream>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <math.h>

//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for x/y translation with mouse movement
//----------------------------------------------------------------------------------------------------------------------
const static float INCREMENT=0.01;
//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for the wheel zoom
//----------------------------------------------------------------------------------------------------------------------
const static float ZOOM=0.1;
const static float dt=1/60.0f;
ngl::Vec3 planeCenter=ngl::Vec3(0,0,0);//plane is defaulted on 0,0,0 at the moment, for demo purposes
ngl::Vec3 planeNormal=ngl::Vec3(0,1,0);//hence... plane's normal would be 0,1,0 at the moment, for demo purposes
const static float gravity=-9.81;

NGLScene::NGLScene()
{
    setFocusPolicy(Qt::StrongFocus);//to make the keyevents respond

    // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
    m_rotate=false;
    // mouse rotation values set to 0
    m_spinXFace=0;
    m_spinYFace=0;
    updateTimer=startTimer(10);
    m_animate=true;
    startSimPressed=false;
}

NGLScene::~NGLScene()
{
    ngl::NGLInit *Init = ngl::NGLInit::instance();
    std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
//    Init->NGLQuit();
}


void NGLScene::initializeGL ()
{
    ngl::NGLInit::instance();
    glClearColor (0.4,0.4,0.4,1);
    std::cout<<"Initializing NGL\n";

    ngl::Vec3 from(0,1,35);ngl::Vec3 to(0,0,0);ngl::Vec3 up(0,1,0);
    m_cam = new ngl::Camera(from,to,up);
    m_cam->setShape(45,(float)720/576,0.05,350);

    // now to load the shader and set the values
    // grab an instance of shader manager
    ngl::ShaderLib *shader=ngl::ShaderLib::instance();
    (*shader)["nglDiffuseShader"]->use();

    shader->setShaderParam4f("Colour",1,0,0,1);
    shader->setShaderParam3f("lightPos",1,1,1);
    shader->setShaderParam4f("lightDiffuse",1,1,1,1);

    //create the ball and the ground
    createPrimitives();

    myball.m_center=ngl::Vec3(2,10,0);myball.m_radius=1;myball.m_velocity=ngl::Vec3(1,-1,0);

    glEnable(GL_DEPTH_TEST);
    // enable multisampling for smoother drawing
    glEnable(GL_MULTISAMPLE);

    // as re-size is not explicitly called we need to do this.
   glViewport(0,0,width(),height());
}

void NGLScene::resizeGL (QResizeEvent *_event )
{
    int w=_event->size().width();
    int h=_event->size().height();
    // set the viewport for openGL
    glViewport(0,0,w,h);
    // now set the camera size values as the screen size has changed
    m_cam->setShape(45,(float)w/h,0.05,350);
    update ();
}


void NGLScene::resizeGL (int _w, int _h )
{
    int w=_w*devicePixelRatio();
    int h=_h*devicePixelRatio();
    // set the viewport for openGL
    glViewport(0,0,w,h);
    // now set the camera size values as the screen size has changed
    m_cam->setShape(45,(float)w/h,0.05,350);
    update ();
}

void NGLScene::createPrimitives()
{
    ngl::VAOPrimitives *plane = ngl::VAOPrimitives::instance ();
    plane->createTrianglePlane ("ground", 50, 50, 10, 10,10);

    ngl::VAOPrimitives *sphere = ngl::VAOPrimitives::instance ();
    sphere->createSphere ("ball", 1,30);
}

///////////////////////////////////
void NGLScene::loadMatricesToShader(ngl::Transformation &_transform, const ngl::Mat4 &_globalTx, ngl::Camera *_cam) const
{
    ngl::ShaderLib *shader=ngl::ShaderLib::instance();
    (*shader)["nglDiffuseShader"]->use();

    ngl::Mat4 MV;
    ngl::Mat4 MVP;
    ngl::Mat3 normalMatrix;
    ngl::Mat4 M;
    M=m_mouseGlobalTX*_transform.getMatrix ();
    MV=  M*m_cam->getViewMatrix();
    MVP=  MV*m_cam->getProjectionMatrix();
    normalMatrix=MV;
    normalMatrix.inverse();
//    shader->setShaderParamFromMat4("MV",MV);
    shader->setShaderParamFromMat4("MVP",MVP);
    shader->setShaderParamFromMat3("normalMatrix",normalMatrix);
//    shader->setShaderParamFromMat4("M",M);
}

//float val=0.1;
//float start=0.0;
//float end=1.0;
//float interpVal=0.0;

void NGLScene::paintGL ()
{
    // clear the screen and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Rotation based on the mouse position for our global
    // transform
    ngl::Mat4 rotX;
    ngl::Mat4 rotY;
    // create the rotation matrices
    rotX.rotateX(m_spinXFace);
    rotY.rotateY(m_spinYFace);
    // multiply the rotations
    m_mouseGlobalTX=rotY*rotX;
    // add the translations
    m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
    m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
    m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;


    ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance ();
    ngl::ShaderLib *shader=ngl::ShaderLib::instance();
    (*shader)["nglDiffuseShader"]->use();

    //draw the ball according to its center
    m_transform.setPosition (myball.m_center);
    loadMatricesToShader (m_transform,m_mouseGlobalTX, m_cam);

    shader->setShaderParam4f("Colour",1,0,0,1);
    prim->draw ("ball");

    //draw the ground on the origin
    m_transform.setPosition (0,0,0);
    loadMatricesToShader (m_transform,m_mouseGlobalTX, m_cam);
    shader->setShaderParam4f("Colour",1,1,0,1);
    prim->draw ("ground");


    glEnable(GL_DEPTH_TEST);
    // enable multisampling for smoother drawing
    glEnable(GL_MULTISAMPLE);


}


void NGLScene::testButtonClicked(bool b)
{
    emit clicked (b);
    std::cout<<"Button Clicked - manual signal-slot connection"<<std::endl;
    m_rColor=1;
    startSimPressed=true;
    update ();
}

const static float acceleration=0;
void NGLScene::updateBall()
{
    //update velocity based on new velocity calculated
//    myball.m_velocity += acceleration*dt;

    //recalculate velocity if ball collided to the ground collided
    bool collided = Collision::SphereToPlane (myball, planeCenter, planeNormal);



    //    Accel = F / mass,
    //    Vel += Accel * deltaTime,
    //    Pos = Vel * time.

    float ballMass=1;
    float planeMass=10;
    ngl::Vec3 prevVel=0;
    ngl::Vec3 relativeVel=0;
    ngl::Vec3 prevBallPos=0;
    ngl::Vec3 prevAccel=0;



    //calculate force
    ngl::Vec3 force;
    force.m_y= gravity * ballMass;
    //calculate acceleration
    ngl::Vec3 accel = force / ballMass;
    std::cout<<accel.m_x<<accel.m_y<<accel.m_z<<std::endl;
    //calculate velocity Euler
    //    myball.m_velocity += accel * dt;

    //calculate velocity Velocity Verlet
    prevVel=myball.m_velocity;
    myball.m_velocity = myball.m_velocity + accel * dt;


    //check if collided
    if (collided)
        myball.m_velocity = calculateCollisionResponse();


    //FRICTION CALCULATION

//    //calculate relative velocity
//    relativeVel=myball.m_velocity-prevVel;
//    ngl::Vec3 tangentVector= relativeVel - relativeVel.dot(planeNormal)*planeNormal;
//    tangentVector.normalize();

//    // find magnitude
//    float jt = -relativeVel.dot(tangentVector);
//    jt = jt / (1 / ballMass + 1 / planeMass);

//    //calculate friction

//    //Save previous velocity
//    prevVel = myball.m_velocity;




    //update ball position based on velocity (Forward Euler Integration)
    //    myball.m_center+= myball.m_velocity *dt;

    //update ball position based on velocity (Velocity Verletr Integration)
    myball.m_center = myball.m_center + (prevVel + myball.m_velocity) * 0.5 * dt;






}


/*
 * Based on the following formula: j = -( 1 + e ) \ p*n
Where:

j is the magnitude of the collision impulse
e is the coefficient of restitution [0,1]
p is the linear momentum of the go stone
n in the contact normal for the collision
 */

#define MAX(x,y) (((x) < (y)) ? (y) : (x))
float NGLScene::e=0.5; //when e=0 ->  collision is perfectly inelastic, when e=1 ->  collision is perfectly elastic;
ngl::Vec3 NGLScene::calculateCollisionResponse()
{
    float d = myball.m_velocity.dot (planeNormal);
    float mag= - ( 1 + e ) * d;
    float j = MAX( mag, 0.0 );
    myball.m_velocity += j* planeNormal;
    return myball.m_velocity;
}

void NGLScene::timerEvent( QTimerEvent *_event )
{
    if(_event->timerId() == updateTimer)
    {
//        if (m_animate !=true)
//        {
//            return;
//        }

        if (startSimPressed)
        {
            updateBall();
        }

        update ();
    }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseMoveEvent (QMouseEvent * _event)
{
  // note the method buttons() is the button state when event was called
  // this is different from button() which is used to check which button was
  // pressed when the mousePress/Release event is generated
  if(m_rotate && _event->buttons() == Qt::LeftButton)
  {
    int diffx=_event->x()-m_origX;
    int diffy=_event->y()-m_origY;
    m_spinXFace += (float) 0.5f * diffy;
    m_spinYFace += (float) 0.5f * diffx;
    m_origX = _event->x();
    m_origY = _event->y();
    update ();
  }
        // right mouse translate code
  else if(m_translate && _event->buttons() == Qt::RightButton)
  {
    int diffX = (int)(_event->x() - m_origXPos);
    int diffY = (int)(_event->y() - m_origYPos);
    m_origXPos=_event->x();
    m_origYPos=_event->y();
    m_modelPos.m_x += INCREMENT * diffX;
    m_modelPos.m_y -= INCREMENT * diffY;
    update();

   }
}


//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mousePressEvent ( QMouseEvent * _event)
{
  // this method is called when the mouse button is pressed in this case we
  // store the value where the maouse was clicked (x,y) and set the Rotate flag to true
  if(_event->button() == Qt::LeftButton)
  {
    m_origX = _event->x();
    m_origY = _event->y();
    m_rotate =true;
  }
  // right mouse translate mode
  else if(_event->button() == Qt::RightButton)
  {
    m_origXPos = _event->x();
    m_origYPos = _event->y();
    m_translate=true;
  }

}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseReleaseEvent ( QMouseEvent * _event )
{
  // this event is called when the mouse button is released
  // we then set Rotate to false
  if (_event->button() == Qt::LeftButton)
  {
    m_rotate=false;
  }
        // right mouse translate mode
  if (_event->button() == Qt::RightButton)
  {
    m_translate=false;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::wheelEvent(QWheelEvent *_event)
{

    // check the diff of the wheel position (0 means no change)
    if(_event->delta() > 0)
    {
        m_modelPos.m_z+=ZOOM;
    }
    else if(_event->delta() <0 )
    {
        m_modelPos.m_z-=ZOOM;
    }
    update();
}
//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
//  case Qt::Key_Up    : m_plane->tilt(1.0,1,0); break;
//  case Qt::Key_Down  : m_plane->tilt(-1.0,1,0); break;
  case Qt::Key_Left  :
  {
//          val-=0.1;
//          interpVal=start+(end-start)*val;
//          std::cout<<interpVal<<std::endl;

      break;
  }

  case Qt::Key_Right :
  {
//          val+=0.1;
//          interpVal=start+(end-start)*val;
//          std::cout<<interpVal<<std::endl;

      break;
  }
  default : break;
  }
  // finally update the GLWindow and re-draw
//  if (isExposed())
    update();
}
