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
ngl::Vec3 planeCenter=ngl::Vec3(0,0.01,0);//plane is defaulted on 0,0,0 at the moment, for demo purposes
ngl::Vec3 planeNormal=ngl::Vec3(0,1,0);//hence... plane's normal would be 0,1,0 at the moment, for demo purposes
ngl::Vec3 planeNormal2=ngl::Vec3(0,1,0);//hence... plane's normal would be 0,1,0 at the moment, for demo purposes

ngl::Vec3 ballcenterInit=ngl::Vec3(5,20,0);
ngl::Vec3 artificialMirrorBallvec((-5,20,0));//used to create the 2nd ground

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

ngl::Vec3 rotationAxis=0;

void NGLScene::initializeGL ()
{
    ngl::NGLInit::instance();
    glClearColor (0.4,0.4,0.4,1);
    std::cout<<"Initializing NGL\n";

    ngl::Vec3 from(-0,40,55);ngl::Vec3 to(0,0,0);ngl::Vec3 up(0,1,0);
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

    //Explixitly define the plane normal, based on the position of the ball
    planeNormal=ballcenterInit-planeCenter;
    planeNormal.normalize();

    //artificially specify planeNormal2 as the mirror of planeNormal in X axis
    artificialMirrorBallvec=ballcenterInit;
    artificialMirrorBallvec.m_x= - ballcenterInit.m_x;//ONLY MIRROR in X axis
    planeNormal2=artificialMirrorBallvec-planeCenter;;
    planeNormal2.normalize();



    myball.m_center=ballcenterInit;
    myball.m_radius=1;
    myball.m_velocity=ngl::Vec3(0.1,0,0);

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
    plane->createTrianglePlane ("ground2", 50, 50, 10, 10,10);//used to mirror the first one

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
    M=_transform.getMatrix ()*m_mouseGlobalTX;
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

float planeThetaCos;

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



    //rotate the plane according to the initial position of the ballcenterInit
//    planeThetaCos = atan2((planeCenter.dot(ballcenterInit)) / (planeCenter.length() *  ballcenterInit.length()))*(180/M_PI);

//    planeCenter.normalize();
//    ballcenterInit.normalize();
//    planeThetaCos = acos((planeCenter.dot(ballcenterInit)))*(180/M_PI);


//    ngl::Vec3 cross = (ballcenterInit.cross(planeCenter));
//    cross.normalize();
//    if (cross.dot( planeNormal) < 0) { // Or > 0
//      planeThetaCos = -planeThetaCos;
//    }

    

    
//    float sina = (ballcenterInit.cross(planeCenter)).length() / ( ballcenterInit.length() * planeCenter.length() );
//    float cosa = (ballcenterInit.dot(planeCenter) / ( ballcenterInit.length() * planeCenter.length() ));
//    float angle = atan2( sina, cosa )*(180/M_PI);




//    ballcenterInit.normalize();
//    planeCenter.normalize();
//    float angle = acos(ballcenterInit.dot(planeCenter))*(180/M_PI);

//    float sign = planeNormal.dot( ballcenterInit.cross(planeCenter) );
//    if(sign>0)
//    {
//        angle=-angle;
//    }





//    float dot = ballcenterInit.dot(planeCenter);
//    float lenSq1 = ballcenterInit.lengthSquared();
//    float lenSq2 = planeCenter.lengthSquared();
//    float angle = acos(dot/sqrt(lenSq1 * lenSq2));



//    float heading, attitude,bank;
//    double s=sin(angle);
//    double c=cos(angle);
//    double t=1-c;
//    //  if axis is not already normalised then uncomment this
//    // double magnitude = sqrt(x*x + y*y + z*z);
//    // if (magnitude==0) throw error;
//    // x /= magnitude;
//    // y /= magnitude;
//    // z /= magnitude;
//    if ((rotationAxis.m_x*rotationAxis.m_y*t + rotationAxis.m_z*s) > 0.998) { // north pole singularity detected
//        heading = 2*atan2(rotationAxis.m_x*sin(angle/2),cos(angle/2));
//        attitude = M_PI/2;
//        bank = 0;
//        return;
//    }
//    if ((rotationAxis.m_x*rotationAxis.m_y*t + rotationAxis.m_z*s) < -0.998) { // south pole singularity detected
//        heading = -2*atan2(rotationAxis.m_x*sin(angle/2),cos(angle/2));
//        attitude = -M_PI/2;
//        bank = 0;
//        return;
//    }
//    heading = atan2(rotationAxis.m_y * s- rotationAxis.m_x * rotationAxis.m_z * t , 1 - (rotationAxis.m_y*rotationAxis.m_y+ rotationAxis.m_z*rotationAxis.m_z ) * t);
//    attitude = asin(rotationAxis.m_x * rotationAxis.m_y * t + rotationAxis.m_z * s) ;
//    bank = atan2(rotationAxis.m_x * s - rotationAxis.m_y * rotationAxis.m_z * t , 1 - (rotationAxis.m_x*rotationAxis.m_x + rotationAxis.m_z*rotationAxis.m_z) * t);


//    m_transform.setRotation(bank*(180/M_PI),heading*(180/M_PI),attitude*(180/M_PI));



    //  Calculate the angle and the rotation axis between the 2 vectors,
    ngl::Vec3 v1=ballcenterInit;
    ngl::Vec3 v2=planeCenter;
    v1.normalize();
    v2.normalize();
    float angle = acos(v1.dot(v2));
    ngl::Vec3 rotationAxis = v1.cross(v2);
    rotationAxis.normalize();

    //create the translation and rotation matrices
    ngl::Mat4 rotateMat = 1;
    rotateMat=matrixFromAxisAngle(rotationAxis,angle);
    ngl::Mat4 translateMat = 1;
    //draw the ground on the origin
    translateMat.translate(planeCenter.m_x,planeCenter.m_y,planeCenter.m_z);

    //Multiply the matrices all together
    /*IMPORTANT NOTE: in NGL we need to specify the order of multiplication as we think it.
     * so rotate and then translate = rotateMat*translateMat, whereas usually in raw opengl or glm libray
     * we do it the other way around and so: rotate and then translate would equal to: translateMat*rotateMat
    */

    ngl::Mat4 finalMatrix=rotateMat*translateMat;//no scale for now, just don't care about it in this example

    //set the m_transform matrix to finalMatrix (extra step not usually needed-happens because of the way we call loadMatricesToShader(m_transform,...))
    m_transform.setMatrix(finalMatrix);

    loadMatricesToShader (m_transform,m_mouseGlobalTX, m_cam);
    shader->setShaderParam4f("Colour",0,1,0,1);
    prim->draw ("ground");


    //***********************Artificially negate the rotation of the 1st ground to create a 2nd ground //***********************
    v1=artificialMirrorBallvec;
    v2=planeCenter;
    v1.normalize();
    v2.normalize();
    angle = acos(v1.dot(v2));
    rotationAxis = v1.cross(v2);
    rotationAxis.normalize();
    rotateMat=matrixFromAxisAngle(rotationAxis,angle);
    finalMatrix=rotateMat*translateMat;//no scale for now, just don't care about it in this example

    //set the m_transform matrix to finalMatrix (extra step not usually needed-happens because of the way we call loadMatricesToShader(m_transform,...))
    m_transform.setMatrix(finalMatrix);

    loadMatricesToShader (m_transform,m_mouseGlobalTX, m_cam);
    shader->setShaderParam4f("Colour",0,0,1,1);
    prim->draw ("ground2");



    glEnable(GL_DEPTH_TEST);
    // enable multisampling for smoother drawing
    glEnable(GL_MULTISAMPLE);


}



ngl::Mat4 NGLScene::matrixFromAxisAngle(ngl::Vec3 axis, float angle) {

    ngl::Mat4 tmp=1;

    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;
    //  if axis is not already normalised then uncomment this
    // double magnitude = sqrt(axis.x*axis.x + axis.m_y*axis.m_y + axis.m_z*axis.m_z);
    // if (magnitude==0) throw error;
    // axis.x /= magnitude;
    // axis.m_y /= magnitude;
    // axis.m_z /= magnitude;

    tmp.m_00 = c + axis.m_x*axis.m_x*t;
    tmp.m_11 = c + axis.m_y*axis.m_y*t;
    tmp.m_22 = c + axis.m_z*axis.m_z*t;


    double tmp1 = axis.m_x*axis.m_y*t;
    double tmp2 = axis.m_z*s;
    tmp.m_10 = tmp1 + tmp2;
    tmp.m_01 = tmp1 - tmp2;
    tmp1 = axis.m_x*axis.m_z*t;
    tmp2 = axis.m_y*s;
    tmp.m_20 = tmp1 - tmp2;
    tmp.m_02 = tmp1 + tmp2;
    tmp1 = axis.m_y*axis.m_z*t;
    tmp2 = axis.m_x*s;
    tmp.m_21 = tmp1 + tmp2;
    tmp.m_12 = tmp1 - tmp2;

    return tmp;
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


    bool collidedwithPlane = Collision::SphereToPlane (myball, planeCenter, planeNormal);
    bool collidedwithPlane2 = Collision::SphereToPlane (myball, planeCenter, planeNormal2);




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
    ngl::Vec3 extraPushForce(0,-100,0.0);
    force.m_y= gravity * ballMass;

    //calculate acceleration
    ngl::Vec3 accel = (force+extraPushForce) / ballMass;
    std::cout<<accel.m_x<<accel.m_y<<accel.m_z<<std::endl;
    //calculate velocity Euler
    //    myball.m_velocity += accel * dt;

    //calculate velocity Velocity Verlet
    prevVel=myball.m_velocity;
    myball.m_velocity = myball.m_velocity + accel * dt;


    //check if collided collidedwithPlane
    if (collidedwithPlane)
        myball.m_velocity = calculateCollisionResponse(planeNormal);
    if (collidedwithPlane2)
        myball.m_velocity = calculateCollisionResponse(planeNormal2);


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

    //update ball position based on velocity (Velocity Verlet Integration)
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
float NGLScene::e=0.81; //when e=0 ->  collision is perfectly inelastic, when e=1 ->  collision is perfectly elastic;
ngl::Vec3 NGLScene::calculateCollisionResponse(const ngl::Vec3 & normal)
{
    float d = myball.m_velocity.dot (normal);
    float mag= - ( 1 + e ) * d;
    float j = MAX( mag, 0.0 );
    myball.m_velocity += j* normal;
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
