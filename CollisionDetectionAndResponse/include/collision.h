#ifndef COLLISION_H
#define COLLISION_H

#include <ngl/Vec3.h>

class Collision
{


public:

    struct Sphere
    {
        Sphere( ngl::Vec3 _center, float _radius, ngl::Vec3 _velocity) : m_center(_center), m_radius(_radius), m_velocity(_velocity)
        {}
        Sphere() {}


        ngl::Vec3 m_center;
        float m_radius;
        ngl::Vec3 m_velocity;
    };

    static bool SphereToPlane(const Sphere& sphere, const ngl::Vec3& planePoint, const ngl::Vec3& planePointNormal)
    {
        //Calculate a vector from the point on the plane to the center of the sphere
        ngl::Vec3 vecTemp(sphere.m_center - planePoint);

        //Calculate the distance: dot product of the new vector with the plane's normal
        float fDist= vecTemp.dot(planePointNormal);

        if(fDist > sphere.m_radius)
        {
            //The sphere is not touching the plane
            return false;
        }

        //Else, the sphere is colliding with the plane
        return true;
    }

    Collision();
    ~Collision();

};

#endif // COLLISION_H
