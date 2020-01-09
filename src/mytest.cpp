#include "iostream"
#include "reactphysics3d.h"
#include "math.h"
#include "Vector3D.hpp"

using namespace reactphysics3d;


int main()
{
    // Create the collision world
    rp3d::CollisionWorld world;
        
    // Initial position and orientation of the rigid body
    rp3d::Vector3 initPosition (0.0, 0.0, 0.0) ;
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);
    
    // Create a rigid body in the world
    rp3d::CollisionBody* body;
    body = world.createCollisionBody(transform);
    
    // Transform to the required position in the world
    // Center of the building is on 5 x-axis, 5 y-axis, 0.5 z-axis, 0.5 is suitable so that the ground plane of the building will be on the z=0 plane.
    rp3d::Vector3 NewPosition (5.0, 5.0, 0.5) ;
    rp3d::Quaternion NewOrientation = rp3d::Quaternion::identity();
    rp3d::Transform Newtransform(NewPosition, NewOrientation);
    body->setTransform(Newtransform);
   
    // Rigid body dimensions 
    rp3d::Vector3 halfExtents(0.5, 0.5, 0.5);
    const rp3d::BoxShape mycube(halfExtents);
    rp3d::decimal mass = rp3d::decimal(100.0);
    rp3d::ProxyShape* myproxyShape;
    myproxyShape = body->addCollisionShape((CollisionShape*) &mycube, transform);

    // Create the ray
    rp3d::Vector3 startPoint(6.15, 4.75, 1);
    rp3d::Vector3 endPoint(2.4, 6.61, 0.5);
    Ray my_ray(startPoint, endPoint);

    // Create the geometry vectors
    Vector3D <float> P1, P2, P3;

    // Create the raycast info object for the raycast result
    RaycastInfo raycastInfo;
   
    // Raycast
    if (myproxyShape->raycast(my_ray, raycastInfo))
    {   
        std::cout << " " << std::endl;
        std::cout << "Hit Detected" << std::endl;
        std::cout << raycastInfo.worldPoint.x <<" "<< raycastInfo.worldPoint.y <<" "<<raycastInfo.worldPoint.z << std::endl;
        std::cout << "" << std::endl;
        // std::cout << raycastInfo.worldPoint.length() << std::endl;
        // std::cout << raycastInfo.hitFraction << std::endl;
        // std::cout << raycastInfo.body->getTransform().getPosition().x << std::endl;
        // std::cout << raycastInfo.body->getTransform().getPosition().y << std::endl;
        // std::cout << raycastInfo.body->getTransform().getPosition().z << std::endl;
        // std::cout << "" << std::endl;
        std::cout <<"Plane X: "<< raycastInfo.worldNormal.x << std::endl;
        std::cout <<"Plane Y: "<< raycastInfo.worldNormal.y << std::endl;
        // std::cout << raycastInfo.worldNormal.z << std::endl;
        std::cout << "" << std::endl;

        if (raycastInfo.worldNormal.y == 1) {
            // Normal vector to the plane of hit
            float NorVec1X = 0, NorVec1Y = 1, NorVec1Z = 0;
            // Vectors
            P1.x = startPoint.x; P1.y = startPoint.y; P1.z = startPoint.z;
            P2.x = raycastInfo.worldPoint.x; P2.y = raycastInfo.worldPoint.y; P2.z = raycastInfo.worldPoint.z;
            P3.x = NorVec1X + raycastInfo.worldPoint.x; P3.y = NorVec1Y + raycastInfo.worldPoint.y; P3.z = NorVec1Z + raycastInfo.worldPoint.z;
        }
        else if (raycastInfo.worldNormal.y == -1){
            // Normal vector to the plane of hit
            float NorVec1X = 0, NorVec1Y = -1, NorVec1Z = 0;
            // Points
            P1.x = startPoint.x; P1.y = startPoint.y; P1.z = startPoint.z;
            P2.x = raycastInfo.worldPoint.x; P2.y = raycastInfo.worldPoint.y; P2.z = raycastInfo.worldPoint.z;
            P3.x = NorVec1X + raycastInfo.worldPoint.x; P3.y = NorVec1Y + raycastInfo.worldPoint.y; P3.z = NorVec1Z + raycastInfo.worldPoint.z;
        }

        else if (raycastInfo.worldNormal.x == 1) {
            // Normal vector to the plane of hit
            float NorVec1X = 1, NorVec1Y = 0, NorVec1Z = 0;
            // Points
            P1.x = startPoint.x; P1.y = startPoint.y; P1.z = startPoint.z;
            P2.x = raycastInfo.worldPoint.x; P2.y = raycastInfo.worldPoint.y; P2.z = raycastInfo.worldPoint.z;
            P3.x = NorVec1X + raycastInfo.worldPoint.x; P3.y = NorVec1Y + raycastInfo.worldPoint.y; P3.z = NorVec1Z + raycastInfo.worldPoint.z;
        }
        else if (raycastInfo.worldNormal.x == -1) {
            // Normal vector to the plane of hit
            float NorVec1X = -1, NorVec1Y = 0, NorVec1Z = 0;
            // Points
            P1.x = startPoint.x; P1.y = startPoint.y; P1.z = startPoint.z;
            P2.x = raycastInfo.worldPoint.x; P2.y = raycastInfo.worldPoint.y; P2.z = raycastInfo.worldPoint.z;
            P3.x = NorVec1X + raycastInfo.worldPoint.x; P3.y = NorVec1Y + raycastInfo.worldPoint.y; P3.z = NorVec1Z + raycastInfo.worldPoint.z;
        }
        // Create the vectors from points
        Vector3D <float> NewPoint1 = P2-P1;
        Vector3D <float> NewPoint2 = P2-P3;
        // We are using cos x = A.B/|A|x|B|, where A.B is the dot product of vector A, and B, |A| or |B| is the magnitude
        float DotProduct = NewPoint1.x*NewPoint2.x + NewPoint1.y*NewPoint2.y + NewPoint1.z*NewPoint2.z;
        float Mag_NewPoint1 = sqrt(pow(NewPoint1.x,2) + pow(NewPoint1.y,2) + pow(NewPoint1.z,2));
        float Mag_NewPoint2 = sqrt(pow(NewPoint2.x,2) + pow(NewPoint2.y,2) + pow(NewPoint2.z,2));
        // Calculate the angle
        float AngleOfHit = (180/M_PI)*acos(DotProduct/(Mag_NewPoint1*Mag_NewPoint2));
        std::cout <<"Angle of Hit: "<<AngleOfHit<<std::endl;
    }

    else
    {
        std::cout << "No Hit Detected!" << std::endl;
    }
}