/*
* Created by aligolestaneh on Aug 27 2022
*
* @brief Utility function to build a IUST Quadruped object
*
*
* This file builds a model of The IUST robot.
* The inertia parameters of all bodies are determined from CAD by mohammadjabarzadeh.
*
*/

#ifndef IUST_CHEETAH_SOFTWARE_IUST_H
#define IUST_CHEETAH_SOFTWARE_IUST_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of IUST robot
 */
template<typename T>
Quadruped<T> buildIUST()
{
    Quadruped<T> iust;
    iust._robotType = RobotType::IUST;

    //Mass(kg)
    iust._bodyMass = 3.94; //Done
    iust._abadMass = 0.650; //Done
    iust._hipMass = 1.154; //Done
    iust._kneeMass = 0.209; //Done
    iust._rotorMass = 0.084; //NI

    //Dimentions(m)
    iust._bodyLength = 0.46; //0.269 * //Done
    iust._bodyWidth = 0.11; //0.19054 * //Done
    iust._bodyHeight = 0.095; //Done

    //Gear ratios
    iust._abadGearRatio = 6;
    iust._hipGearRatio = 6;
    iust._kneeGearRatio = 6;

    //Link lengths(m)
    iust._abadLinkLength = 0.0576;
    iust._hipLinkLength = 0.218;
    iust._kneeLinkY_offset = 0.07125;
    iust._kneeLinkLength = 0.240; // *
    iust._maxLegLength = 0.453;
    iust._hipRotorLocationYOffset = 0.0527;

    //Motor properties
    iust._motorTauMax = 8.f;
    iust._batteryV = 24;
    iust._motorKT = 0.08;   //(MiLab Team: this is flux linkage * pole pairs)
    iust._motorR = 0.13;
    iust._jointDamping = 0.02;
    iust._jointDryFriction = 0.4;

    //Rotor inertia (if the rotor is oriented so it spins around the z-axis)
    Mat3<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = rotorRotationalInertiaZ * 1e-6;

    Mat3 <T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3 <T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3 <T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3 <T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

    //Spatial inertia for abad
    Mat3<T> abadRotationalInertia;
    abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
    SpatialInertia<T> abadInertia(iust._abadMass, abadCOM, abadRotationalInertia);

    //Spatial inertia for hip
    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(0, 0.016, -0.02);
    SpatialInertia<T> hipInertia(iust._hipMass, hipCOM, hipRotationalInertia);

    //Spatial inertia for knee
    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    //kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0, 0, -0.061);
    SpatialInertia<T> kneeInertia(iust._kneeMass, kneeCOM, kneeRotationalInertiaRotated);

    //Rotor inertia (x-axis and y-axis)
    Vec3<T> rotorCOM(0, 0, 0);
    SpatialInertia<T> rotorInertiaX(iust._rotorMass, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(iust._rotorMass, rotorCOM, rotorRotationalInertiaY);

    //Body inertia
    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
    bodyRotationalInertia = bodyRotationalInertia * 1e-6;
    Vec3<T> bodyCOM(0, 0, 0);
    Vec3<T> bodyDims(iust._bodyLength, iust._bodyWidth, iust._bodyHeight);
    SpatialInertia<T> bodyInertia(iust._bodyMass, bodyCOM, rotInertiaOfBox(iust._bodyMass, bodyDims));
    
    //Adjust IUST inertias
    iust._abadInertia = abadInertia;
    iust._hipInertia = hipInertia;
    iust._kneeInertia = kneeInertia;
    iust._abadRotorInertia = rotorInertiaX;
    iust._hipRotorInertia = rotorInertiaY;
    iust._kneeRotorInertia = rotorInertiaY;
    iust._bodyInertia = bodyInertia;

    //Locations
    iust._abadRotorLocation = Vec3<T>(iust._bodyLength, iust._bodyWidth, 0) * 0.5;
    iust._abadLocation = Vec3<T>(iust._bodyLength, iust._bodyWidth, 0) * 0.5;
    iust._hipLocation = Vec3<T>(0, iust._abadLinkLength, 0);
    iust._hipRotorLocation = Vec3<T>(0, iust._abadLinkLength - iust._hipRotorLocationYOffset, 0);
    iust._kneeLocation = Vec3<T>(0, 0, -iust._hipLinkLength);
    iust._kneeRotorLocation = Vec3<T>(0, 0, 0);

    return iust;
}

#endif //IUST_H