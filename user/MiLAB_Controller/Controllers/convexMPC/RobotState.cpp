#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt yaw_)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->yaw = yaw_;

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r_[rs*4 + c];

    R = this->q.toRotationMatrix();
    fpt yc = cos(yaw_);
    fpt ys = sin(yaw_);


    if (true)
    {
        const fpt ps = sin(pitch_ascension1);
        const fpt pc = cos(pitch_ascension1);
        R_yaw <<  yc*pc,  -ys,   yc*ps,
                    ys*pc,  yc,   ys*ps,
                    -ps,   0,   pc;
        //std::cout << "ascending+++++++++++ " << R_yaw << std::endl;
    }
    else if (false)
    {
        const fpt ps = sin(pitch_descension1);
        const fpt pc = cos(pitch_descension1);
        R_yaw <<  yc*pc,  -ys,   yc*ps,
                    ys*pc,  yc,   ys*ps,
                    -ps,   0,   pc;

        //std::cout << "descending++++++++++++ " << R_yaw << std::endl;
    }
    
    else
    { 
        R_yaw <<  yc,  -ys,   0,
                  ys,  yc,   0,
                   0,   0,   1;
        //std::cout << "ground+++++++++++++ " << R_yaw << std::endl;
    }
    //std::cout << "se++++++++++++++++++++++++++++ " << R_yaw << std::endl;

    Matrix<fpt,3,1> Id_mini,Id_milab,Id_cheetah3;
    Id_mini << .07f, 0.26f, 0.242f;
    Id_cheetah3 << 0.41f, 2.1f, 2.1f;
//    Id_milab << 0.0996f, 0.765f, 0.765f;//25.7kg
    Id_milab <<  0.02289f, 0.23901f, 0.24209f;//13kg of IUST
//    Id_milab << 0.1084f, 0.834f, 0.834f;//28kg
    I_body_mini.diagonal() = Id_mini;
    I_body_milab.diagonal() = Id_milab;
    I_body_cheetah3.diagonal() = Id_cheetah3;
    //TODO: Consider normalizing quaternion??
}

void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_yaw<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl<<endl;
}



