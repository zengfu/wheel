#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "quaternion.h"

#define pi 3.1415926
//x->y->z
void euler_toRotation(float* euler,float *r)
{
  float roll,pitch,yaw;
  //roll for x
  //pitch for y
  //yaw for z
  roll=euler[0];
  pitch=euler[1];
  yaw=euler[2];
  
  r[0]=arm_cos_f32(pitch)*arm_cos_f32(yaw);
  r[1]=arm_sin_f32(roll)*arm_sin_f32(pitch)*arm_cos_f32(yaw)-arm_cos_f32(roll)*arm_sin_f32(yaw);  
  r[2]=arm_cos_f32(roll)*arm_sin_f32(pitch)*arm_cos_f32(yaw)+arm_sin_f32(roll)*arm_sin_f32(yaw);
  
  r[3]=arm_cos_f32(pitch)*arm_sin_f32(yaw);
  r[4]=arm_sin_f32(roll)*arm_sin_f32(pitch)*arm_sin_f32(yaw)+arm_cos_f32(roll)*arm_cos_f32(yaw);  
  r[5]=arm_cos_f32(roll)*arm_sin_f32(pitch)*arm_sin_f32(yaw)-arm_sin_f32(roll)*arm_cos_f32(yaw);
  
  r[6]=-arm_sin_f32(pitch);
  r[7]=arm_sin_f32(roll)*arm_cos_f32(pitch);
  r[8]=arm_cos_f32(roll)*arm_cos_f32(pitch);
}
void euler_fromRotation(float* euler,float *r)
{
  float roll,pitch,yaw;
  if(r[6]==1)//
  {
    pitch=-pi/2.0;
    yaw=0;//gimbal lock
    roll=-yaw+atan2(-r[1],-r[2]);
  }
  else if(r[6]==-1)
  {
    pitch=pi/2.0;
    yaw=0;//gimbla lock
    roll=yaw+atan2(r[1],r[2]);
  }
  else
  {
    pitch=-asin(r[6]);//maybe 2*pi-pitch
    yaw=atan2(r[3],r[0]);
    pitch=atan2(r[7],r[8]);
  }
  euler[0]=roll;
  euler[1]=pitch;
  euler[2]=yaw;
  
}
void quat_fromRotation(float* q,float *r)
{
  float trace = r[0] + r[4] + r[8];
  float s;
  if(trace > 0)
  {
          s = 0.5f * sqrt(trace + 1.0f);
          q[0] = 0.25f / s;
          q[1] = (r[7] - r[5]) * s;
          q[2] = (r[2] - r[6]) * s;
          q[3] = (r[3] - r[1]) * s;
  }
  else
  {
          if(r[0] > r[4] && r[0] > r[8] )
          {
                  s = 0.5f / sqrt(1.0f + r[0] - r[4] - r[8]);
                  q[0] = (r[7] - r[5]) * s;
                  q[1] = 0.25f / s;
                  q[2] = (r[1] + r[3]) * s;
                  q[3] = (r[2] + r[6]) * s;
          }
          else if(r[4] > r[8]) 
          {
                  s = 0.5f / sqrt(1.0f + r[4] - r[0] - r[8]);
                  q[0] = (r[2] - r[6]) * s;
                  q[1] = (r[1] + r[3]) * s;
                  q[2] = 0.25f / s;
                  q[3] = (r[5] + r[7]) * s;
          }
          else
          {
                  s = 0.5f / sqrt(1.0f + r[8] - r[0] - r[4]);
                  q[0] = (r[3] - r[1]) * s;
                  q[1] = (r[2] + r[6]) * s;
                  q[2] = (r[5] + r[7]) * s;
                  q[3] = 0.25f / s;
          }
  }
}
void quat_toRotation(float* q,float *r)
{
  float qw,qx,qy,qz;
  qw=q[0];
  qx=q[1];
  qy=q[2];
  qz=q[3];
  
  r[0] = 1 - 2*qy*qy - 2*qz*qz;
  r[1] = 2*qx*qy - 2*qz*qw;
  r[2] = 2*qx*qz + 2*qy*qw;
  r[3]= 2*qx*qy + 2*qz*qw;
  r[4] = 1 - 2*qx*qx - 2*qz*qz;
  r[5] = 2*qy*qz - 2*qx*qw;
  r[6] = 2*qx*qz - 2*qy*qw;
  r[7] = 2*qy*qz + 2*qx*qw;
  r[8] = 1 - 2*qx*qx - 2*qy*qy;
}


void quat_fromEuler(float* q,float* euler)
{
 
  float fCosHroll = arm_cos_f32(euler[0]* .5f);
  float fSinHroll = arm_sin_f32(euler[0] * .5f);
  float fCosHPitch = arm_cos_f32(euler[1]* .5f);
  float fSinHPitch = arm_sin_f32(euler[1] * .5f);
  float fCosHYaw = arm_cos_f32(euler[2] * .5f);
  float fSinHYaw = arm_sin_f32(euler[2]* .5f);
  
  q[0] = fCosHroll * fCosHPitch * fCosHYaw + fSinHroll * fSinHPitch * fSinHYaw;
  q[1] = fSinHroll * fCosHPitch * fCosHYaw - fCosHroll * fSinHPitch * fSinHYaw;
  q[2] = fCosHroll * fSinHPitch * fCosHYaw + fSinHroll * fCosHPitch * fSinHYaw;
  q[3] = fCosHroll * fCosHPitch * fSinHYaw - fSinHroll * fSinHPitch * fCosHYaw;
}


void quat_toEuler(float*q,float* euler)
{
  euler[0]=atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
  euler[1]=asin(2*(q[0]*q[2]-q[1]*q[3]));
  euler[2]=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
}