#include "KrenMdl.hpp"
#include <string>
#include <math.h>

void integr(float dt, float inp, float tm, float& out){  if(tm > dt) out += inp * dt / tm; else out=0; };
void intert(float dt, float inp, float k, float tm, float& out){  
  if(tm > dt) out += (inp * k - out) * dt / tm; 
  else out = inp * k;  // если время инерции меньше или равно времени шага, то это пропорциональное звено без динамики
};
void rldiff(float dt, float dinp, float tdf, float tm, float& out){ if(tm > dt) out += dinp * tdf/tm - out * dt / tm; };
void saturate(float& val, float min, float max){  val = (val < min)?min:(val>max?max:val); }
void saturatePi(float& val){
  while(val > 3142) val-=6284;
  while(val < -3142) val+=6284;
}
float my_rand(float min, float max){ return ( rand() * (max - min)) / RAND_MAX + min; }

KrenMdl::KrenMdl(float Tf, float Tv, float Tm) { mTf = Tf; mTv = Tv; mTm = Tm; mFi = 0; mVi = 0; mAc = 0; };
KrenMdl::KrenMdl() {  mTf = 1; mTv = 0.05; mTm = 0.05; mFi = 0; mVi = 0; mAc = 0;};

void KrenMdl::setParam (float Tf, float Tv, float Tm) { mTf = Tf; mTv = Tv; mTm = Tm;}

/* without SinNoise
    ________1,2   ______     ______
_u_|___1____|_dv_|__1___|_V_|__1___|_Fi_
   |(1+s Tm)|    |(s Tv)|   |(s Tf)|
   |________|    |______|   |______|
*/
float KrenMdl::updateMdl(float dt, float inp) {
  saturate(inp, -1000.0,1000.0);
  intert(dt, inp, 1, mTm, mPreAc);
  intert(dt, mPreAc, 1, dt, mAc);
  integr(dt, mAc, mTv, mVi);
  saturate(mVi, -32767,32767);
  integr(dt, mVi, mTf, mFi);
  saturate(mFi, -3142,3142);
  return mFi;
};
char* KrenMdl::print(char* msg) {
  sprintf(msg, " %f %f %f", getFi()/3142*180, getVi()/3142*18, mAc / 10);
  return msg;
};
float KrenMdl::getFi() {return mFi;}
float KrenMdl::getVi() {return mVi;}
float KrenMdl::getTr() {return mAc;}
