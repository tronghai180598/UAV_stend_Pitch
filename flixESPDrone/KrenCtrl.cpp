#include "KrenCtrl.hpp"
#include <string>
#include <math.h>
#include <algorithm>

#define _slCtrl_2PD 1
#define __SelCtrl 1

KrenCtrl::KrenCtrl(float Tf, float Tv, float Tm, float Tmi){
  mTi = 0.2;
  mTe = 0.04; // инициализация времени интегратора
  setParam(Tf, Tv, Tm);
  mTmu = mTm;
  setCtrlParam();
}

KrenCtrl::KrenCtrl(){
  klmf = klmv = 0.1;
  mTi = 0.25;
  mTe = 0.1;
  mTmu = 0.05;
  mTm = 0.05;
  setCtrlParam();
}
void KrenCtrl::reset()
{
    Us = 0.0f;
    Uv = 0.0f;
    Ui = 0.0f;
    Um = 0.0f;
    Umm = 0.0f;
    erVi1 = 0.0f;
    Us_prev = 0.0f;
    mdVi  = 0.0f;
    muMd  = 0.0f;
    uMold = 0.0f;
    mFi = 0.0f;
    mVi = 0.0f;
}

void KrenCtrl::setCtrlParam(){
#if (__SelCtrl==_slCtrl_2PD)
  Kpv = mTe*mTv/(25.0*mTmu*mTmu);
  Kdv = mTe*mTv/(2.0*mTmu); // SO 4--5
  Kpf = mTf / (15 * mTmu); // MO  15-30
/* -Fi       -Vi        Us
    |   ___   |   ___   |      ___    _______       _______ Ui Um 
Fs--O--|Kpf|--O--|KpV|--O--O--|sat|--|1/(sTe)|--*--|1/(sTi)|--O-->u
       |___|Vs | |___|  |  |  |___|  |_______|  |Uv|_______|  |  
              _|_____   |  \--------------------*-------------/1  
             |s Kdv  |  |           
             |1+s Tmu|--/
             |_______|
*/
#endif
}

void KrenCtrl::UdateKalman(float Fi, float Vi){
  mFi += (Fi - mFi) * klmf;
  mVi += (Vi - mVi) * klmv;
}

float KrenCtrl::updateCtrl(float dt, float setFi, float Fi, float Vi){
  UdateKalman(Fi, Vi);  
  float erFi = setFi - mFi;
  float erVi = Kpf * erFi  - mVi;
  rldiff(dt, erVi - erVi1, 1.0, mTmu, mdVi); erVi1 = erVi;
#if (__SelCtrl==_slCtrl_2PD)
  Us =  erVi * Kpv + mdVi * Kdv;
  Us -= Uv;
#endif

saturate(Us, -1000.0f, 1000.0f);

integr(dt, Us, mTe, Uv);
saturate(Uv, -1000.0f, 1000.0f);

float Ui_before = Ui;
integr(dt, Uv, mTi, Ui);

float Ui_sat = Ui;
saturate(Ui_sat, -1000.0f, 1000.0f);
if ((Ui_sat != Ui) && ((Ui > 0 && Uv > 0) || (Ui < 0 && Uv < 0))) {
    Ui = Ui_before;
} else {
    Ui = Ui_sat;
}

Um = Ui + Uv;
saturate(Um, -1000.0f, 1000.0f);
  intert(dt, Um,1.0,mTmu,Umm);
  rldiff(dt, Umm-uMold, mTmu, mTmu, muMd); uMold = Umm;
  updateMdl(dt, muMd);
  return Um;
}

float KrenCtrl::GetUi() { return Um; }
