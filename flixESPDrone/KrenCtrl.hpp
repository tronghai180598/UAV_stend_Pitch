#ifndef __KrenCtrl_HPP
#define __KrenCtrl_HPP

#include "KrenMdl.hpp"

class KrenCtrl : public KrenMdl {
public:
  float Us=0, Us_prev=0, Ui=0, mdVi=0, Uv=0, Um=0, Umm=0, erVi1=0;
  float muMd = 0;
  float uMold = 0;
  float klmf, /// коэффициент калмана углом
        klmv, /// коэффициент калмана скоростью угла
        Kpf, /// коэффициент в контуре управления углом
        Kpv, /// коэффициент в контуре управления скоростью изменения угла
        Kdv, /// коэффициент дифференцирования в контуре управления скоростью изменения угла
        mTmu,/// время инерции контура по модульному оптимуму
        mTe, /// время интергратора  ограничивающего скорость изменения управляющего сигнала ESC
        mTi; /// Время интегрирования контура по углу
  void UdateKalman(float Fi, float Vi);
  void setCtrlParam();
  KrenCtrl(float Tf, float Tv, float Tm, float Tmi);
  KrenCtrl();
  float updateCtrl(float dt, float setFi, float Fi, float Vi);
  float GetUi();
  void reset();
};

#endif /* __KrenCtrl_HPP */
