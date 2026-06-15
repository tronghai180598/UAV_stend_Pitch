/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef _Kalman_h_
#define _Kalman_h_

// Фильтр Калмана для угла pitch (1D: угол + смещение gyro).
// Настройка: KQang (Q), KQbias, KRmeas (R) — через Flash и Qt.

class Kalman {
public:
    Kalman();

    // newAngle — acc (рад), newRate — gyro (рад/с), dt — с
    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle); // начальный угол при смене режима
    float getRate(); // оценка скорости без смещения

    void setQangle(float Q_angle);  // шум процесса угла
    void setQbias(float Q_bias);    // шум смещения gyro (по умолчанию 0.003)
    void setRmeasure(float R_measure); // шум измерения acc

    float getQangle();
    float getQbias();
    float getRmeasure();

private:
    float Q_angle;   // дисперсия шума процесса угла
    float Q_bias;    // дисперсия шума смещения gyro
    float R_measure; // дисперсия шума измерения acc

    float angle; // оценка угла (состояние фильтра)
    float bias;  // оценка смещения gyro
    float rate;  // скорость без смещения

    float P[2][2]; // ковариация ошибки 2×2
};

#endif
