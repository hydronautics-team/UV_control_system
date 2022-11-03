#include "sin_generation.h"
#include <QObject>

sin_generation::sin_generation(QObject *parent)
    : QObject{parent}{

}

void sin_generation::sin_generate() {
   double U0 = K[301]; // постоянная составляющая напряжения
   double A = K[302]; // амплитуда синуса
   double w = K[303]; // частота гармонического сигнала
   double k = K[304]; // дискретное время
   double h = K[305]; // шаг
   Upsi = U0 + A*sin(w*k*h);
}

