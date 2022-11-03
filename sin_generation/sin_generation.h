#ifndef SIN_GENERATION_H
#define SIN_GENERATION_H

#include "kx_pult/qkx_coeffs.h"
#include "cs_rov.h"
#include <QObject>
#include "pult_connection/pultcontrolsystemprotocols.h"
#include "vma/vmacontroller.h"

class sin_generation : public QObject
{
     Q_OBJECT
public:
    explicit sin_generation(QObject *parent = nullptr);
signals:
    void startup_sin_mode();
public slots:
    void sin_generate();
protected:
};

#endif // SIN_GENERATION_H
