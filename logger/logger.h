#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include "vectornav/vectornavprotocol.h"
extern QVector<double> K;

class Logger : public QObject
{
    Q_OBJECT
public:
    explicit Logger(QObject *parent = nullptr);

    signals:
public slots:
    void logStart();
    void logTick(DataFromVectorNav data);
    void logStop();
protected:
    QTextStream stream;
    QFile file;
    bool writeLog = false;
};

#endif // LOGGER_H