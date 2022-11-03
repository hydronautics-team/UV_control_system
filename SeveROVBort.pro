QT += core network
QT += serialport

CONFIG += c++11

TARGET = SeveROVBort
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app


SOURCES += *.cpp\
          vectornav/vectornavprotocol.cpp\
          logger/logger.cpp\
          vma/vmacontroller.cpp\
          kx_pult/kx_protocol.cpp\
          kx_pult/qkx_coeffs.cpp\
          kx_pult/qpiconfig.cpp\
          pult_connection/uv_state.cpp

HEADERS += *.h\
          vectornav/vectornavprotocol.h\
          logger/logger.h\
          vma/vmacontroller.h\
          kx_pult/kx_protocol.h\
          kx_pult/qkx_coeffs.h\
          kx_pult/qpiconfig.h\
          pult_connection/pultcontrolsystemprotocols.h\
          pult_connection/udp_protocol.h\
          pult_connection/uv_state.h

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
