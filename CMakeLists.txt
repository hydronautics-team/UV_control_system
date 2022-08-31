cmake_minimum_required (VERSION 3.5)
project (UV_control_system VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_C_COMPILER iccarm)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=c++0x")
#set(CMAKE_EXE_LINKER_FLAGS "-lc++abi")

#set(CMAKE_CXX_COMPILER "clang++")
#DCMAKE_CXX_FLAGS="-stdlib=libc++"
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets SerialPort REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Gui Widgets SerialPort Network Test REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include_vn100")
include_directories("${PROJECT_SOURCE_DIR}/kx_pult")
include_directories("${PROJECT_SOURCE_DIR}/pult_connection")

set(PROJECT_SOURCES
        main.cpp

        include_vn100/vn100.c
        include_vn100/vncp_services.c

        kx_pult/kx_protocol.cpp
        kx_pult/qkx_coeffs.cpp
        kx_pult/qpiconfig.cpp

        pult_connection/uv_state.cpp

        #----------------------------------
        include_vn100/vectornav.h
        include_vn100/vectornavprotocol.h
        include_vn100/vn_common.h
        include_vn100/vn_errorCodes.h
        include_vn100/vn_kinematics.h
        include_vn100/vn_linearAlgebra.h
        include_vn100/vn_math.h
        include_vn100/vncp_services.h

        kx_pult/kx_protocol.h
        kx_pult/qkx_coeffs.h
        kx_pult/qpiconfig.h

        pult_connection/pultcontrolsystemprotocols.h
        pult_connection/udp_protocol.h
        pult_connection/uv_state.h

        )

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(UV_control_box
            MANUAL_FINALIZATION
            ${PROJECT_SOURCES}
            )
    # Define target properties for Android with Qt 6 as:
    #    set_property(TARGET untitled APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
    #                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
    # For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(UV_control_box SHARED
                ${PROJECT_SOURCES}
                )
        # Define properties for Android with Qt 5 after find_package() calls as:
        #    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(UV_control_box
                ${PROJECT_SOURCES}
                )
    endif()
endif()

target_link_libraries(UV_control_box PRIVATE
        sfml-graphics sfml-window sfml-system
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
        Qt${QT_VERSION_MAJOR}::Network
        )

set_target_properties(UV_control_box PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        )

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(UV_control_box)
endif()
