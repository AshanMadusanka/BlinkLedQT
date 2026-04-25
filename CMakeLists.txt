cmake_minimum_required(VERSION 3.14)

project(BlinkLed LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)

add_executable(BlinkLed
  main.cpp
)
target_link_libraries(BlinkLed Qt${QT_VERSION_MAJOR}::Core)

# set_target_properties(BlinkLed PROPERTIES
#     INSTALL_RPATH "/usr/local/qt6/lib"
#     BUILD_WITH_INSTALL_RPATH TRUE
# )

set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "/usr/local/qt6/lib"
    BUILD_RPATH "/usr/local/qt6/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

include(GNUInstallDirs)
install(TARGETS BlinkLed
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
