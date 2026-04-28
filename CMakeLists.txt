cmake_minimum_required(VERSION 3.14)

project(BlinkLed LANGUAGES C CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(Threads REQUIRED)
find_library(GPIOD_LIBRARY NAMES gpiod REQUIRED)

add_executable(BlinkLed
  main.cpp
  mcp23017_rpi/mcp23017_rpi.c
  mcp23017_rpi/mcp23017_rpi.h
  modbus_rs485_rpi/modbus_rs485_rpi.c
  modbus_rs485_rpi/modbus_rs485_rpi.h
  cwt_sl_lth_rpi/cwt_sl_lth_rpi.c
  cwt_sl_lth_rpi/cwt_sl_lth_rpi.h
  sn3002_co2_rpi/sn3002_co2_rpi.c
  sn3002_co2_rpi/sn3002_co2_rpi.h
  xl9535_rpi/xl9535_rpi.c
  xl9535_rpi/xl9535_rpi.h
  flow_sensor_rpi/flow_sensor_rpi.c
  flow_sensor_rpi/flow_sensor_rpi.h
)
target_include_directories(BlinkLed PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/mcp23017_rpi
)
target_link_libraries(BlinkLed Qt${QT_VERSION_MAJOR}::Core Threads::Threads ${GPIOD_LIBRARY})

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
