cmake_minimum_required(VERSION 3.10)


IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
  SET (ENV{ROBOCOMP} "/opt/robocomp/")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )
INCLUDE( $ENV{ROBOCOMP}/cmake/modules/qt.cmake )
INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  $ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  $ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  $ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui 3DCore 3DRender 3DExtras 3DInput 3DAnimation)
set(DOpen3D_ROOT "/home/juancarlos/sopen3d_install")
find_package(Open3D REQUIRED)
find_package(OpenMP)
        #ROBOCOMP
ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_IDSL_TO_ICE( CommonBehavior )
ROBOCOMP_ICE_TO_SRC( CommonBehavior )
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)

QT_WRAP_UI( UI_HEADERS mainUI.ui )

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-g  -fmax-errors=1 -fno-char8_t)


qt5_add_resources(GUI_RESOURCES  ../resources/res.qrc)

link_directories(${Open3D_LIBRARY_DIRS})
# Specify construction and link process
ADD_EXECUTABLE( qt3d_pruebas ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} ${GUI_RESOURCES})
target_include_directories(qt3d_pruebas PUBLIC ${Open3D_INCLUDE_DIRS})



TARGET_LINK_LIBRARIES( qt3d_pruebas ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${Ice_LIBRARIES}
        Qt5::3DCore
        Qt5::3DRender
        Qt5::3DAnimation
        Qt5::3DExtras
        Qt5::3DInput
        flann
        lz4
        )


SET_TARGET_PROPERTIES(qt3d_pruebas
        PROPERTIES
        CXX_STANDARD 20
        )


INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/qt3d_pruebas DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )
