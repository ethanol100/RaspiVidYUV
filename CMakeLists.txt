cmake_minimum_required(VERSION 2.8)
# raspividyuv
set(VCOS_PTHREADS_BUILD_SHARED TRUE)
set(CMAKE_BUILD_TYPE Release)

SET(COMPILE_DEFINITIONS -Werror)

include_directories(/opt/vc/include/interface/vcos/pthreds)
include_directories(/opt/vc/include/interface/vcos/pthreads)
include_directories(/opt/vc/include/interface/vmcs_host/linux)
include_directories(/opt/vc/include/interface/vmcs_host)
include_directories(/opt/vc/include)

find_package (Threads)

FIND_LIBRARY(MMAL_CORE_LIBRARY mmal_core /opt/vc/lib)
FIND_LIBRARY(MMAL_UTIL_LIBRARY mmal_util /opt/vc/lib)
FIND_LIBRARY(MMAL_VC_CLIENT_LIBRARY mmal_vc_client /opt/vc/lib)
FIND_LIBRARY(VCOS_LIBRARY vcos /opt/vc/lib)
FIND_LIBRARY(BCM_HOST_LIBRARY bcm_host /opt/vc/lib)

set (COMMON_SOURCES
   RaspiCamControl.c
   RaspiCLI.c
   RaspiPreview.c)

add_executable(raspividyuv   ${COMMON_SOURCES} RaspiVidYUV.c)



target_link_libraries(raspividyuv 
     ${MMAL_CORE} 
     ${MMAL_CORE_LIBRARY} 
     ${MMAL_UTIL_LIBRARY} 
     ${MMAL_VC_CLIENT_LIBRARY} 
     ${VCOS_LIBRARY} 
     ${BCM_HOST_LIBRARY}
     ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS raspividyuv DESTINATION /usr/local/bin)
