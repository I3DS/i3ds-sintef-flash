find_package (Boost COMPONENTS program_options log REQUIRED)

set (SRCS
  sintef_flash.cpp
  )

set (LIBS
  i3ds
  zmq
  pthread
  ${Boost_LIBRARIES}
  )

include_directories ("../include/")

add_executable (i3ds_sintef_flash i3ds_sintef_flash.cpp ${SRCS})
target_link_libraries (i3ds_sintef_flash ${LIBS})
install(TARGETS i3ds_sintef_flash DESTINATION bin)
