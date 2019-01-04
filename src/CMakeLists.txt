cmake_minimum_required(VERSION 2.8.3)


################module:#######################
#input:
  #config:
    set(MODULE_NAME ZLTT_LIB)
    
  #set env:
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -lpthread -std=c++0x -Wfatal-errors")
    SET(CMAKE_CXX_COMPILER "arm-none-linux-gnueabi-g++")


  #set include_dir:
    include_directories(
      include
      ${Boost_INCLUDE_DIRS}
    )

  #set src_set:
    AUX_SOURCE_DIRECTORY(. SRC_SET_${MODULE_NAME})


#output:
  add_library(${MODULE_NAME} STATIC ${SRC_SET_${MODULE_NAME}})
  
  target_link_libraries(${MODULE_NAME}
    ${Boost_LIBRARIES}
  )
############################################