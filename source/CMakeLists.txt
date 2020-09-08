cmake_minimum_required(VERSION 2.8)
project(simpleagent_template)

#aux_source_directory(. SRC_LIST)
#add_executable(${PROJECT_NAME} ${SRC_LIST})

# point to cosmos/source folder
set(COSMOS_SOURCE   "../../../")
set(USE_COSMOS_FROM "SOURCE")



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -std=c++11 -Wno-invalid-offsetof")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# -----------------------------------------------
# To use COSMOS from the source folder
#include(${CMAKE_SOURCE_DIR}/use_cosmos_from_source.cmake)
include(${COSMOS_SOURCE}/core/cmake/use_cosmos_from_source.cmake)

#add_executable(agent_cpu "${COSMOS_SOURCE}/core/programs/agents/agent_cpu.cpp")
#target_link_libraries(agent_cpu
#    CosmosSupport
#    CosmosAgent
#    CosmosDeviceGeneral
#    CosmosDeviceCpu
#    CosmosDeviceDisk
#    CosmosMath
#    )

set(COSMOS_LIBS
    CosmosSupport
    CosmosAgent
    CosmosDeviceGeneral
    CosmosDeviceI2C
    CosmosDeviceSerial
    CosmosDeviceCpu
    CosmosDeviceDisk
    CosmosMath
    )




# -----------------------------------------------

include_directories("../include")


set(SIMPLEAGENT_SRC utility/SimpleAgent.cpp
						utility/Device.cpp)
add_library(SimpleAgent STATIC ${SIMPLEAGENT_SRC})
target_link_libraries(SimpleAgent ${COSMOS_LIBS})

set(CUBESAT_LIBS SimpleAgent)



# ============================== Agents ==============================

add_executable(my_agent my_agent.cpp)
target_link_libraries(my_agent ${COSMOS_LIBS} ${CUBESAT_LIBS})
