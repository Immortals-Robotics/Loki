project(${CMAKE_PROJECT_NAME}-vision)
find_package(Eigen3 CONFIG REQUIRED)

find_package(immortals-common CONFIG REQUIRED)

set(HEADER_FILES
        pch.h
        filtered/filtered.h
        kalman/filtered_object.h
        raw/raw.h)

set(SOURCE_FILES
        filtered/ball.cpp
        filtered/filtered.cpp
        filtered/robot.cpp
        kalman/filtered_object.cpp
        raw/ball.cpp
        raw/communication.cpp
        raw/robot.cpp
        raw/raw.cpp)

add_library(${PROJECT_NAME}
        ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES} ${HEADER_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
        immortals::common)
	Eigen3::Eigen)

set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ${USE_UNITY_BUILDS})

target_precompile_headers(${PROJECT_NAME}
        PUBLIC pch.h
        INTERFACE ${HEADER_FILES})
