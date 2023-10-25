project(dummy NONE)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    set(CMAKE_CXX_COMPILER g++-13)
endif()
cmake_minimum_required(VERSION 3.14)
project(periodic CXX)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include(FetchContent)

# Optional: set this to ON if your target publicly links to nlohmann_json and needs to install()
# set(JSON_Install ON)

FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
    GIT_TAG v3.11.2
)

FetchContent_MakeAvailable(json)

message(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
cmake_path(GET PROJECT_SOURCE_DIR PARENT_PATH PARENT_DIR)
message(PARENT_DIRECTORY="${PARENT_DIR}")
cmake_path(GET PARENT_DIR PARENT_PATH ROOT_DIR)
message(ROOT_DIRECTORY="${ROOT_DIR}")
cmake_path(APPEND INCLUDE_DIR ${ROOT_DIR} include)
message(INCLUDE_DIR="${INCLUDE_DIR}")
cmake_path(APPEND SOURCE_DIR ${ROOT_DIR} src)
message(SOURCE_DIR="${SOURCE_DIR}")

add_executable(periodic main.cpp)
target_include_directories(periodic PRIVATE
    ${INCLUDE_DIR}
)
target_sources(periodic PRIVATE
    ${SOURCE_DIR}/Periodic.cpp
    ${SOURCE_DIR}/Rods.cpp
    ${SOURCE_DIR}/ParametersForRods.cpp
    ${SOURCE_DIR}/ForcesOnSegments2d.cpp
    ${SOURCE_DIR}/util.cpp
)

target_compile_options(periodic PRIVATE -O3 -Wall -Wextra -Wno-psabi)
target_link_libraries(periodic PRIVATE nlohmann_json::nlohmann_json)
target_compile_features(periodic PRIVATE cxx_std_20)
