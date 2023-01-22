cmake_minimum_required(VERSION 3.15)
cmake_policy(SET CMP0048 NEW)

project(curveAttract VERSION 1.0 DESCRIPTION "Curve attract deformer for Maya" LANGUAGES CXX)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

add_subdirectory(src)