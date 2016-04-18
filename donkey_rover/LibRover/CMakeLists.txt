cmake_minimum_required(VERSION 3.0.2)

project(Rover)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

add_library(Rover STATIC BogieScanner.cc CANopenHelper.cc CANopenMaster.cc Drive.cc Track.cc libRover.cc)
