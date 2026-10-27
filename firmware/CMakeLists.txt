# Top-level ESP-IDF project file for the gripper_plus firmware.
#
# Build with:
#   idf.py set-target esp32c6
#   idf.py build
#
# All firmware modules live under components/. Only the entry point
# (app_main) lives under main/.

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(gripper_plus)
