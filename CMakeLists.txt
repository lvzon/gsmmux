cmake_minimum_required(VERSION 3.8)
project(gsmmux)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        src/buffer.c
        src/buffer.h
        src/gsm0710.c
        src/gsm0710.h
        Makefile)

add_executable(gsmmux ${SOURCE_FILES})