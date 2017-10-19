#!/bin/sh
/usr/local/DigiEL-5.7/x-tools/arm-unknown-linux-uclibcgnueabi/bin/arm-unknown-linux-uclibcgnueabi-gcc -Os -march=armv5te -mtune=arm926ej-s -o build/gsmmuxd-herman -Wall src/gsm0710.c src/buffer.c
