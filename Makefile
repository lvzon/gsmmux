# Comment/uncomment the following line to disable/enable debugging
DEBUG = y

TARGET = build/gsmmuxd
INSTALLPATH = /usr/local/bin
SRC = src/gsm0710.c src/buffer.c
OBJS = build/gsm0710.o build/buffer.o

CC = gcc
LD = gcc
CFLAGS = -Wall
LDLIBS = -lm

ifeq ($(DEBUG),y)
  CFLAGS += -DDEBUG
endif


all: $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
	rm -rf build
	mkdir build

build/buffer.o:
	$(CC) $(CFLAGS) -I./src -c -o build/buffer.o src/buffer.c
	 
build/gsm0710.o: build/buffer.o
	$(CC) $(CFLAGS) -I./src -c -o build/gsm0710.o src/gsm0710.c build/buffer.o

%.o: %.c
	mkdir build
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(LD) $(LDLIBS) -o $@ $(OBJS)

install: $(TARGET)
	install -D -o root -g root -s $(TARGET) $(INSTALLPATH)/$(TARGET)
	@echo "NOTE: if you want to use the automatic startup script mux.d, you need to"
	@echo "install it manually and make sure it is called at startup (e.g. through SYS-V init)"

.PHONY: all clean
