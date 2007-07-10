# Comment/uncomment the following line to disable/enable debugging
DEBUG = y

TARGET = gsmMuxd
INSTALLPATH = /usr/local/bin
SRC = gsm0710.c buffer.c
OBJS = gsm0710.o buffer.o

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

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(LD) $(LDLIBS) -o $@ $(OBJS)

install: $(TARGET)
	install -D -o root -g root -s $(TARGET) $(INSTALLPATH)/$(TARGET)
	@echo "NOTE: if you want to use the automatic startup script mux.d, you need to"
	@echo "install it manually and make sure it is called at startup (e.g. through SYS-V init)"

.PHONY: all clean
