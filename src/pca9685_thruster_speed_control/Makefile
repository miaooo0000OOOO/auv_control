CC = gcc
CFLAGS_DEBUG = -g -Wall
CFLAGS_RELEASE = -O2 -Wall
SRC = src/pca9685_thruster_speed_control.c
OBJ = $(SRC:.c=.o)
TARGET_DEBUG = pca9685_thruster_speed_control_debug
TARGET_RELEASE = pca9685_thruster_speed_control_release
LIB_TARGET = libpca9685.so

.PHONY: all clean debug release shared

all: debug

debug: CFLAGS = $(CFLAGS_DEBUG)
debug: $(TARGET_DEBUG)

release: CFLAGS = $(CFLAGS_RELEASE)
release: $(TARGET_RELEASE)

shared: CFLAGS = -fPIC -shared
shared: $(SRC)
	$(CC) $(CFLAGS) -o $(LIB_TARGET) $^ -lm

$(TARGET_DEBUG): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ -lm

$(TARGET_RELEASE): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ -lm

clean:
	rm -f $(OBJ) $(TARGET_DEBUG) $(TARGET_RELEASE) $(LIB_TARGET)