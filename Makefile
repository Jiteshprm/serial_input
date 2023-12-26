TARGET = srcd
CC ?= gcc
CFLAGS ?= -g -Wall
LDFLAGS ?= -Wall -lrt

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c) $(wildcard inih/*.c))
HEADERS = $(wildcard *.h) $(wildcard inih/*.h)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@ -lrt

clean:
	-rm -f *.o inih/*.o
	-rm -f $(TARGET)
