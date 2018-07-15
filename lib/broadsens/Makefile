###########################################  
#Makefile for simple programs  
###########################################

PWD := $(shell pwd)
LIB_DIR = $(shell dirname $(PWD))
LIB_BLD_DIR = $(LIB_DIR)/output

LIB_SRCS = $(wildcard *.c i2c/*.c bno055/*.c ads122/*.c drv_api/*.c e2prom/*.c common/*.c)
LIB_OBJS = $(LIB_SRCS:.c = .o)

SRCS = $(wildcard *.c sensor/*.c )
OBJS = $(SRCS:.c = .o)
CC = ../../tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++
INCLUDES = -I $(LIB_BLD_DIR)/libiio/include

CCFLAGS = -g -Wall -O0
OUTPUT = test.out
LIB_TARGET = libdriver.so
LIBS = -liio -lxml2
LDFLAGS = -L$(LIB_BLD_DIR)/libiio/lib \
          -L$(LIB_BLD_DIR)/libxml2/lib
LIB_INIPARSER = ../lib/iniparser

drv : $(LIB_TARGET)
$(LIB_TARGET) : $(LIB_OBJS)
	$(CC) $^ -fPIC -shared -o $@ $(INCLUDES) -liniparser -L $(LIB_INIPARSER)

test : $(OUTPUT)

$(OUTPUT) : $(OBJS)
	$(CC) $^ -o $@ $(INCLUDES) $(LIBS) $(LDFLAGS)

%.o : %.c
	$(CC) -c $< $(CCFLAGS)

clean:
	rm -rf *.out *.so

.PHONY:clean