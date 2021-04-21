GCC=g++
CFLAGS=-O3 -Wall -lm

EXE=broadcast

SRCS=$(wildcard *.cpp)

OBJS=$(SRCS:.cpp=.o)

all: $(OBJS)
	$(GCC) -o $(EXE) $(OBJS) $(CFLAGS) 

$(OBJS):%.o:%.cpp
	$(GCC) -c -o $@ $< $(CFLAGS)

clean:
	rm -rf $(OBJS) $(EXE)

