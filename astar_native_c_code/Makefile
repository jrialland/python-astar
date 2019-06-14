
CC=gcc
CCFLAGS=-I. -I /usr/include/python3.6 -O0 -ggdb -Wfatal-errors
LDFLAGS=-lpthread -lpython3.6m

all: test_main

.PHONY: clean

map_test : mempool.o utils.o rbtree.o astar_impl.o main.o
	$(CC) $^ $(LDFLAGS) -o $@ 

%.o: %.c
	$(CC) -c $(CCFLAGS) $< -o $@

test: map_test
	@gdb -quiet -x gdbinit $<

clean:
	@rm -f *.o

