
CC=gcc
CCFLAGS=-I. -I /usr/include/python3.6 -O0 -ggdb -Wfatal-errors
LDFLAGS=-lpthread -lpython3.6m

all: test_main

.PHONY: clean

test_main : mempool.o rbtree.o priorityqueue.o astar_impl.o test_main.o
	$(CC) $^ $(LDFLAGS) -o $@ 

%.o: %.c
	$(CC) $< -c $(CCFLAGS)  -o $@

test: test_main
	@gdb -quiet -x gdbinit $<

clean:
	@rm -f *.o

