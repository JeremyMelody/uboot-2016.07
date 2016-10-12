CC = $(CROSS_COMPILE)gcc
all: can_send can_receive
.PHONY: all
can_send 	: can_send.c
	$(CC) -o $@ $^
can_receive : can_receive.c
	$(CC) -o $@ $^
clean:
	$(RM) *.o can_send can_receive
