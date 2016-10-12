CC = arm-linux-gnueabihf-gcc
LIB = lasound
INC = /usr/include/
TARGET = audio
SRC =  $(TARGET).c
OBJS = $(patsubst %.c ,%.o ,$(SRC))
.PHONY: all
all: $(TARGET)
$(TARGET) : $(OBJS)
	$(CC) -o $@ $^ -$(LIB) -I$(INC)
%.o : %.c
	$(CC) -c $< -o $@ 
clean:
	$(RM) *.o $(TARGET)
