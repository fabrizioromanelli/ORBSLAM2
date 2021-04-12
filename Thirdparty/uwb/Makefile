MAIN = main

BIN_DIR = bin
SRC_DIR = source
INC_DIR = include
OBJ_DIR = object

SRC = $(wildcard $(SRC_DIR)/*.c)
OBJ = $(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

CC = gcc
CFLAGS += -Wall -Wextra  -O2 -I$(INC_DIR)
CFLAGS += -Werror=implicit-function-declaration -IlibUWBranging.so
#CFLAGS += -Werror
LDFLAGS += -L. -Llib
LDLIBS += -lm -lpthread -lrt -lUWBranging #-lpigpio

# verbose control
# DEFINES += -DMDEBUG
#keyboard control
DEFINES += -DNCURSES

.PHONY: clean

$(MAIN): main.o $(OBJ)
	$(CC) -DMAIN $(LDFLAGS) $^ $(LDLIBS) -o $(BIN_DIR)/$@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(DEFINES) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJ) rover.o drone.o main.o
