CC = gcc -Wall -Wextra

SOURCEDIRS = . ../common ./wrapper ./adjm_proto
vpath %.c $(SOURCEDIRS)
CFLAGS += ${addprefix -I,$(SOURCEDIRS)}

srcs = $(wildcard *.c)
objs = $(srcs:.c=.o)
deps = $(srcs:.c=.d)
exec = test
# "all" target to compile everything with a single command ("make all")
# you should list the compilation targets here
ranging_service: cubes_wrapper.o cubes_common.o serial.o network_handler.o ipc_interface.o ipc_proto.o range_scheduler.o ranging_service.o
	$(CC) -o $@ $^  -lpthread -lrt
	$(info modify the file ranging_config.h to change loop duration, node order and timing)

# a generic method for compiling all source .c files into
# object .o files with automatic creation of file dependencies
%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@ -lrt

-include $(deps)

# "clean" target, to remove all binaries ("make clean")
.PHONY: clean
clean:
	rm -f *.o *.d $(ALL_TARGETS)
