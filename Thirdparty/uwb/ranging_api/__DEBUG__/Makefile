CC = gcc -Wall -Wextra

SOURCEDIRS = . ../common
vpath %.c $(SOURCEDIRS)
CFLAGS += ${addprefix -I,$(SOURCEDIRS)}


# "all" target to compile everything with a single command ("make all")
# you should list the compilation targets here
ranging_benchmark: main_test.c
	$(CC) -o $@ $^  -L. -lpthread -lrt -lUWBranging

# a generic method for compiling all source .c files into
# object .o files with automatic creation of file dependencies
%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@ -L. -lrt -lUWBranging

-include $(deps)

# "clean" target, to remove all binaries ("make clean")
.PHONY: clean
clean:
	rm -f *.o *.d $(ALL_TARGETS)
