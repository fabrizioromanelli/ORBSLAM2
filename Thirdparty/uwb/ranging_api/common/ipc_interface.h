#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>
#include <errno.h>
#include <semaphore.h>
#include <string.h>

#define CONST_MEM_SUFFIX "__const"
#define SHM_MEM_SUFFIX "__data"
#define SHM_SEM_SUFFIX "sem"


typedef struct IPC_Channel {
	char * IPC_mmap_ptr;
	char * IPC_const_ptr;
	sem_t * IPC_sem_ptr;
	int shm_size;
	int const_size;
} ipc_channel_t;

// WARNING: The difference between the init and the open is the exclusiveness of the
// shm object creation. the init must be called only by the controlling process, quile
// the open should be called by each process who wants to access the shared data.

ipc_channel_t * ipc_init(const char * channel_name, void * constant_mem, int const_size, int shm_size);
ipc_channel_t * ipc_open(const char * channel_name);
int ipc_close(ipc_channel_t * ipc, const char * channel_name);
int ipc_write(ipc_channel_t * channel, void * buffer, int size, int offset);
int ipc_read(ipc_channel_t * channel, void * buffer, int size, int offset);
int ipc_read_constant(ipc_channel_t * channel, void * buffer, int size, int offset);
