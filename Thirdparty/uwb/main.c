#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include "uwb.h"

int main(int argc, char *argv[])
{
    argc = argc;
    argv = argv; 

    initialize_UWB();

    signal(SIGINT, sig_handler);
    signal(SIGQUIT, sig_handler);

    for (;;)
    {
        usleep(500000);
        printf("Reading UWB...\n");
	compute_coordinates();
        read_UWB();
    }
    return EXIT_SUCCESS;
}
