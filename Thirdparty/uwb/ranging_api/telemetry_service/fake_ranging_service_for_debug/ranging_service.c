#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include "adjm_proto/range_scheduler.h"

void termination_handler(int signum) {
	(void) signum;
	fprintf(stderr, "Terminating protocol...\n");
	stop_handler();
	
	exit(EXIT_SUCCESS);
}


int main(int argc, char ** argv)
{
	if(argc < 2) {
		fprintf(stderr, "Error: serial device path required\nusage: %s <serial_device_file>\n", argv[0]);
		exit(1);
	}
	
	if(init_ranging_scheduler(argv[1]) < 0) {
		fprintf(stderr, "Initialization error\n");
		exit(1);
	}
	
	/* Adding signal handler for a clean termination */
	
	struct sigaction signal_handler;
	
	memset(&signal_handler, 0, sizeof(struct sigaction));
	
	signal_handler.sa_handler = termination_handler;

	signal_handler.sa_flags = 0;

	sigaction (SIGINT,  &signal_handler, NULL);
	sigaction (SIGHUP,  &signal_handler, NULL);
	sigaction (SIGHUP,  &signal_handler, NULL);
	sigaction (SIGTERM, &signal_handler, NULL);
	
	start_ranging_scheduler();
}

