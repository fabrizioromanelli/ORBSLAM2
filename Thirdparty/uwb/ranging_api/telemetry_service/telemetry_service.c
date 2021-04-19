/****************************************************************************************
 *	Telemetry service:								*
 *	this software is a relay service that forwards ipc data to an UDP socket.	*
 * 	on the other side of the socket the telemetry printer should show the		*
 * 	relative positions of each entity in the system.				*
 ****************************************************************************************/

#include "../libUWBranging/ranging_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h> 
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <unistd.h>


#define TELEMETRY_PRINTER_PORT 22111
#define SLEEP_TIME 500000

int main(int argc, char *argv[])
{
	/* IPC initialization */
	int ipc_test_counter = 0;
	ipc_channel_t * channel = NULL;
	do {
		channel = start_ipc();
		if(channel == NULL) {
			fprintf(stderr, "IPC not ready.\n");
			if(ipc_test_counter < 5){
				fprintf(stderr, "retry\n");
			}
			else {
				fprintf(stderr, "maximum attempt reached\n");
				exit(1);
			}
			ipc_test_counter++;
			usleep(500000);
		}
	} while(channel == NULL);
	
	
	// number of nodes
	int number_of_nodes = get_number_of_nodes(channel);
	if(number_of_nodes < 0) {
		fprintf(stderr, "Error, invalid number of nodes\n");
		exit(1);
	}
	uint16_t * adjacency_vector = malloc(number_of_nodes * (number_of_nodes - 1) /  2 * sizeof(uint16_t));
	if(adjacency_vector == NULL) {
		fprintf(stdout, "OutOfMemoryError: cannot allocate adjacency vector\n");
		exit(1);
	}
	coordinates_t * coords_vector = malloc(number_of_nodes * sizeof(coordinates_t));
	if(adjacency_vector == NULL) {
		fprintf(stdout, "OutOfMemoryError: cannot allocate adjacency vector\n");
		exit(1);
	}
	uint64_t timestamp;

	printf("Node:\t%X\t%X\t%X\t%x      \n" \
                        "Lat:\t%.2f\t%.2f\t%.2f\t%.2f      \n"\
                        "Lon:\t%.2f\t%.2f\t%.2f\t%.2f      \n", 
        		coords_vector[0].c_lat, coords_vector[0].c_lon, coords_vector[1].c_lat, coords_vector[1].c_lon,
        		coords_vector[2].c_lat, coords_vector[2].c_lon, coords_vector[3].c_lat, coords_vector[3].c_lon);

	/* socket initialization*/
	struct sockaddr_in server_addr; /* connector's address information */
	struct hostent *server_address;
	int numbytes;

	if(argc < 2) {
		fprintf(stderr, "Usage: %s <address>\n", argv[0]);
		exit(1);
	}

	if ((server_address = gethostbyname(argv[1])) == NULL) { 
		herror("gethostbyname");
		exit(1);
	}
	
	/* server info */
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(TELEMETRY_PRINTER_PORT);
	server_addr.sin_addr = *((struct in_addr *)server_address->h_addr);
	bzero(&(server_addr.sin_zero), 8);     /* zero the rest of the struct */

	int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd == -1) {
		perror("socket");
		exit(1);
	}

	clock_t t; 
	
	/* END OF THE INITIALIZATION */

	while(1) {
		
		t = clock();   		
		if(get_data_vectors(channel, adjacency_vector, coords_vector, &timestamp) < 0) {
			fprintf(stderr, "ERROR! cannot read shared memory\n");
			break;
		}
		else {
			
			numbytes = sendto(sockfd, (char *) coords_vector, number_of_nodes * sizeof(coordinates_t), 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
			if (numbytes == -1) {
				perror("sendto");
				exit(1);
			}
			
			printf("sent %d bytes to %s\n", numbytes, inet_ntoa(server_addr.sin_addr));
		}
		
		t = clock() - t; 
		double time_taken = ((double)t)/CLOCKS_PER_SEC;
		printf("Elapsed: %e seconds\n", time_taken);
		
		usleep(SLEEP_TIME);
	}
	
	close(sockfd);
	

	return 0;
}
