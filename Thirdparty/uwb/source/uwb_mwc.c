#include "uwb.h"
#include "common.h"
#include "../ranging_api/common/pretty_printer.h"
#include "../ranging_api/libUWBranging/ranging_api.h"

#include <time.h>
#include <math.h>

bool uwb_print = TRUE;
int ipc_counter = 0;
ipc_channel_t *channel = NULL;
uint16_t *nodes_list = NULL;
uint16_t *adjacency_vector = NULL;
coordinates_t *coords_vector = NULL;
uint64_t timestamp = 0;

int number_of_nodes = 0;
uint8_t node_index = 0;
uint16_t my_id = 0;
uint16_t distances[6] = {0};
double data_out[8];

int check_delta;

int C1[] = {0,0,0};
int C2[] = {0,200,0};
int C3[] = {200,0,0};

double coord[3];

FILE *f;

void sig_handler(void)
	{	
 
	fclose(f);
	printf("\n##########\nCLOSING FILE\n##########\n");
	exit(0);
	
	}

void error_handler(const char *msg)
{
    fprintf(stderr, "==> ERROR\n");
    perror(msg);
    exit(EXIT_FAILURE);
}

void initialize_UWB(void)
{
	fprintf(stdout, "Initializing UWB...\n");
	/* 
	 * If the start_ipc() fails, probably the ranging process
	 * is not in execution. Wait for it for a bit and then fail
	 */

	time_t ltime; /* calendar time */
    	ltime=time(NULL); /* get current cal time */
    	char *fname = asctime( localtime(&ltime));
	
	char fileName[64];
	sprintf(fileName, "test/%s.txt", fname);

	if ((f=fopen(fileName, "w"))==NULL)  //in test??
		printf("ERROR: opening file\n");
	else
		printf("FILE CREATED\n");
	do
    {
		channel = start_ipc();

		if (channel == NULL)
        {
			fprintf(stderr, "IPC not ready.\n");

			if (ipc_counter < 5)
				fprintf(stderr, "retry\n");
			else
                error_handler("initialize_UWB | maximum attempts reached");

			ipc_counter++;
			usleep(500000);
		}
	} while(channel == NULL);

	// number of nodes
	number_of_nodes = get_number_of_nodes(channel);

	if (number_of_nodes < 0)
        error_handler("initialize_UWB | invalid number of nodes");
	
	// nodes list
	nodes_list = malloc(number_of_nodes * sizeof(uint16_t));
    if (nodes_list == NULL)
    {
        error_handler("initialize_UWB | malloc [nodes_list]");
    }
	if (get_node_list(channel, nodes_list) < 0)
    {
        error_handler("initialize_UWB | get_node_list");
    }
	
	// my id
	my_id = get_my_nodeid(channel);
	if (my_id == 0)
        error_handler("initialize_UWB | get_my_nodeid");
	
	/* We have read all the constant information we need.
	 * here we can start our protocol
	 */
    fprintf(stdout, "\e[1;33m");
    printintro(my_id, "__SHM_FILE__ /telemetry");
    fprintf(stdout, "\e[21;0m");
	fflush(stdout);

	/* vector allocation: here adjacency vector and coordinates
	 * vector are preallocated to be filled in the main loop
	 */
	adjacency_vector = malloc(number_of_nodes * (number_of_nodes - 1) /  2 * sizeof(uint16_t));
    if (adjacency_vector == NULL)
    {
        error_handler("initialize_UWB | malloc [adjacency_vector]");
    }

	coords_vector = malloc(number_of_nodes * sizeof(coordinates_t));
	if (coords_vector == NULL)
    {
        error_handler("initialize_UWB | malloc [coords_vector]");
    }

    node_index_from_id(get_my_nodeid(channel));
    fprintf(stdout, "I am node number %d [get_my_nodeid: 0x%2X]\n", node_index, get_my_nodeid(channel));
}

void compute_coordinates(void)
{
	
	// raggi in cm
	double R1 = adjacency_vector[2]; 
	double R2 = adjacency_vector[4];
	double R3 = adjacency_vector[5];

	double a0 = -2*C1[0];
	double b0 = -2*C1[1];
	double c0 = -2*C1[2];
	double d0 = a0*a0/4 + b0*b0/4 + c0*c0/4 - R1*R1;

	double e0 = -2*C2[0];
	double f0 = -2*C2[1];
	double g0 = -2*C2[2];
	double h0 = e0*e0/4 + f0*f0/4 + g0*g0/4 - R2*R2;

	double i0 = -2*C3[0];
	double l0 = -2*C3[1];
	double m0 = -2*C3[2];
	double n0 = i0*i0/4 + l0*l0/4 + m0*m0/4 - R3*R3;

	double a = a0-e0;
	double b = b0-f0;
	double c = c0-g0;
	double d = d0-h0;

	double e = a0-i0;
	double f = b0-l0;
	double g = c0-m0;
	double h = d0-n0;

	double p1 = (b*h-f*d)/(a*f-e*b);
	double p2 = (b*g-f*c)/(a*f-e*b);
	double p3 = (e*d-a*h)/(a*f-e*b);
	double p4 = (e*c-a*g)/(a*f-e*b);

	double A = p2*p2 + p4*p4 +1;
	double B = 2*p1*p2 + 2*p3*p4 + a0*p2 + b0*p4 + c0;
	double C = p1*p1 + p3*p3 + a0*p1 + b0*p3 + d0;

	double delta = B*B-4*A*C;
	
	if (delta < 0)
	{
/*
		check_delta = 1;
		delta = abs(delta);
*/				
		check_delta = 0;
		fprintf(stdout, "\n#########\nNO REAL SOLUTION FOUND\n#########\n");
		//fprintf(f, "\n#########\nNO REAL SOLUTION FOUND\n#########\n");
		fprintf(stdout, "DELTA: %f\n", delta);
		fprintf(stdout, "R1: %d\n", adjacency_vector[2]);

	}
	else 
	{
		check_delta = 1;
		double z = /*abs*/(-B+sqrt(delta));
		double x = p1 + p2*z;
		double y = p3 + p4*z;

		coord[0] = x;
		coord[1] = y;
		coord[2] = z;
	}
}

bool read_UWB(void)
{
    // adjacency_vector = distances
    // coords_vector = coordinates

    if (get_data_vectors(channel, adjacency_vector, coords_vector, &timestamp) < 0)
        error_handler("read_UWB | get_data_vectors");
    else
    {
        if (uwb_print)
        {
/*
            fprintf(stdout, "#########\n");
            fprintf(stdout, "\tD12: %d\n", adjacency_vector[0]);
            fprintf(stdout, "\tD13: %d\n", adjacency_vector[1]);
            fprintf(stdout, "\tD23: %d\n", adjacency_vector[3]);
            fprintf(stdout, "\tD14: %d\n", adjacency_vector[2]);
            fprintf(stdout, "\tD24: %d\n", adjacency_vector[4]);
            fprintf(stdout, "\tD34: %d\n", adjacency_vector[5]);
            fprintf(stdout, "\t---------\n");
            fprintf(stdout, "\tdata_1_1: %.4f | data_2_1: %.4f\n", coords_vector[0].c_lon, coords_vector[0].c_lat);
            fprintf(stdout, "\tdata_1_2: %.4f | data_2_2: %.4f\n", coords_vector[1].c_lon, coords_vector[1].c_lat);
            fprintf(stdout, "\tdata_1_3: %.4f | data_2_3: %.4f\n", coords_vector[2].c_lon, coords_vector[2].c_lat);
            fprintf(stdout, "\tdata_1_4: %.4f | data_2_4: %.4f\n", coords_vector[3].c_lon, coords_vector[3].c_lat);
            fprintf(stdout, "#########\n\n");
*/

	    if (adjacency_vector[2] == 65534 || adjacency_vector[4] == 65534 || adjacency_vector[5] == 65534)
	 	{
			fprintf(stdout, "One of UWB is not connected!\nCheck for 65534 value\n");
			fprintf(stdout, "\tD14: %d\n", adjacency_vector[2]);
			fprintf(stdout, "\tD24: %d\n", adjacency_vector[4]);
		        fprintf(stdout, "\tD34: %d\n", adjacency_vector[5]);
		}
	    else
		{
		    
		if (check_delta == 1)
			{
			fprintf(stdout, "\t%f", coord[0]);
	                fprintf(stdout, "\t%f", coord[1]);
		        fprintf(stdout, "\t%f\n", coord[2]);
			fprintf(stdout, "R1: %d\n", adjacency_vector[2]);

	                fprintf(f, "%f,", coord[0]);
		        fprintf(f, "%f,", coord[1]);
		        fprintf(f, "%f\n", coord[2]);

			}		
		}
      	}

        for (int a = 0; a < 6; a++)
        {
            if (adjacency_vector[a] == MAX_uint16_T)
            {
                if (uwb_print)
                    fprintf(stderr, "read_UWB | someone has %d value!\n", MAX_uint16_T);
                return FALSE;
            }
        }

        distances[0] = adjacency_vector[0]; // Distance between agent 1 and agent 2 
        distances[1] = adjacency_vector[1]; // Distance between agent 1 and agent 3 
        distances[2] = adjacency_vector[3]; // Distance between agent 2 and agent 3
        distances[3] = adjacency_vector[2]; // Distance between agent 1 and agent 4
        distances[4] = adjacency_vector[4]; // Distance between agent 2 and agent 4
        distances[5] = adjacency_vector[5]; // Distance between agent 3 and agent 4
    }

    data_out[0] = coords_vector[0].c_lon;
    data_out[1] = coords_vector[0].c_lat;
    data_out[2] = coords_vector[1].c_lon;
    data_out[3] = coords_vector[1].c_lat;
    data_out[4] = coords_vector[2].c_lon;
    data_out[5] = coords_vector[2].c_lat;
    data_out[6] = coords_vector[3].c_lon;
    data_out[7] = coords_vector[3].c_lat;

    return TRUE;
}

void write_UWB(double data_1, double data_2)
{
    if (uwb_print)
        fprintf(stdout, "write_UWB | %.3f | %.3f\n", data_1, data_2);
    write_coordinates(channel, data_1, data_2);
}

void node_index_from_id(unsigned int node_id)
{
    switch (node_id)
    {
        case NODE_1:
            node_index = 0;
            break;
        case NODE_2:
            node_index = 1;
            break;
        case NODE_3:
            node_index = 2;
            break;
        case NODE_4:
            node_index = 3;
            break;
    }
}

double *get_UWB_data(void)
{
    return data_out;
}

uint16_t *get_UWB_dist(void)
{
    return distances;
}
