#!/usr/bin/env python3

import random
import socket
import struct

from TelemetryDrawer import TelemetryPrinter
from TelemetryServer import TelemetryServer
from time import sleep
from datetime import datetime



if __name__ == '__main__':

	tp = TelemetryPrinter()
	
	host = ''
	port = 22111
	data_size = 8 # size of double
	data_elements = 8;
	
	
	tserver = TelemetryServer(host, port, data_size * data_elements)
	tserver.start()
	
	r1_c = [100, 100]
	r2_c = [100, 100]
	r3_c = [100, 100]
	d0_c = [100, 100]
	
	print("\n\n\n\n\n\n")
	start_time = datetime.now()
	
	continue_loop = True
	while continue_loop:
		# get coordinates
		#print("Waiting for data")
		
		data = tserver.getTelemetry()

		double_sequence = []
		#if(data == None or len(data) < data_size * data_elements):
			#print("Invalid data received. Size of the data is less than the required amount")
		#else:
		if(data != None and len(data) >= data_size * data_elements):
			byte_list = list(data)
			for i in range(data_elements):
				#double_bytes = byte_list[i * data_size: (i + 1) * data_size]
				b = struct.pack('8B', *byte_list[i * data_size: (i + 1) * data_size])
				double_sequence.append(struct.unpack('d', b)[0])
			
			print("\033[6ANew data received. Data vector:\nRover 1 : \t%.2f \t%.2f \t(Marco - BLUE)\nRover 2 : \t%.2f \t%.2f\t(Gialla - GREEN) \nRover 3 : \t%.2f \t%.2f\t(Giova - RED) \nDrone   : \t%.2f \t%.2f\t(Luca - BLACK) \nLast message received %.2f seconds ago" % (double_sequence[0], double_sequence[1], double_sequence[2], double_sequence[3], double_sequence[4], double_sequence[5], double_sequence[6], double_sequence[7], (datetime.now() - start_time).total_seconds()))
		
			# update gfx
			tp.updateGfx(double_sequence[0:2], double_sequence[2:4], double_sequence[4:6], double_sequence[6:8])
			
		continue_loop = not tp.closeWindowRequired();
		sleep(0.032) #max achievable framerate is 30 FPS
	
	tserver.stopServer();
	
