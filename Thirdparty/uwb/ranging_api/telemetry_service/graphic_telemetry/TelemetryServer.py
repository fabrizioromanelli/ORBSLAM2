#!/usr/bin/env python3

import socket
import multiprocessing as mp
import os
import signal
import sys

class TelemetryServer():
	def __init__(self, host, port, datasize = 1024):
		self.host = host
		self.port = port
		self.datasize = datasize

	def __p_signal(self):
		print("SIGNAL RECEIVED")
		self.sock.close()
		sys.exit(0)
		

	def __start(self, outqhandle, host, port, datasize):
		signal.signal(signal.SIGINT, self.__p_signal)
		
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.sock.bind((host, port))
		
		while True:
			data = self.sock.recv(datasize);
			outqhandle.put(data)

	def start(self):
		self.q = mp.Queue()
		self.p = mp.Process(target=self.__start, args=(self.q, self.host, self.port, self.datasize))
		self.p.start()

	def getTelemetry(self):
		if self.q.empty():
			return None
		return self.q.get()
		
	def stopServer(self):
		print("termination request")
		#self.p.terminate()
		os.kill(self.p.pid, signal.SIGINT)
		self.p.join()
		print("termination completed")
	
	
