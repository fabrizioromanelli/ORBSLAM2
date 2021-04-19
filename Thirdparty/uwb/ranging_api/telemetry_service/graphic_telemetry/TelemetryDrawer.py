#!/usr/bin/env python3

import pygame
import math
from Entities.DroneEntity import DroneEntity
from Entities.RoverEntity import RoverEntity
from Entities.Background import GridBackground
from Entities.Background import RadialGridBackground
from Entities.TextBox import TextBox

class TelemetryPrinter :

	# Parameters:
	
	MODE_X = 800 # width of the screen
	MODE_Y = 600 # height of the screen
	CENTER_X = MODE_X / 2 # screen center, do not modify
	CENTER_Y = MODE_Y / 2 # screen center, do not modify
	TRANSLATION_X = 0;
	TRANSLATION_Y = 0;
	PX2METER = 40 # Pixel to meter scale
	
	SPRITE_SIZE = 30; # size of the sprites

	# Default colors: 
	R1_COLOR = (100, 100, 200) # Blue
	R2_COLOR = (100, 200, 100) # Green
	R3_COLOR = (200, 100, 100) # Red
	D0_COLOR = (128, 128, 128)
	NEUTRAL_COLOR = (250, 250, 250)
	BLACK_COLOR = (150, 150, 150)

	
	def __init__(self):
		pygame.init()
		self.screen = pygame.display.set_mode((self.MODE_X, self.MODE_Y))
		
		#self.gridBG = GridBackground(self.screen, self.NEUTRAL_COLOR, self.BLACK_COLOR, self.PX2METER)
		self.gridBG = RadialGridBackground(self.screen, self.NEUTRAL_COLOR, self.BLACK_COLOR, self.PX2METER)
		self.TxtBox = TextBox(self.screen)

		self.Drone = DroneEntity(self.screen, self.SPRITE_SIZE, self.SPRITE_SIZE)
		self.Rover1 = RoverEntity(self.screen, self.R1_COLOR, self.SPRITE_SIZE, self.SPRITE_SIZE)
		self.Rover2 = RoverEntity(self.screen, self.R2_COLOR, self.SPRITE_SIZE, self.SPRITE_SIZE)
		self.Rover3 = RoverEntity(self.screen, self.R3_COLOR, self.SPRITE_SIZE, self.SPRITE_SIZE)

	def __screenClean(self):
		self.gridBG.draw()
	
	def __drawBuffer(self):
		pygame.display.flip()

	def __tx_coord(self, coord):
		return self.PX2METER * coord + self.CENTER_X - self.TRANSLATION_X
	
	def __ty_coord(self, coord):
		return self.PX2METER * coord + self.CENTER_Y - self.TRANSLATION_Y

	def updateGfx(self, r1_coords, r2_coords, r3_coords, d0_coords):
	
		self.TRANSLATION_X = self.PX2METER * d0_coords[0]
		self.TRANSLATION_Y = self.PX2METER * d0_coords[1]
	
		self.__screenClean()
		
		self.TxtBox.draw(r1_coords, r2_coords, r3_coords, d0_coords)
		
		self.Rover1.update(self.__tx_coord(r1_coords[0]), self.__ty_coord(r1_coords[1]), 0)
		self.Rover2.update(self.__tx_coord(r2_coords[0]), self.__ty_coord(r2_coords[1]), 0)
		self.Rover3.update(self.__tx_coord(r3_coords[0]), self.__ty_coord(r3_coords[1]), 0)
		self.Drone.update (self.__tx_coord(d0_coords[0]), self.__ty_coord(d0_coords[1]), 0)
		self.__drawBuffer()
		
	def closeWindowRequired(self):
		# Check for external events. If the quit button has been pressed,
		# the system must be closed.
		# If this function returns true then the external layer must quit
		# the program
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				return True
		return False
