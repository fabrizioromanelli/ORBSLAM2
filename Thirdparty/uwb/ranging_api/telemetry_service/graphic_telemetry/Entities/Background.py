import pygame
import math

class GridBackground():
	def __init__(self, screen, background_color, grid_color, cell_size):
		self.screen = screen
		self.bgc = background_color
		self.grc = grid_color
		self.size = cell_size
		
		
	def draw(self):
		self.screen.fill(self.bgc)
		for i in range(int(self.screen.get_height() / self.size)):
			icoord = i * self.size
			pygame.draw.line(self.screen, self.grc, (0, icoord), (self.screen.get_width(), icoord))
		
		for j in range(int(self.screen.get_width() / self.size)):
			jcoord = j * self.size
			pygame.draw.line(self.screen, self.grc, (jcoord, 0), (jcoord, self.screen.get_height()))

class RadialGridBackground():
	def __init__(self, screen, background_color, grid_color, cell_size):
		self.screen = screen
		self.bgc = background_color
		self.grc = grid_color
		self.center = (int(self.screen.get_width() / 2), int(self.screen.get_height() / 2))
		self.n_circles = int(max(self.screen.get_height(), self.screen.get_width()) / cell_size)
		self.linsize = cell_size
		self.angular_steps = 30
		
	def draw(self):
		self.screen.fill(self.bgc)
		
		for i in range(self.n_circles):
			iradius = (i + 1) * self.linsize
			pygame.draw.circle(self.screen, self.grc, self.center, iradius, 1)
		
		rmax = max(self.screen.get_width(), self.screen.get_height()) / 2
		for i in range(int(360 / self.angular_steps)):
			angle = (i * self.angular_steps) * math.pi / 180
			spx = rmax * math.cos(angle) + self.screen.get_width() / 2
			spy = rmax * math.sin(angle) + self.screen.get_height() / 2
			print((spx, spy))
			pygame.draw.line(self.screen, self.grc, (spx, spy), self.center)	
		
		#pygame.draw.line(self.screen, self.grc, (0, 0), (self.screen.get_width(), self.screen.get_height()))
		#pygame.draw.line(self.screen, self.grc, (self.screen.get_width() / 2, 0), (self.screen.get_width() / 2, self.screen.get_height()))
		#pygame.draw.line(self.screen, self.grc, (self.screen.get_width(), 0), (0, self.screen.get_height()))
		#pygame.draw.line(self.screen, self.grc, (0, self.screen.get_height() / 2), (self.screen.get_width(), self.screen.get_height() / 2))
	
