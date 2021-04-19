import pygame


class TextBox():
	def __init__(self, screen, font_size = 14, padding = 10):
		self.FSIZE = font_size
		self.font = pygame.font.SysFont("Liberation Serif", self.FSIZE)
		self.screen = screen;
		self.defaultText = "Rover1: %.2f\t%.2f\nRover2: %.2f\t%.2f\nRover3: %.2f\t%.2f\nDrone:  %.2f\t%.2f"
		self.PADDING = padding
	
	def draw(self, r1, r2, r3, d):
		complete_text = (self.defaultText % (r1[0], r1[1], r2[0], r2[1], r3[0], r3[1], d[0], d[1])).split("\n")
		max_len = len(max(complete_text))
		n_rows = len(complete_text);
		
		position_x = self.screen.get_width() - (max_len * self.FSIZE/2) - self.PADDING
		position_y = self.PADDING
		
		pygame.draw.rect(self.screen, (255, 255, 255), (position_x, position_y, (max_len * self.FSIZE/2) + 2 * self.PADDING, (n_rows + 1) * self.PADDING + (n_rows) * self.FSIZE))
		
		for i in range(n_rows):
			text_surface = self.font.render(complete_text[i], True, (0, 0, 0))
			self.screen.blit(text_surface, (position_x, position_y))
			position_y = position_y + self.FSIZE + self.PADDING
