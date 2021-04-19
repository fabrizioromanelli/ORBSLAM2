#!/usr/bin/env python3

from abc import ABC, abstractmethod
import pygame

class BaseEntity(ABC):
	def __init__(self, icon_path, screen, sx = 60, sy = 60):
		self.sizex = sx;
		self.sizey = sy;
		self.screen_hook = screen
		tmp_sprite = pygame.image.load(icon_path).convert_alpha()
		self.sprite = pygame.transform.scale(tmp_sprite, (self.sizex, self.sizey))
		self.rect = self.sprite.get_rect();
		
		
	def update(self, px, py, theta):
		theta %= 360 
		rotated_image = pygame.transform.rotate(self.sprite, theta)
		translation_rect = self.rect.move((px - self.sizex / 2, py - self.sizey / 2))

		self.screen_hook.blit(rotated_image, translation_rect)
	
