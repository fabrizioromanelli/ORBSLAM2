#!/usr/bin/env python3

import pygame
import os
from Entities.BaseEntity import BaseEntity

class RoverEntity(BaseEntity):
	def __init__(self, screen, color, sx = 60, sy = 60):
		super().__init__("Assets/rover_icon.png", screen, sx, sy)
		self.sprite.fill((0, 0, 0, 255), None, pygame.BLEND_RGBA_MULT)
		self.sprite.fill(color[0:3] + (0,), None, pygame.BLEND_RGBA_ADD)
