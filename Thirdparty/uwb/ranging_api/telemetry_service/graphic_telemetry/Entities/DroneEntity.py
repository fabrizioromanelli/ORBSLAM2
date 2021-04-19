#!/usr/bin/env python3

import pygame
import os
from Entities.BaseEntity import BaseEntity

class DroneEntity(BaseEntity):
	def __init__(self, screen, sx = 60, sy = 60):
		super().__init__("Assets/drone_icon.png", screen, sx, sy)
