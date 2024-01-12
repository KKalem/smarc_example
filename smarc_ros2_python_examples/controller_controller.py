#!/usr/bin/python3

import rclpy
import sys

from pid_model import PIDModel

class ControllerController:
    def __init__(self,
                 model: PIDModel):
        self._model = model