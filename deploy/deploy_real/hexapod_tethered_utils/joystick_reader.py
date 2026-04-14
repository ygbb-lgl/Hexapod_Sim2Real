# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Xbox 360 Gamepad class that uses Pygame under the hood."""

import threading
import time
import numpy as np
import pygame
from typing import List


def _interpolate(value, new_scale, deadzone=0.01):
    if abs(value) < deadzone:
        return 0.0
    return value * new_scale


class Gamepad:
    """Gamepad class that reads from an Xbox 360 gamepad using Pygame."""

    def __init__(
        self,
        joystick_id=0,
        vel_scale_x=0.8,
        vel_scale_y=0.5,
        vel_scale_rot=0.5,
        button_map=None,
    ):
        try:
            pygame.mixer.quit()
        except:
            pass
        pygame.init()
        # 禁用不需要的模块以避免 ALSA 报错
        try:
            pygame.mixer.quit()
        except:
            pass
            
        pygame.joystick.init()
        
        self._joystick_id = joystick_id
        self._vel_scale_x = vel_scale_x
        self._vel_scale_y = vel_scale_y
        self._vel_scale_rot = vel_scale_rot

        # Pygame button index mapping (varies by controller/driver).
        self._button_map = button_map or {"A": 0, "B": 1, "X": 3, "Y": 4, "LB": 6}

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Button states
        # level: 1 pressed / 0 released
        # clicked: rising-edge (True only on press transition)
        self.a = 0
        self.b = 0
        self.x = 0
        self.y = 0
        self.lb = 0
        self.a_clicked = False
        self.b_clicked = False
        self.x_clicked = False
        self.y_clicked = False
        self.lb_clicked = False
        self._prev_a = 0
        self._prev_b = 0
        self._prev_x = 0
        self._prev_y = 0
        self._prev_lb = 0

        self.is_running = True

        self._joystick = None
        self._connect_device()

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def _connect_device(self):
        try:
            if pygame.joystick.get_count() > self._joystick_id:
                self._joystick = pygame.joystick.Joystick(self._joystick_id)
                self._joystick.init()
                print(f"Connected to {self._joystick.get_name()}")
                return True
            else:
                print("No joystick found at ID", self._joystick_id)
                return False
        except Exception as e:
            print(f"Error connecting to device: {e}")
            return False

    def read_loop(self):
        while self.is_running:
            try:
                pygame.event.pump()  # Process event queue
                
                # For Xbox 360 controller:
                # Left stick: axis 0 (horizontal), axis 1 (vertical)
                # Right stick: axis 3 (horizontal)
                vy = -self._joystick.get_axis(0)
                vx = -self._joystick.get_axis(1)  # Invert Y axis
                wz = -self._joystick.get_axis(3) 

                # Buttons
                a = int(self._joystick.get_button(self._button_map["A"]))
                b = int(self._joystick.get_button(self._button_map["B"]))
                x = int(self._joystick.get_button(self._button_map["X"]))
                y = int(self._joystick.get_button(self._button_map["Y"]))
                lb = int(self._joystick.get_button(self._button_map["LB"]))

                self.a = a
                self.b = b
                self.x = x
                self.y = y
                self.lb = lb

                self.a_clicked = bool(a and not self._prev_a)
                self.b_clicked = bool(b and not self._prev_b)
                self.x_clicked = bool(x and not self._prev_x)
                self.y_clicked = bool(y and not self._prev_y)
                self.lb_clicked = bool(lb and not self._prev_lb)
                self._prev_a = a
                self._prev_b = b
                self._prev_x = x
                self._prev_y = y
                self._prev_lb = lb
                
                # # 乘scale系数，并应用死区
                self.vx = _interpolate(vx, 1)
                self.vy = _interpolate(vy, 1)
                self.wz = _interpolate(wz, 1)
                # self.vx = _interpolate(vx, self._vel_scale_x)
                # self.vy = _interpolate(vy, self._vel_scale_y)
                # self.wz = _interpolate(wz, self._vel_scale_rot)
                
                time.sleep(0.01)  # Small delay to prevent CPU overuse
                
            except Exception as e:
                print(f"Error reading from device: {e}")
                time.sleep(1)  # Wait before retrying
                if not self._connect_device():
                    self.is_running = False
                    return

    # 获取当前命令（线速度 vx, vy 和角速度 wz）
    def get_command(self):
        return np.array([self.vx, self.vy, self.wz])

    # A 键：按下=1，松开=0
    def get_button_a(self) -> int:
        return int(self.a)

    def get_button_b(self) -> int:
        return int(self.b)

    def get_button_x(self) -> int:
        return int(self.x)

    def get_button_y(self) -> int:
        return int(self.y)

    def get_button_lb(self) -> int:
        return int(self.lb)

    # A 键：只在“按下瞬间”返回 True，并自动清除（用于触发一次性动作）
    def consume_a_click(self) -> bool:
        clicked = bool(self.a_clicked)
        self.a_clicked = False
        return clicked

    def consume_b_click(self) -> bool:
        clicked = bool(self.b_clicked)
        self.b_clicked = False
        return clicked

    def consume_x_click(self) -> bool:
        clicked = bool(self.x_clicked)
        self.x_clicked = False
        return clicked

    def consume_y_click(self) -> bool:
        clicked = bool(self.y_clicked)
        self.y_clicked = False
        return clicked

    def consume_lb_click(self) -> bool:
        clicked = bool(self.lb_clicked)
        self.lb_clicked = False
        return clicked

    def stop(self):
        self.is_running = False
        pygame.quit()

    def get_pressed_button_indices(self) -> List[int]:
        if self._joystick is None:
            return []
        try:
            n = self._joystick.get_numbuttons()
            return [i for i in range(n) if int(self._joystick.get_button(i))]
        except Exception:
            return []


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scan-buttons",
        action="store_true",
        help="Print pressed pygame button indices for mapping calibration",
    )
    args = parser.parse_args()

    gamepad = Gamepad()
    try:
        while True:
            if args.scan_buttons:
                pressed = gamepad.get_pressed_button_indices()
                if pressed:
                    print("pressed button indices:", pressed)
            else:
                print(
                    "cmd:",
                    gamepad.get_command(),
                    "A:",
                    gamepad.get_button_a(),
                    "B:",
                    gamepad.get_button_b(),
                    "X:",
                    gamepad.get_button_x(),
                    "Y:",
                    gamepad.get_button_y(),
                    "LB:",
                    gamepad.get_button_lb(),
                    "click:",
                    {
                        "A": gamepad.consume_a_click(),
                        "B": gamepad.consume_b_click(),
                        "X": gamepad.consume_x_click(),
                        "Y": gamepad.consume_y_click(),
                        "LB": gamepad.consume_lb_click(),
                    },
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        gamepad.stop()