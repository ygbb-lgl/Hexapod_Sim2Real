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
"""Xbox gamepad reader implemented with Pygame.

Note: Button indices vary across controllers/drivers (SDL, xpad, bluetooth, etc).
This module provides a sane default mapping for common SDL Xbox layouts and a
legacy fallback mapping for older setups.
"""

import threading
import time
import os
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
        # Common SDL (Linux) Xbox layout (note: some controllers swap X/Y indices):
        #   A=0, B=1, X=2, Y=3, LB=4, RB=5, BACK=6, START=7, XBOX=8, LS=9, RS=10
        # Legacy layout (previous project default):
        #   A=0, B=1, X=3, Y=4, LB=6
        self._button_map = button_map  # may be None; resolved after joystick connects

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

        if self._button_map is None:
            self._button_map = self._choose_default_button_map()

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def _choose_default_button_map(self):
        preset = (os.environ.get("HEXAPOD_GAMEPAD_PRESET") or "").strip().lower()
        legacy = {"A": 0, "B": 1, "X": 3, "Y": 4, "LB": 6}
        # Default for this repo: X/Y swapped controller (observed on some pads).
        sdl_xbox = {"A": 0, "B": 1, "X": 3, "Y": 2, "LB": 4}

        if preset in {"legacy", "old"}:
            return legacy
        if preset in {"sdl", "standard", "xbox"}:
            return sdl_xbox

        # Heuristic default: SDL/Xbox mapping is most common on modern Linux.
        default_map = dict(sdl_xbox)

        # Optional per-button overrides, e.g.:
        #   export HEXAPOD_GAMEPAD_BTN_Y=4
        #   export HEXAPOD_GAMEPAD_BTN_LB=6
        for key in ("A", "B", "X", "Y", "LB"):
            env_key = f"HEXAPOD_GAMEPAD_BTN_{key}"
            if env_key not in os.environ:
                continue
            try:
                default_map[key] = int(os.environ[env_key])
            except Exception:
                pass

        return default_map

    def _read_button(self, name: str) -> int:
        if self._joystick is None:
            return 0
        idx = self._button_map.get(name, None) if isinstance(self._button_map, dict) else None
        if idx is None:
            return 0
        try:
            idx = int(idx)
        except Exception:
            return 0
        if idx < 0:
            return 0
        try:
            if idx >= int(self._joystick.get_numbuttons()):
                return 0
            return int(self._joystick.get_button(idx))
        except Exception:
            return 0

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
                a = self._read_button("A")
                b = self._read_button("B")
                x = self._read_button("X")
                y = self._read_button("Y")
                lb = self._read_button("LB")

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