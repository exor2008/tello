import time
import ctypes
from abc import ABC, abstractmethod

import cv2
import numpy as np
import pygame as pg
from screeninfo import get_monitors
from djitellopy import tello

FPS = 30

class App:
    def __init__(self, device, fps=FPS):
        pg.init()
        self.bg = (17, 148, 218)
        self.fg = (241, 93, 29)
        self.init_font()
        self.init_screen()
        self.init_device(device)
        self.init_info()
        self.clock = pg.time.Clock()

        self.fps = fps
        self._running = True

    def init_device(self, device):
        if device == 'notebook':
            self.drone = NotebookCamera()
        else:
            self.drone = Drone()

    def init_screen(self):
        self.width, self.height = resolution()
        self.screen = pg.display.set_mode([self.width, self.height])#, pg.FULLSCREEN)
        self.screen.fill(self.bg)
        self.text('Loading...', self.width // 2, self.height // 2)
        pg.display.update()

    def init_info(self):
        self.info = Info(self.screen, self.drone)

    def init_font(self):
        self.font = pg.font.Font(pg.font.get_default_font(), 36)

    def text(self, txt, x, y):
        text = self.font.render(txt, True, self.fg)
        text_rect = text.get_rect(center=(x, y))
        self.screen.blit(text, text_rect)

    def run(self):
        while self._running:
            self.clock.tick(self.fps)
            self.on_info()
            self.on_event()
            self.on_render()
            self.drone.update_rc()
            # time.sleep(0.01)
        self.on_exit()

    def on_info(self):
        self.info.update()

    def on_event(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self._running = False
            if event.type == pg.KEYUP:
                if event.key == pg.K_ESCAPE:
                    self._running = False
                elif event.key == pg.K_0:
                    self.drone.land()
                elif event.key == pg.K_1:
                    self.drone.takeoff()
                elif event.key == pg.K_DELETE:
                    self.drone.emergency()

        keys = pg.key.get_pressed()

        if keys[pg.K_w]:
            self.drone.forward()            
        elif keys[pg.K_s]:
            self.drone.backward()
        else:
            self.drone.waste_forward_backward()

        if keys[pg.K_a]:
            self.drone.left()
        elif keys[pg.K_d]:
            self.drone.right()
        else:
            self.drone.waste_left_right()

        if keys[pg.K_SPACE]:
            self.drone.up()
        elif keys[pg.K_c]:
            self.drone.down()
        else:
            self.drone.waste_up_down()

        if keys[pg.K_q]:
            self.drone.cw()
        elif keys[pg.K_e]:
            self.drone.ccw()
        else:
            self.drone.waste_yaw()

    def on_render(self):
        self.draw_frame()
        pg.display.update()

    def on_exit(self):
        self.drone.release()
        pg.quit()

    def draw_frame(self):
        frame = self.drone.get_frame()
        self.screen.blit(frame, (0, 0))


class Device(ABC):
    def __init__(self):
        self.width, self.height = resolution()

    @abstractmethod
    def get_frame(self):
        pass

    @abstractmethod
    def release(self):
        pass


class NotebookCamera(Device):
    def __init__(self):
        super().__init__()
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def get_frame(self):
        success, image = self.cam.read()
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = np.fliplr(image)
        image = cv2.resize(image, (self.width, self.height))
        image = np.swapaxes(image, 0, 1)
        surf = pg.pixelcopy.make_surface(image)

        return surf

    def release(self):
        self.cam.release()

    def left(self):
        print('left')

    def right(self):
        print('right')

    def forward(self):
        print('forward')

    def backward(self):
        print('backward')

    def up(self):
        print('up')

    def down(self):
        print('down')

    def cw(self):
        print('cw')

    def ccw(self):
        print('ccw')

    def land(self):
        print('land')

    def takeoff(self):
        print('takeoff')

    def emergency(self):
        print('emergency')

    def stop(self):
        print('stop')


def resolution():
    [monitor] = get_monitors()
    return monitor.width, monitor.height


class Drone(Device):
    def __init__(self):
        super().__init__()
        self.drone = tello.Tello()
        self.drone.connect()
        self.drone.streamon()
        self.stop()
        self.update_rc()

    @property
    def battery(self):
        return self.drone.get_battery()

    @property
    def altittude(self):
        return self.drone.get_height()

    @property
    def temperature(self):
        return self.drone.get_temperature()

    @property
    def noise(self):
        return self.drone.query_wifi_signal_noise_ratio()

    def get_frame(self):
        image = self.drone.get_frame_read().frame
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # image = np.fliplr(image)
        image = cv2.resize(image, (1440, 1080))
        image = np.swapaxes(image, 0, 1)
        surf = pg.pixelcopy.make_surface(image)

        return surf

    def update_rc(self):
        self.drone.send_rc_control(
            self.left_right_velocity,
            self.forward_backward_velocity,
            self.up_down_velocity,
            self.yaw_velocity)

    def release(self):
        self.drone.streamoff()
        self.stop()
        self.update_rc()
        self.drone.end()

    # left right
    def right(self):
        self._gain_velocity_plus('left_right_velocity', 100)

    def left(self):
        self._gain_velocity_minus('left_right_velocity', -100)

    def waste_left_right(self):
        self._waste_velocity('left_right_velocity')

    # forward backward
    def forward(self):
        self._gain_velocity_plus('forward_backward_velocity', 100)

    def backward(self):
        self._gain_velocity_minus('forward_backward_velocity', -100)

    def waste_forward_backward(self):
        self._waste_velocity('forward_backward_velocity')

    # up down
    def up(self):
        self._gain_velocity_plus('up_down_velocity', 100)

    def down(self):
        self._gain_velocity_minus('up_down_velocity', -100)

    def waste_up_down(self):
        self._waste_velocity('up_down_velocity')

    # yaw
    def ccw(self):
        self._gain_velocity_plus('yaw_velocity', 100)

    def cw(self):
        self._gain_velocity_minus('yaw_velocity', -100)

    def waste_yaw(self):
        self._waste_velocity('yaw_velocity')

    def stop(self):
        self.left_right_velocity = 0
        self.forward_backward_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0

    def land(self):
        self.drone.land()

    def takeoff(self):
        self.drone.takeoff()

    def emergency(self):
        self.drone.emergency()

    def _gain_velocity_plus(self, velocity, max_vel):
        vel = getattr(self, velocity)

        vel += 3
        vel = max(0, min(vel, max_vel))

        setattr(self, velocity, vel)

    def _gain_velocity_minus(self, velocity, min_vel):
        vel = getattr(self, velocity)

        vel -= 3
        vel = max(min_vel, min(vel, 0))

        setattr(self, velocity, vel)

    def _waste_velocity(self, velocity):
        vel = getattr(self, velocity)

        if vel > 0:
            vel -= 10
            vel = max(0, vel)
        else:
            vel += 10
            vel = min(0, vel)

        setattr(self, velocity, vel)


class Info:
    def __init__(self, screen, drone):
        self.bg = (17, 148, 218)
        self.fg = (211, 82, 27)
        self.screen = screen
        self.drone = drone
        self.last_upd_time = time.localtime().tm_sec
        self.init_font(30)
        self.init_text_area()

    def update(self):
        now = time.localtime().tm_sec
        if now != self.last_upd_time:
            self.last_upd_time = now
            self._update()

    def init_font(self, size):
        self.font = pg.font.Font(pg.font.get_default_font(), size)

    def text(self, txt, x, y):
        text = self.font.render(txt, True, self.fg)
        text_rect = text.get_rect(topleft=(x, y))
        return text, text_rect

    def init_text_area(self):
        self.area = pg.surface.Surface([480, 1080])
        self.rect = self.area.get_rect(topleft=(1440, 0))
        self.area.fill(self.bg)

    def _update(self):
        self.area.fill(self.bg)

        txt = f'Forward speed {self.drone.forward_backward_velocity}'
        text, rect = self.text(txt, 10, 10)
        self.area.blit(text, rect)

        txt = f'Side speed {self.drone.left_right_velocity}'
        text, rect = self.text(txt, 10, rect.y + rect.h + 10)
        self.area.blit(text, rect)

        txt = f'Z speed {self.drone.up_down_velocity}'
        text, rect = self.text(txt, 10, rect.y + rect.h + 10)
        self.area.blit(text, rect)

        txt = f'Yaw speed {self.drone.yaw_velocity}'
        text, rect = self.text(txt, 10, rect.y + rect.h + 10)
        self.area.blit(text, rect)

        txt = f'Height {self.drone.altittude}'
        text, rect = self.text(txt, 10, rect.y + rect.h + 10)
        self.area.blit(text, rect)

        txt = f'Battery {self.drone.battery}'
        text, rect = self.text(txt, 10, rect.y + rect.h + 10)
        self.area.blit(text, rect)

        celsius = (self.drone.temperature - 32) * 5 / 9
        txt = f'Temperature {celsius:.1f}'
        text, rect = self.text(txt, 10, rect.y + rect.h + 10)
        self.area.blit(text, rect)

        txt = f'WiFi Signal {self.drone.noise}'
        text, rect = self.text(txt, 10, rect.y + rect.h + 10)
        self.area.blit(text, rect)

        self.screen.blit(self.area, self.rect)