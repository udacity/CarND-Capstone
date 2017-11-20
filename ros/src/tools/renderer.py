import cv2
import numpy as np
import pygame

class Canvas:
    def __init__(self, width=2500, height=2500, channels=3):
        self.width = width
        self.height = height
        self.channels = channels
        self.data = None
        self.clear()

    def clear(self, grayscale=True):
        self.data = np.zeros((self.width, self.height, self.channels), dtype=np.uint8)
        if grayscale is True:
            self.data.fill(40)

    def get(self):
        return self.data


class PyGameScreen:
    def __init__(self, width=1200, height=800, caption='Default Screen'):
        pygame.init()
        pygame.display.set_caption(caption)
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF)
        self.canvas = Canvas()
        pygame.display.flip()

    def update(self):
        sim_img = pygame.image.frombuffer(
            cv2.resize(self.canvas.get(),
                       (self.width, self.height),
                       interpolation=cv2.INTER_AREA).tobytes(),
            (self.width, self.height),
            'RGB')

        self.screen.blit(sim_img, (0, 0))
        pygame.display.flip()

    def clear_canvas(self):
        self.canvas.clear()

    def get_canvas(self):
        return self.canvas

    def write_text(self,
                   txt,
                   pos,
                   font=cv2.FONT_HERSHEY_DUPLEX,
                   font_size=2,
                   color=(255, 255, 255),
                   thickness=2):
        cv2.putText(self.canvas.get(),
                    txt,
                    pos,
                    font,
                    font_size,
                    color,
                    thickness)

    def get_center_of_screen(self):
        return self.width // 2, self.height // 2

