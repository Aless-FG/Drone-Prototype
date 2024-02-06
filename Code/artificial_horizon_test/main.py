import serial
import time
import pygame

def blitRotateCenter(surf, image, topleft, angle):
    rotated_image = pygame.transform.rotate(image, angle)
    new_rect = rotated_image.get_rect(center = image.get_rect(topleft = topleft).center)
    surf.blit(rotated_image, new_rect)

def draw_window(set_pitch, set_roll, scale_factor = 1):
    screen.fill(bgColor)
    blitRotateCenter(screen, interior, (-width/2, -height/2 + set_pitch*scale_factor), set_roll)
    blitRotateCenter(screen, inel, (0, 0), set_roll)
    screen.blit(frame, (0,0))
    pygame.display.update()


pygame.display.set_caption("Attitude Indicator")
size = width, height = 500, 500
bgColor = 255, 255, 255
FPS = 60
scale_factor = 7

screen = pygame.display.set_mode(size)

frame = pygame.image.load("./Frame.png").convert_alpha()
inel = pygame.image.load("./Ring.png").convert_alpha()
interior = pygame.image.load("./Interior.png").convert_alpha()


ser = serial.Serial('COM4', 9800, timeout=1)
time.sleep(2)
lista = [];
while True:
    # Reading all bytes available bytes till EOL
    line = ser.readline()
    if line:
        # Converting Byte Strings into unicode strings
        string = line.decode()
        lista = string.split(";")
        # Converting Unicode String into integer
        #print("Roll:" + lista[-2])
        print("Pitch:" + lista[-1] + "Roll:" + lista[-2])
        draw_window(float(lista[-1]), float(lista[-2]), scale_factor)
ser.close()