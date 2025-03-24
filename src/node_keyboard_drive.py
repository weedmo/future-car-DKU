#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

# Keyboard drive node
# ICELAB (Han Sol Kim)

import rospy
from std_msgs.msg import Int16

import pygame
from pygame.locals import *

def render_text(text, x, y):
    text_surface = font.render(text, True, WHITE)
    screen.blit(text_surface, (x, y))

if __name__ == '__main__':
    pygame.init()
    screen_width, screen_height = 600, 300
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption('Keyboard drive by ICELAB')

    font = pygame.font.Font(None, 36)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)

    rospy.init_node('keyboard_drive')
    pub_lat = rospy.Publisher('latt_cmd', Int16, queue_size=1)
    pub_lon = rospy.Publisher('longi_cmd', Int16, queue_size=1)
    msg_lat = Int16()
    msg_lon = Int16()

    latt = 0
    longi = 0
    LOW_SPEED, HIGH_SPEED = 100, 200
    MAX_LATT_CMD = 600
    vel = LOW_SPEED
    loop_hz = 50
    rate = rospy.Rate(loop_hz)

    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_w:
                    longi = vel
                elif event.key == K_s:
                    longi = -vel
                elif event.key == K_a:
                    latt = MAX_LATT_CMD
                elif event.key == K_d:
                    latt = -MAX_LATT_CMD
                elif event.key == K_LSHIFT:
                    vel = HIGH_SPEED
            elif event.type == KEYUP:
                if event.key in (K_w, K_s):
                    longi = 0
                elif event.key in (K_a, K_d):
                    latt = 0
                elif event.key == K_q:
                    running = False
                elif event.key == K_LSHIFT:
                    vel = LOW_SPEED

        msg_lat.data = latt
        msg_lon.data = longi
        pub_lat.publish(msg_lat)
        pub_lon.publish(msg_lon)

        screen.fill(BLACK)
        render_text("Press 'w', 's', 'a', or 'd'", 50, 50)
        render_text("Press 'L_SHIFT' to increase speed", 50, 100)
        render_text(f'latt: {latt}, longi: {longi}', 50,150)
        pygame.display.flip()
        
        rate.sleep()

    pygame.quit()