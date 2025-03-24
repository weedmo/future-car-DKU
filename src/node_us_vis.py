#!/usr/bin/env python3
# -*- coding: utf-8 -*- 16

# Ultrasonic visualizer node
# ICELAB (Han Sol Kim)

import pygame
import rospy
from std_msgs.msg import Int16MultiArray
import sys

def render_text(text, x, y):
    text_surface = font.render(text, True, WHITE)
    screen.blit(text_surface, (x, y))

def render_sensor_values(distances):
    full_length = 1360//5
    for i in range(6):
        line_length = distances[i] / 5
        color_value = min(int(distances[i] / 1360 * 255), 255)
        line_color = (255 - color_value, color_value, 0)
        start_x = car_body_outline[0][0] + sensor_offsets[i][0]
        start_y = car_body_outline[0][1] + sensor_offsets[i][1]
        end_x = start_x
        end_y = start_y - line_length if i < 3 else start_y + line_length
        
        end_y_bg = start_y - full_length if i < 3 else start_y + full_length
        pygame.draw.line(screen, GRAY, (start_x, start_y), (end_x, end_y_bg), 5)
        pygame.draw.line(screen, line_color, (start_x, start_y), (end_x, end_y), 5)
        render_text(str(distances[i]), start_x-70, start_y)
    
def ultrasonic_callback(msg):
    distances = msg.data
    draw(distances)

def draw(distances):
    screen.fill(BLACK)
    pygame.draw.polygon(screen, WHITE, car_body_outline, 2)
    render_sensor_values(distances)
    pygame.display.flip()

def ultrasonic_visualizer():
    rospy.init_node('ultrasonic_visualizer', anonymous=True)
    rospy.Subscriber('ultrasonics', Int16MultiArray, ultrasonic_callback)
    
    rate = rospy.Rate(10)  # 10Hz
    distances = [-1,-1,-1,-1,-1,-1]
    draw(distances)

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        rate.sleep()

if __name__ == '__main__':
    pygame.init()
    screen_width, screen_height = 800, 800
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption('ultrasonic visualizer ICELAB')

    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GRAY = (100,100,100)
    font = pygame.font.Font(None, 36)

    car_body_outline = [
        (300, 300),   # Front-left corner
        (500, 300),   # Front-right corner
        (500, 500),   # Rear-right corner
        (300, 500),   # Rear-left corner
    ]

    sensor_offsets = [
        (0, 0),    # Front-left
        (100, 0),      # Front-center
        (200, 0),     # Front-right
        (200, 200),  # Rear-right
        (100, 200),    # Rear-center
        (0, 200)    # Rear-left
    ]
    ultrasonic_visualizer()