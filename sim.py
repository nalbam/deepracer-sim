#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import copy
import datetime
import json
import math
import pygame
import random
import time

from pygame.math import Vector2

from tracks import baadal as track

from functions import TwoDigits as deepracer


TITLE = "DeepRacer Simulator"

DEBUG_LOG = False

FRAME_RATE = 15

SCREEN_RATE = 70

TAIL_LENGTH = 200

MIN_REWARD = 0.0001

STEERING_ANGLE = [-30, -20, -10, 0, 10, 20, 30]
SPEED = 1.0

BOTS_COUNT = 6
BOTS_SPEED = 0

FONT_FACE = "assets/FreeSansBold.ttf"
FONT_SIZE = 24

CAR_BOT = "assets/car-gray.png"
CAR_CONTROLLED = "assets/car-blue.png"
CAR_CRASHED = "assets/car-red.png"
CAR_OFFTRACK = "assets/car-purple.png"
CAR_ORIGIN = "assets/car-green.png"
CAR_WARNED = "assets/car-yello.png"

COLOR_CENTER = (242, 156, 56)
COLOR_CIRCLE = (250, 250, 250)
COLOR_FLOOR = (87, 191, 141)
COLOR_OBJECT = (250, 200, 100)
COLOR_ROAD = (37, 47, 61)
COLOR_SHORTCUT = (150, 150, 150)
COLOR_TRACK = (255, 255, 255)

COLOR_RAY = (100, 255, 255)
COLOR_RAY_TRACK = (255, 100, 100)
COLOR_RAY_SHORTCUT = (255, 255, 100)

COLOR_TEXT = (255, 255, 100)

g_scr_adjust = []
g_scr_rate = SCREEN_RATE
g_scr_width = 0
g_scr_height = 0


def parse_args():
    p = argparse.ArgumentParser(description=TITLE)
    p.add_argument("-a", "--autonomous", action="store_true", help="autonomous")
    p.add_argument("-d", "--draw-lines", action="store_true", help="draw lines")
    p.add_argument("-f", "--full-screen", action="store_true", help="full screen")
    p.add_argument("-s", "--speed", type=float, default=SPEED, help="speed")
    p.add_argument("--bots-count", type=int, default=BOTS_COUNT, help="bots count")
    p.add_argument("--bots-speed", type=float, default=BOTS_SPEED, help="bots speed")
    return p.parse_args()


def get_suffix():
    return datetime.datetime.now().strftime("%H%M")


def get_distance(coor1, coor2):
    return math.sqrt(
        (coor1[0] - coor2[0]) * (coor1[0] - coor2[0])
        + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1])
    )


def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def get_angle(coor1, coor2):
    return math.atan2((coor2[1] - coor1[1]), (coor2[0] - coor1[0]))


def get_angle_degrees(coor1, coor2):
    return math.degrees(get_angle(coor1, coor2))


def get_diff_angle(angle1, angle2):
    diff = (angle1 - angle2) % (2.0 * math.pi)
    if diff >= math.pi:
        diff -= 2.0 * math.pi
    return diff


def get_diff_degrees(angle1, angle2):
    return math.degrees(get_diff_angle(angle1, angle2))


def get_distance_list(pos, waypoints):
    dist_list = []
    min_dist = float("inf")
    min_idx = -1

    for i, waypoint in enumerate(waypoints):
        dist = get_distance(pos, waypoint)
        if dist < min_dist:
            min_dist = dist
            min_idx = i
        dist_list.append(dist)

    return dist_list, min_dist, min_idx, len(waypoints)


def get_closeset(waypoints, pos):
    dist_list, min_dist, min_idx, length = get_distance_list(pos, waypoints)
    return min_idx, (min_idx / length) * 100


def draw_line(surface, color, start_pos, end_pos, width):
    try:
        pygame.draw.line(
            surface,
            color,
            get_adjust_point(start_pos),
            get_adjust_point(end_pos),
            width,
        )
    except Exception as ex:
        print("Error:", ex, start_pos, end_pos, width)


def draw_lines(surface, color, closed, lines, width, dashed):
    if dashed:
        lgngth = len(lines)
        for i in range(0, lgngth - 1):
            if i % 2 == 0:
                draw_line(
                    surface, color, lines[i - 1], lines[i], width,
                )
    else:
        pygame.draw.lines(surface, color, closed, get_adjust_points(lines), width)


def draw_polygon(surface, color, lines):
    try:
        pygame.draw.polygon(surface, color, get_adjust_points(lines))
    except Exception as ex:
        print("Error:", ex, color)


def draw_circle(surface, color, center, radius, width):
    try:
        pygame.draw.circle(
            surface, color, get_adjust_point(center), get_adjust_length(radius), width
        )
    except Exception as ex:
        print("Error:", ex, center, radius, width)


def init_bot(args):
    bots = []

    if args.bots_count < 1:
        return bots

    lanes = [
        get_waypoints("left"),
        get_waypoints("right"),
    ]

    for i in range(0, args.bots_count):
        index = i % len(lanes)
        waypoints = lanes[index]

        start_index = int(len(waypoints) / (args.bots_count + 2)) * (i + 2)
        target_index = (start_index + 3) % len(waypoints)

        car_angle = get_angle_degrees(waypoints[start_index], waypoints[target_index])

        car = Car(args, waypoints[start_index], car_angle, args.bots_speed, True)
        bot = Bot(car, waypoints, i % 2 == 0)
        bots.append(bot)

    return bots


class Bot:
    def __init__(self, car, waypoints, is_left):
        self.car = car
        self.waypoints = waypoints
        self.is_left = is_left

    def get_pos(self):
        return self.car.get_pos()

    def get_angle(self):
        return self.car.get_angle()

    def left_of_center(self):
        if self.is_left:
            return 1
        else:
            return 0

    def move(self, surface):
        pos = self.car.get_pos()

        dist_list, min_dist, min_idx, length = get_distance_list(pos, self.waypoints)

        index = (min_idx + 3) % len(self.waypoints)

        angle = math.radians(self.car.get_angle())
        target_angle = get_angle(pos, self.waypoints[index])

        diff_angle = get_diff_degrees(angle, target_angle)

        if abs(diff_angle) > 15:
            if diff_angle > 0:
                target = -15
            else:
                target = 15
        else:
            target = 0

        self.car.move(surface, target, False, False)


class Car:
    def __init__(self, args, pos, angle, speed, is_bot):
        # global g_scr_rate

        self.args = args

        self.images = {
            "bot": pygame.image.load(CAR_BOT).convert_alpha(),
            "controlled": pygame.image.load(CAR_CONTROLLED).convert_alpha(),
            "crashed": pygame.image.load(CAR_CRASHED).convert_alpha(),
            "offtrack": pygame.image.load(CAR_OFFTRACK).convert_alpha(),
            "origin": pygame.image.load(CAR_ORIGIN).convert_alpha(),
            "warned": pygame.image.load(CAR_WARNED).convert_alpha(),
        }

        self.image = self.images["origin"]

        self.vel = Vector2((speed / FRAME_RATE), 0)

        self.pos = Vector2(pos)
        self.rect = self.image.get_rect(center=pos)

        self.angle = angle * -1
        self.vel.rotate_ip(angle)

        self.is_bot = is_bot

    def get_pos(self):
        return self.pos

    def get_angle(self):
        return self.angle * -1

    def move(self, surface, angle, offtrack=False, crashed=False, warned=False):
        # global g_scr_width
        # global g_scr_height

        angle *= -1

        keys = pygame.key.get_pressed()

        self.key_pressed = False

        # pos
        if self.is_bot:
            self.pos += self.vel
        elif keys[pygame.K_UP]:
            self.pos += self.vel
            self.key_pressed = True
        elif keys[pygame.K_DOWN]:
            self.pos -= self.vel
            self.key_pressed = True
        elif self.args.autonomous:
            self.pos += self.vel

        # self.pos[0] = min(max(self.pos[0], 0), g_scr_width)
        # self.pos[1] = min(max(self.pos[1], 0), g_scr_height)

        self.rect.center = get_adjust_point(self.pos)

        # angle
        if self.is_bot:
            if abs(angle) > 0:
                self.angle += angle
                self.vel.rotate_ip(-angle)
        elif keys[pygame.K_LEFT]:
            self.angle -= 10
            self.vel.rotate_ip(10)
            self.key_pressed = True
        elif keys[pygame.K_RIGHT]:
            self.angle += 10
            self.vel.rotate_ip(-10)
            self.key_pressed = True
        elif self.args.autonomous:
            if abs(angle) > 0:
                self.angle += angle
                self.vel.rotate_ip(-angle)

        if self.angle > 180:
            self.angle = self.angle - 360
        elif self.angle < -180:
            self.angle = self.angle + 360

        # car
        if self.is_bot:
            image = self.images["bot"]
        elif offtrack:
            image = self.images["offtrack"]
        elif crashed:
            image = self.images["crashed"]
        elif warned:
            image = self.images["warned"]
        elif self.key_pressed:
            image = self.images["controlled"]
        else:
            image = self.images["origin"]

        self.image = pygame.transform.rotate(image, -self.angle)

        self.rect = self.image.get_rect(center=self.rect.center)
        mask_car = pygame.mask.from_surface(self.image)

        # draw car
        surface.blit(self.image, self.rect)

        return self.pos, (self.angle * -1)


def run():
    global g_scr_adjust
    global g_scr_rate
    global g_scr_width
    global g_scr_height

    args = parse_args()

    prev_time = float("inf")
    record = float("inf")

    steps = 0
    prev_prograss = 100

    start_time = time.time()

    tails = []

    # pygame
    pygame.init()

    clock = pygame.time.Clock()

    # title
    pygame.display.set_caption(TITLE)

    # screen
    if args.full_screen:
        surface = pygame.display.set_mode((0, 0), pygame.FULLSCREEN, 32)

        width, height = pygame.display.Info().current_w, pygame.display.Info().current_h

        print("screen", width, height)

        g_scr_width = width
        g_scr_height = height
    else:
        adjust, rate, width, height = get_adjust()

        print("screen", width, height)

        g_scr_width = width
        g_scr_height = height

        surface = pygame.display.set_mode((g_scr_width, g_scr_height))

    # track
    inside_lines = get_waypoints("inside")
    outside_lines = get_waypoints("outside")
    center_lines = get_waypoints("center")

    track_width = get_distance(inside_lines[0], outside_lines[0])

    # laptime
    font = pygame.font.Font(FONT_FACE, FONT_SIZE)

    laptime = font.render("", True, COLOR_TEXT, COLOR_FLOOR)
    laptime_rect = laptime.get_rect(center=(20, 30))

    latest = font.render("", True, COLOR_TEXT, COLOR_FLOOR)
    latest_rect = laptime.get_rect(center=(20, 60))

    # car angle
    car_angle = get_angle_degrees(center_lines[0], center_lines[1])

    # init car
    car = Car(args, center_lines[0], car_angle, args.speed, False)

    # init bots
    bots = init_bot(args)

    run = True
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break

        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE] or keys[pygame.K_q]:
            run = False

        if run == False:
            break

        # fill bg
        surface.fill(COLOR_FLOOR)

        # draw track
        draw_polygon(surface, COLOR_ROAD, outside_lines)
        draw_polygon(surface, COLOR_FLOOR, inside_lines)

        # draw lines
        draw_lines(surface, COLOR_TRACK, False, inside_lines, 5, False)
        draw_lines(surface, COLOR_TRACK, False, outside_lines, 5, False)
        draw_lines(surface, COLOR_CENTER, False, center_lines, 5, True)

        # car
        pos = car.get_pos()
        heading = car.get_angle()

        dist_list, min_dist, min_idx, length = get_distance_list(pos, center_lines)
        closest2 = (min_idx + 1) % length

        closest_waypoints = [min_idx, closest2]

        dist_inside = get_distance(pos, inside_lines[min_idx])
        dist_outside = get_distance(pos, outside_lines[min_idx])

        if dist_inside < dist_outside:
            is_left_of_center = True
        else:
            is_left_of_center = False

        if min_dist > (track_width * 0.5):
            offtrack = True
        else:
            offtrack = False

        # objects
        closest_objects = []
        objects_location = []
        objects_distance = []
        objects_left_of_center = []

        crashed = False
        warned = False

        if len(bots) > 0:
            # draw_bots
            for i, bot in enumerate(bots):
                bot.move(surface)

                obj_pos = bot.get_pos()

                objects_location.append([obj_pos[0], obj_pos[1]])
                objects_distance.append(get_distance(pos, obj_pos))
                objects_left_of_center.append(bot.left_of_center())

            bot_dist = min(objects_distance)
            bot_idx = objects_distance.index(bot_dist)
            closest_objects = objects_location[bot_idx]

            if bot_dist < track_width:
                warned = True

        # tails
        tails.append([pos[0], pos[1]])
        if len(tails) > TAIL_LENGTH:
            del tails[0]
        if len(tails) > 1:
            draw_lines(surface, COLOR_SHORTCUT, False, tails, 2, False)

        # progress
        index, progress = get_closeset(center_lines, pos)

        if steps > 0 and prev_prograss > progress:
            steps = 0
            start_time = time.time()
        steps += 1
        prev_prograss = progress

        # dummy
        params = {
            "all_wheels_on_track": offtrack == False,
            "closest_objects": closest_objects,
            "closest_waypoints": closest_waypoints,
            "crashed": crashed,
            "distance_from_center": min_dist,
            "heading": heading,
            "is_left_of_center": is_left_of_center,
            "is_reversed": False,
            "objects_distance": objects_distance,
            "objects_left_of_center": objects_left_of_center,
            "objects_location": objects_location,
            "offtrack": offtrack,
            "progress": progress,
            "speed": args.speed,
            "steering_angle": 0,
            "steps": steps,
            "track_width": track_width,
            "waypoints": center_lines,
            "x": pos[0],
            "y": pos[1],
        }

        # pick target
        rewards = []

        pick = -1
        max_reward = MIN_REWARD

        target = []
        target_angle = 0

        for i, steering_angle in enumerate(STEERING_ANGLE):
            params["steering_angle"] = steering_angle

            reward = deepracer.reward_function(params)

            if DEBUG_LOG:
                print("reward", i, round(reward, 5))

            if reward > max_reward:
                rewards.clear()
                rewards.append(i)
                max_reward = reward
            elif reward == max_reward:
                rewards.append(i)

        n = len(rewards)

        if n == 0:
            target_angle = 0
        elif n == 1:
            pick = rewards[0]
            target_angle = STEERING_ANGLE[pick]
        else:
            i = random.randint(0, n - 1)
            pick = rewards[i]
            target_angle = STEERING_ANGLE[pick]

        print("pick", pick, round(max_reward, 5), target_angle, warned)

        # moving
        pos, heading = car.move(surface, target_angle, offtrack, crashed, warned)

        # time
        race_time = time.time() - start_time

        # laptime
        s = "{:3.3f}".format(race_time)
        laptime = font.render(s, True, COLOR_TEXT, COLOR_FLOOR)

        if progress == 0 and prev_time > 5:
            record = prev_time
        prev_time = race_time

        # latest
        if record < 60 and record > 5:
            s = "{:3.3f}".format(record)
            latest = font.render(s, True, COLOR_TEXT, COLOR_FLOOR)

        surface.blit(laptime, laptime_rect)
        surface.blit(latest, latest_rect)

        # draw lines
        if args.draw_lines:
            draw_circle(surface, COLOR_CIRCLE, pos, track_width, 1)

            if warned:
                draw_line(surface, COLOR_OBJECT, pos, closest_objects, 2)

            target = [
                track_width * math.cos(math.radians(heading)) + pos[0],
                track_width * math.sin(math.radians(heading)) + pos[1],
            ]
            if target:
                draw_line(surface, COLOR_RAY, pos, target, 3)

        # pygame.display.flip()
        pygame.display.update()
        clock.tick(FRAME_RATE)

    pygame.quit()


def get_adjust():
    global g_scr_adjust
    global g_scr_rate
    global g_scr_width
    global g_scr_height

    if len(g_scr_adjust) > 0:
        return g_scr_adjust, g_scr_rate, g_scr_width, g_scr_height

    min_x = float("inf")
    min_y = float("inf")
    max_x = float("-inf")
    max_y = float("-inf")

    lines = track.get_outside_waypoints()

    for point in lines:
        min_x = min(min_x, point[0])
        min_y = min(min_y, point[1])

        max_x = max(max_x, point[0])
        max_y = max(max_y, point[1])

    print("min", min_x, min_y)
    print("max", max_x, max_y)

    if g_scr_width == 0 or g_scr_height == 0:
        g_scr_width = int(((max_x - min_x) + ((max_x - min_x) * 0.05)) * g_scr_rate)
        g_scr_height = int(((max_y - min_y) + ((max_y - min_y) * 0.05)) * g_scr_rate)

    x = ((g_scr_width / g_scr_rate) * 0.5) - ((max_x + min_x) * 0.5)
    y = ((g_scr_height / g_scr_rate) * 0.5) - ((max_y + min_y) * 0.5)

    g_scr_adjust = [x, y]

    print("adjust", g_scr_adjust)

    return g_scr_adjust, g_scr_rate, g_scr_width, g_scr_height


def get_waypoints(key):
    waypoints = None
    if key == "center":
        waypoints = track.get_center_waypoints()
    elif key == "inside":
        waypoints = track.get_inside_waypoints()
    elif key == "outside":
        waypoints = track.get_outside_waypoints()
    elif key == "left":
        waypoints = get_merge_waypoints(
            track.get_inside_waypoints(), track.get_center_waypoints()
        )
    elif key == "left2":
        waypoints = get_merge_waypoints(
            track.get_inside_waypoints(),
            get_merge_waypoints(
                track.get_inside_waypoints(), track.get_center_waypoints()
            ),
        )
    elif key == "right":
        waypoints = get_merge_waypoints(
            track.get_center_waypoints(), track.get_outside_waypoints()
        )
    elif key == "right2":
        waypoints = get_merge_waypoints(
            get_merge_waypoints(
                track.get_center_waypoints(), track.get_outside_waypoints()
            ),
            track.get_outside_waypoints(),
        )
    # return get_adjust_points(waypoints)
    return waypoints


def get_adjust_length(val):
    adjust, rate, width, height = get_adjust()

    return val * rate


def get_adjust_point(point):
    adjust, rate, width, height = get_adjust()

    if rate == 1:
        return point

    x = (point[0] + adjust[0]) * rate
    y = height - ((point[1] + adjust[1]) * rate)
    return [x, y]


def get_adjust_points(points):
    results = []
    for point in points:
        results.append(get_adjust_point(point))
    return results


def get_merge_waypoints(points1, points2):
    length = min(len(points1), len(points2))
    results = []
    for i in range(0, length):
        x = (points1[i][0] + points2[i][0]) * 0.5
        y = (points1[i][1] + points2[i][1]) * 0.5
        results.append([x, y])
    return results


if __name__ == "__main__":
    run()
