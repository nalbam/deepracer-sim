#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import numpy as np
import pygame
import random
import time

from pygame.math import Vector2

from tracks import track202303 as track

from functions import custom as deepracer


TITLE = "DeepRacer Simulator"

DEBUG_LOG = False

FRAME_RATE = 15  # fps

SCREEN_RATE = 100  # % of screen size

TAIL_LENGTH = 100

MIN_REWARD = 0.0001

STEERING_ANGLE = [-30, -20, -10, 0, 10, 20, 30]
SPEED = 1.0

BOTS_COUNT = 0
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
    p.add_argument("-a", "--autonomous", default=False, action="store_true", help="autonomous")
    p.add_argument("-d", "--draw-lines", default=True, action="store_true", help="draw lines")
    p.add_argument("-f", "--full-screen", default=False, action="store_true", help="full screen")
    p.add_argument("-s", "--speed", type=float, default=SPEED, help="speed")
    p.add_argument("--bots-count", type=int, default=BOTS_COUNT, help="bots count")
    p.add_argument("--bots-speed", type=float, default=BOTS_SPEED, help="bots speed")
    p.add_argument("--debug", default=DEBUG_LOG, action="store_true", help="debug")
    return p.parse_args()


def get_distance(coor1, coor2):
    return math.sqrt((coor1[0] - coor2[0]) * (coor1[0] - coor2[0]) + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1]))


def up_sample(waypoints, factor=10):
    p = waypoints
    n = len(p)

    return [
        [
            i / factor * p[int((j + 1) % n)][0] + (1 - i / factor) * p[j][0],
            i / factor * p[int((j + 1) % n)][1] + (1 - i / factor) * p[j][1],
        ]
        for j in range(n)
        for i in range(factor)
    ]


def get_target(pos, angle, dist):
    return [
        dist * math.cos(math.radians(angle)) + pos[0],
        dist * math.sin(math.radians(angle)) + pos[1],
    ]


def get_radians(coor1, coor2):
    return math.atan2((coor2[1] - coor1[1]), (coor2[0] - coor1[0]))


def get_degrees(coor1, coor2):
    return math.degrees(get_radians(coor1, coor2))


def get_diff_radians(angle1, angle2):
    diff = (angle1 - angle2) % (2.0 * math.pi)
    if diff >= math.pi:
        diff -= 2.0 * math.pi
    return diff


def get_diff_degrees(angle1, angle2):
    return math.degrees(get_diff_radians(angle1, angle2))


def get_distance_list(pos, waypoints):
    dist_list = []
    min_dist = float("inf")
    min_idx = -1

    for i, p in enumerate(waypoints):
        dist = get_distance(pos, p)
        if dist < min_dist:
            min_dist = dist
            min_idx = i
        dist_list.append(dist)

    return dist_list, min_dist, min_idx, len(waypoints)


def get_angle_list(pos, waypoints):
    angle_list = []
    dist_list = []

    for i, p in enumerate(waypoints):
        angle = get_degrees(pos, p)
        angle_list.append(angle)

        dist = get_distance(pos, p)
        dist_list.append(dist)

    return angle_list, dist_list, len(waypoints)


def draw_line(surface, color, start_pos, end_pos, width):
    try:
        pygame.draw.line(surface, color, get_adjust_point(start_pos), get_adjust_point(end_pos), width)
    except Exception as ex:
        print("Error:", ex, start_pos, end_pos, width)


def draw_lines(surface, color, closed, lines, width, dashed):
    try:
        if dashed:
            for i in range(0, len(lines) - 1):
                if i % 2 == 0:
                    draw_line(surface, color, lines[i - 1], lines[i], width)
        else:
            pygame.draw.lines(surface, color, closed, get_adjust_points(lines), width)
    except Exception as ex:
        print("Error:", ex, color)


def draw_polygon(surface, color, lines):
    try:
        pygame.draw.polygon(surface, color, get_adjust_points(lines))
    except Exception as ex:
        print("Error:", ex, color)


def draw_circle(surface, color, center, radius, width):
    try:
        pygame.draw.circle(surface, color, get_adjust_point(center), get_adjust_length(radius), width)
    except Exception as ex:
        print("Error:", ex, center, radius, width)


def intersection(x1, y1, x2, y2, x3, y3, x4, y4):
    # Calculate the intersection point between two line segments
    # Based on the algorithm from https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

    det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if det == 0:
        return None

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / det
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / det

    if 0 <= t <= 1 and 0 <= u <= 1:
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        return (x, y)

    return None


def get_collision(pos, angles, walls, dist):
    collisions = []
    for angle in angles:
        target = get_target(pos, angle, dist)

        intersections = []
        for i in range(0, len(walls) - 1):
            intersection_point = intersection(pos[0], pos[1], target[0], target[1], walls[i][0], walls[i][1], walls[i + 1][0], walls[i + 1][1])
            if intersection_point is not None:
                intersections.append(intersection_point)

        if len(intersections) > 0:
            collisions.append(min(intersections, key=lambda x: get_distance(pos, x)))

    return max(collisions, key=lambda x: get_distance(pos, x)) if len(collisions) > 0 else None


def find_destination(pos, heading, inside, outside, closest_idx, length, track_width):
    angles = [heading + angle for angle in range(-50, 50)]

    cut = length // 3
    dist = track_width * 20

    inside_cut = inside[closest_idx : closest_idx + cut]
    if len(inside_cut) < cut:
        inside_cut += inside[: cut - len(inside_cut)]
    outside_cut = outside[closest_idx : closest_idx + cut]
    if len(outside_cut) < cut:
        outside_cut += outside[: cut - len(outside_cut)]

    walls = inside_cut + outside_cut[::-1]

    collision = get_collision(pos, angles, walls, dist)

    if collision is not None:
        angle = get_degrees(pos, collision)
        angles = np.arange(angle - 1, angle + 1, 0.1)
        return get_collision(pos, angles, walls, dist)

    return None


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

        car_angle = get_degrees(waypoints[start_index], waypoints[target_index])

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

    def move(self, surface, pause=False):
        pos = self.car.get_pos()

        _, _, min_idx, _ = get_distance_list(pos, self.waypoints)

        index = (min_idx + 3) % len(self.waypoints)

        angle = math.radians(self.car.get_angle())
        target_angle = get_radians(pos, self.waypoints[index])
        diff_angle = get_diff_degrees(angle, target_angle)

        if abs(diff_angle) > 15:
            if diff_angle > 0:
                angle = -15
            else:
                angle = 15
        else:
            angle = 0

        self.car.move(surface, angle, pause, False, False)


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

    def move(self, surface, angle, pause=False, offtrack=False, crashed=False, warned=False):
        # global g_scr_width
        # global g_scr_height

        angle *= -1

        keys = pygame.key.get_pressed()

        self.key_pressed = False

        if pause == False:
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

        if pause == False:
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

        scale_width = int(self.image.get_width() * (g_scr_rate / 100))
        scale_height = int(self.image.get_height() * (g_scr_rate / 100))
        self.image = pygame.transform.scale(self.image, (scale_width, scale_height))

        self.rect = self.image.get_rect(center=self.rect.center)
        pygame.mask.from_surface(self.image)

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
        _, _, width, height = get_adjust()

        print("screen", width, height)

        g_scr_width = width
        g_scr_height = height

        surface = pygame.display.set_mode((g_scr_width, g_scr_height))

    # track
    inside = get_waypoints("inside")
    outside = get_waypoints("outside")

    waypoints = get_waypoints("center")

    shortcut = get_waypoints("shortcut")

    track_width = get_distance(inside[0], outside[0])

    # laptime
    font = pygame.font.Font(FONT_FACE, FONT_SIZE)

    laptime = font.render("", True, COLOR_TEXT, COLOR_FLOOR)
    laptime_rect = laptime.get_rect(center=(20, 30))

    latest = font.render("", True, COLOR_TEXT, COLOR_FLOOR)
    latest_rect = laptime.get_rect(center=(20, 60))

    # car angle
    car_angle = get_degrees(waypoints[0], waypoints[1])

    # init car
    car = Car(args, waypoints[0], car_angle, args.speed, False)

    # init bots
    bots = init_bot(args)

    run = True
    pause = False
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                break

        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE] or keys[pygame.K_q]:
            run = False
        if keys[pygame.K_SPACE] or keys[pygame.K_p]:
            pause = pause == False

        if run == False:
            break

        # fill bg
        surface.fill(COLOR_FLOOR)

        # draw track
        draw_polygon(surface, COLOR_ROAD, outside)
        draw_polygon(surface, COLOR_FLOOR, inside)

        # draw lines
        draw_lines(surface, COLOR_TRACK, False, inside, 5, False)
        draw_lines(surface, COLOR_TRACK, False, outside, 5, False)

        draw_lines(surface, COLOR_CENTER, False, waypoints, 5, True)

        if len(shortcut) > 0:
            draw_lines(surface, COLOR_SHORTCUT, False, shortcut, 2, True)

        # car
        pos = car.get_pos()
        heading = car.get_angle()

        # closest
        _, closest_dist, closest_idx, waypoints_length = get_distance_list(pos, waypoints)

        closest_waypoints = [closest_idx, (closest_idx + 1) % waypoints_length]

        # progress
        progress = (closest_idx / waypoints_length) * 100
        if steps > 0 and prev_prograss > progress:
            steps = 0
            start_time = time.time()
        steps += 1
        prev_prograss = progress

        if args.debug:
            print("")
            print("run", steps, progress)

        # offtrack
        if closest_dist > (track_width * 0.6):
            offtrack = True
        else:
            offtrack = False

        dist_inside = get_distance(pos, inside[closest_idx])
        dist_outside = get_distance(pos, outside[closest_idx])

        # is_left
        if dist_inside < dist_outside:
            is_left_of_center = True
        else:
            is_left_of_center = False

        # objects
        closest_objects = []
        objects_location = []
        objects_distance = []
        objects_left_of_center = []

        crashed = False
        warned = False

        # draw_bots
        if len(bots) > 0:
            for i, bot in enumerate(bots):
                bot.move(surface, pause)

                obj_pos = bot.get_pos()

                objects_location.append([obj_pos[0], obj_pos[1]])
                objects_distance.append(get_distance(pos, obj_pos))
                objects_left_of_center.append(bot.left_of_center())

            bot_dist = min(objects_distance)
            bot_idx = objects_distance.index(bot_dist)
            closest_objects = objects_location[bot_idx]

            if bot_dist < (track_width * 1.5):
                warned = True

        # tails
        tails.append([pos[0], pos[1]])
        if len(tails) > TAIL_LENGTH:
            del tails[0]
        if len(tails) > 1:
            draw_lines(surface, COLOR_SHORTCUT, False, tails, 2, False)

        # dummy
        params = {
            "all_wheels_on_track": offtrack == False,
            "closest_objects": closest_objects,
            "closest_waypoints": closest_waypoints,
            "crashed": crashed,
            "distance_from_center": closest_dist,
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
            "waypoints": waypoints,
            "x": pos[0],
            "y": pos[1],
            "surface": surface,
        }

        # pick target
        indexes = []
        rewards = []

        pick = -1
        max_reward = MIN_REWARD

        target = []
        angle = 0

        if pause == False:
            for i, steering_angle in enumerate(STEERING_ANGLE):
                params["steering_angle"] = steering_angle

                reward = deepracer.reward_function(params)
                rewards.append("{:03.5f}".format(reward))

                if args.debug:
                    print("reward", i, round(reward, 5), steering_angle)

                if reward > max_reward:
                    indexes.clear()
                    indexes.append(i)
                    max_reward = reward
                elif reward == max_reward:
                    indexes.append(i)

        n = len(indexes)

        if n == 0:
            angle = 0
        elif n == 1:
            pick = indexes[0]
            angle = STEERING_ANGLE[pick]
        else:
            i = random.randint(0, n - 1)
            pick = indexes[i]
            angle = STEERING_ANGLE[pick]

        if pause == False:
            print("pick {} {:03.5f} {}".format(pick, max_reward, rewards))

        # moving
        pos, heading = car.move(surface, angle, pause, offtrack, crashed, warned)

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

            target = get_target(pos, heading, track_width)
            if target:
                draw_line(surface, COLOR_RAY, pos, target, 3)

            destination = find_destination(pos, heading, inside, outside, closest_idx, waypoints_length, track_width * 20)
            if destination:
                draw_line(surface, COLOR_RAY, pos, destination, 1)

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
    if key == "center":
        return track.get_center_waypoints()
    elif key == "inside":
        return track.get_inside_waypoints()
    elif key == "outside":
        return track.get_outside_waypoints()
    elif key == "shortcut":
        return track.get_shortcut_waypoints()
    elif key == "left":
        return get_merge_waypoints(
            track.get_center_waypoints(),
            track.get_inside_waypoints(),
        )
    elif key == "left2":
        return get_border_waypoints(track.get_center_waypoints(), track.get_inside_waypoints(), 0.9)
    elif key == "right":
        return get_merge_waypoints(
            track.get_center_waypoints(),
            track.get_outside_waypoints(),
        )
    elif key == "right2":
        return get_border_waypoints(track.get_center_waypoints(), track.get_outside_waypoints(), 0.9)
    return None


def get_adjust_length(val):
    _, rate, _, _ = get_adjust()

    return int(val * rate)


def get_adjust_point(point):
    adjust, rate, _, height = get_adjust()

    if rate == 1:
        return point

    x = (point[0] + adjust[0]) * rate
    y = height - ((point[1] + adjust[1]) * rate)
    return [int(x), int(y)]


def get_adjust_points(points):
    results = []
    for point in points:
        results.append(get_adjust_point(point))
    return results


def get_merge_waypoints(points1, points2, rate=0.5):
    length = min(len(points1), len(points2))
    results = []
    for i in range(0, length):
        results.append(
            [
                (points1[i][0] + points2[i][0]) * rate,
                (points1[i][1] + points2[i][1]) * rate,
            ]
        )
    return results


def get_border_waypoints(points1, points2, rate=1.2):
    length = min(len(points1), len(points2))
    results = []
    for i in range(0, length):
        dist = get_distance(points1[i], points2[i]) * rate
        angle = get_radians(points1[i], points2[i])
        results.append(
            [
                dist * math.cos(angle) + points1[i][0],
                dist * math.sin(angle) + points1[i][1],
            ]
        )
    return results


if __name__ == "__main__":
    run()
