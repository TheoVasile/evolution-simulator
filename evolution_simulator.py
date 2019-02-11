import pygame as pg
from math import sqrt, degrees, atan, sin, cos, radians
from random import randint, randrange
import time

pg.init()

#initialize constand variables
w = 600
h = 500
screen = pg.display.set_mode((w,h))
pg.display.set_caption("Evolution Simulator")
clock = pg.time.Clock()
meter = 5
fps = 120
gravity = 1/fps
ground_rect = pg.Rect(0,h-100,w,100)

def dist(x1,y1,x2,y2):
    return sqrt(((x2-x1)**2)+((y2-y1)**2))

#-----------------------------------------------------------------------------------------------------------------------

class joint:

    #initialize joint variables
    def __init__(self,x,y,radius,friction):
        self.x = x
        self.y = y
        self.friction = friction
        self.vx = 0
        self.vy = 0
        self.ax = 0
        self.ay = 0
        self.radius = radius*meter
        self.theoretical_positions = []
        self.tension = []

    #pivot the joint around a fixed point
    def move(self,pivot_point,arm_length):
        #initiate variables
        self.pivot_point = pivot_point
        self.arm_length = arm_length
        self.xi = self.x
        self.yi = self.y
        self.x += self.vx
        self.y += self.vy
        print ("pos = ({},{}), pivot = ({},{})".format(self.x,self.y,self.pivot_point[0],self.pivot_point[1]))

        #move the point along arm until its within range of the arm
        self.displacement = dist(self.x,self.y,self.pivot_point[0],self.pivot_point[1]) - self.arm_length
        self.move_to_target(self.pivot_point,self.displacement)

        #update variables
        self.vx = (self.x - self.xi) * 0.99 #account for wind resistance by slowing down swing
        self.vy = (self.y - self.yi) * 0.99 #account for wind resistance by slowing down swing
        self.vy += gravity

    #move a point towards a target
    def move_to_target(self,target_pos, displace):
        #initialize variables
        self.displace = displace
        self.target_x = target_pos[0]
        self.target_y = target_pos[1]

        #calculate the angle needed to move to the desired position
        self.rise = self.target_y - self.y
        self.run = self.target_x - self.x

        #account for angle irregularities
        try:
            self.slope = self.rise/self.run
            self.angle = degrees(atan(self.slope))
        except:
            if self.rise > 0:
                self.angle = 90
            elif self.rise < 0:
                self.angle = -90
            elif self.rise == 0:
                self.angle = 0
        if self.run < 0:
            self.angle += 180

        #find displacement across x & y axis
        self.x_displace = self.displace * cos(radians(self.angle))
        self.y_displace = self.displace * sin(radians(self.angle))

        #adjust coordinates
        self.x += self.x_displace
        self.y += self.y_displace

    #account for gravity in joint
    def fall(self):
        self.vy += gravity
        if self.ground_collide():
            self.vy = 0
            self.y = h - 100
        self.y += self.vy

    #check for joint collion with the ground
    def ground_collide(self):
        if ground_rect.collidepoint(self.x, self.y + self.radius):
            return True
        return False

    #draw the joint
    def display(self):
        pg.draw.circle(screen, (250, 255*(1-self.friction), 255*(1-self.friction)), (int(self.x), int(self.y)), self.radius, 0)

    #return position of joint
    def get_pos(self):
        return int(self.x),int(self.y)

    #set new coordinates for the joints
    def set_pos(self,x,y):
        self.x = x
        self.y = y

    #return friction of the joint
    def get_friction(self):
        return self.friction

#-----------------------------------------------------------------------------------------------------------------------

class limb:
    def __init__(self,joint1,joint2,min_length,max_length,extend_range,strength):
        self.tick = 0
        self.joint1 = joint1
        self.joint2 = joint2
        self.bone_length = dist(self.joint1.get_pos()[0],self.joint1.get_pos()[1],self.joint2.get_pos()[0],self.joint2.get_pos()[1])
        self.min_length = self.bone_length * min_length
        self.max_length = self.bone_length * max_length
        self.extend_range = extend_range
        self.strength = strength
        self.percentage = self.bone_length - self.min_length
        self.percentage = (self.percentage * (100 / (self.max_length - self.min_length)))/100

    #swing joints around each other
    def pivot(self):
        pg.draw.line(screen,(0,0,0),self.joint1.get_pos(),self.joint2.get_pos(),2)
        self.joint2.move(self.joint1.get_pos(),self.bone_length)
        pg.draw.circle(screen,(250,0,0),self.joint2.get_pos(),5,0)

    #extend & retract the joint
    def extend(self):

        #find the percentage by which to extend the limb length
        self.percentage = self.percentage*100
        if int(self.percentage) in range(0,101):
            if self.tick in self.extend_range:
                self.percentage += 10 * self.strength
            else:
                self.percentage -= 10 * self.strength
        if self.percentage > 100:
            self.percentage = 100
        if self.percentage < 0:
            self.percentage = 0
        self.percentage = self.percentage/100

        #adjust the length of the bone
        self.new_bone_length = (self.max_length - self.min_length) * (self.percentage)
        self.new_bone_length += self.min_length

        #move the nodes to the ends of the bone
        self.displacement = (self.bone_length - self.new_bone_length) / 2
        self.displacement_joint1 = self.displacement
        self.displacement_joint2 = self.displacement
        self.x1, self.y1 = self.joint1.get_pos()
        self.x2, self.y2 = self.joint2.get_pos()
        #print ("displacement = {}".format(self.displacement))

        #update variables
        self.joint1.move_to_target((self.x2, self.y2), self.displacement)
        self.joint2.move_to_target((self.x1, self.y1), self.displacement)

        #account for friction
        if self.joint1.ground_collide():
            #self.xf,self.yf = self.joint1.get_pos()
            self.friction = 1 - self.joint1.get_friction()
            self.displacement_joint1 = self.displacement * self.friction
            #self.x_displace = self.xf - self.x1
            #print("x_displace = {}".format(self.x_displace))
            #self.x_displace = self.x_displace * (1 - self.friction)
            #print("x_displace = {}".format(self.x_displace))
            #self.xf -= self.x_displace
            #self.joint1.set_pos(self.xf,self.yf)
            #print("friction = {}".format(1 - self.friction))
            #print("----------------------------------")
        if self.joint2.ground_collide():
            self.xf,self.yf = self.joint2.get_pos()
            self.friction = self.joint2.get_friction()
            self.x_displace = self.xf - self.x2
            print("x_displace = {}".format(self.x_displace))
            self.x_displace = self.x_displace * (1 - self.friction)
            print("x_displace = {}".format(self.x_displace))
            self.xf -= self.x_displace
            self.joint2.set_pos(self.xf,self.yf)
            print("friction = {}".format(1 - self.friction))
            print("----------------------------------")

        self.bone_length = self.new_bone_length
        #print("bone length = {}".format(self.bone_length))
        #print("joint_dist = {}".format(dist(joint1.get_pos()[0],joint1.get_pos()[1],joint2.get_pos()[0],joint2.get_pos()[1])))
        self.tick += 1
        if self.tick == fps:
            self.tick = 0

    #make sure all the limbs are at the length they are supposed to be
    def evaluate(self):
        pass

    #apply gravity to limb
    def fall(self):
        #initiate variables
        self.x1f, self.y1f = self.joint1.get_pos()
        self.x2f, self.y2f = self.joint2.get_pos()
        self.length = dist(self.x1f,self.y1f,self.x2f,self.y2f)

        #apply gravity to joints
        self.joint1.fall()
        self.joint2.fall()

        #check for ground collision
        if self.joint1.ground_collide() and not self.joint2.ground_collide():
            self.joint2.move(self.joint1.get_pos(),self.length)
        elif self.joint2.ground_collide() and not self.joint1.ground_collide():
            self.joint1.move(self.joint2.get_pos(),self.length)

    #display the limb
    def display(self):
        pg.draw.line(screen, (255*(1-self.strength), 255*(1-self.strength), 255*(1-self.strength)), self.joint1.get_pos(), self.joint2.get_pos(), 2)
        self.joint1.display()
        self.joint2.display()

    #find if a joint can be found in the limb
    def find(self,joint):
        if joint == self.joint1 or joint == self.joint2:
            return True
        return False

#-----------------------------------------------------------------------------------------------------------------------

class animal:

    def __init__(self):
        self.joints = []
        self.limbs = []
        self.joint_positions = []
        #randomly generate joints
        for x in range(0,randint(3,5)):
            self.pos = (randint(w/2-50,w/2+50),randint(h/2-50,h/2+50))
            while self.pos in self.joint_positions:
                self.pos = (randint(w / 2 - 50, w / 2 + 50), randint(h / 2 - 50, h / 2 + 50))
            self.joints.append(joint(self.pos[0],self.pos[1],1,randint(0,100)/100))
            self.joint_positions.append(self.pos)

        #attatch muscles to joints
        self.invalid_joints = self.joints.copy()
        for self.j in self.joints:
            self.other_joints = self.joints.copy()
            self.other_joints.remove(self.j)
            for self.l in self.limbs:
                for self.j2 in self.other_joints:
                    if self.l.find(self.j) and self.l.find(self.j2):
                        self.other_joints.remove(self.j2)

            #make sure every joint is connected to at least 2 other joints
            if len(self.joints) - len(self.other_joints) < 3:
                for x in range(2):
                    if len(self.other_joints) > 0:
                        self.selected_joint = self.other_joints[randint(0,len(self.other_joints)-1)]
                        self.limbs.append(limb(self.j,self.selected_joint,  randint(10,99) / 100,randint(101,200) / 100,range(randint(0,fps/2),randint(fps/2,fps)),randint(1,100)/100))
                        self.other_joints.remove(self.selected_joint)
                        self.invalid_joints.append((self.j,self.selected_joint))

            #randomly decide if a joint should create a connection with another joint
            for self.k in self.other_joints:
                if (self.k,self.j) in self.invalid_joints or (self.j,self.k) in self.invalid_joints:
                    if [False,True,False][randint(0,2)]:
                        self.limbs.append(limb(self.j, self.selected_joint, randint(10, 99) / 100, randint(101, 200) / 100,range(randint(0,fps/2),randint(fps/2,fps)), randint(1, 100) / 100))
                        self.invalid_joints.append((self.j,self.k))

    #extend and contract each muscle
    def extend(self):
        for self.l in self.limbs:
            self.l.extend()
            self.l.fall()

    #center the animal to the middle of the screen
    def center(self):
        self.screen_center = w/2
        self.positions = []
        self.x_positions = []
        for self.j in self.joints:
            self.jx, self.jy = self.j.get_pos()
            self.positions.append((self.jx,self.jy))
            self.x_positions.append(self.jx)
        self.average_joint_position = sum(self.x_positions)/len(self.x_positions)
        self.x_displace = self.average_joint_position - self.screen_center
        for self.index in range(0,len(self.positions)):
            print(self.positions[self.index])
            self.joints[self.index].set_pos(self.positions[self.index][0]-self.x_displace,self.positions[self.index][1])

    #display the animal
    def display(self):
        #for self.i in self.joints:
        #    self.i.display()
        for self.l in self.limbs:
            self.l.display()

#-----------------------------------------------------------------------------------------------------------------------

#initiate variables
joint1 = joint(w/2,100,1,1)
joint2 = joint(w/2+1,20,1,1)
body = limb(joint1,joint2,0.1,1,range(0,1),1)
limb1 = limb(joint1,joint2,0.1,2,range(20,60),1)
running = True
tock = 0
animol = animal()
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    #generate a new creature every few seconds or on a click
    if tock == fps * 10 or pg.mouse.get_pressed()[0]:
        animol = animal()
        tock = 0

    #display everything to the screen
    screen.fill((145,200,255))
    pg.draw.rect(screen,(0,200,0),ground_rect,0)
    animol.extend()
    animol.center()
    animol.display()
    #body.pivot()
    #limb1.extend()
    #limb1.fall()
    #limb1.display()

    #update variables
    tock += 1
    pg.display.update()
    clock.tick(fps)