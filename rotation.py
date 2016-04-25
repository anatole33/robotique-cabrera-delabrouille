from __future__ import division
from pypot.robot import from_json
from contextlib import closing
from math import *
import time


"""def leg_ik(x,y,z, L1, L2, L3, alpha, beta):
                teta1= degrees(atan2(y,x))
                d13=sqrt(x**2 + y**2)-L1
                d=sqrt(d13**2+z**2)
                teta3=degrees(acos((L2**2+L3**2-d**2)/(2*L2*L3))) - 180
                a=degrees(atan(z/d13))
                b=degrees(acos((L2**2+d**2-L3**2)/(2*L2*d)))
                teta2=a+b
                teta2=-(teta2+alpha)
                teta3=-(abs(teta3)-90+alpha+beta)

                return[teta1,teta2,teta3]"""

constL1 = 51
constL2 = 63.7
constL3 = 93
# Angle to match the theory with reality for theta 2 (measures of the triangle are 22.5, 60.7, 63.7). => Angle =  -20.69
theta2Correction = -20.69
# Same goes for theta 3 : +90 - 20.69 - a. Where a = asin(8.2/93) = 5.06
theta3Correction = 90 + theta2Correction - 5.06

# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c):
    value = ((a*a)+(b*b)-(c*c))/(2*a*b)
    #Note : to get the other altenative, simply change the sign of the return :
    return -acos(value)


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2,l3=constL3) :
    theta1 = theta1 * pi / 180.0
    theta2 = (theta2 - theta2Correction) * pi / 180.0
    theta3 = -(theta3 - theta3Correction) * pi / 180.0

    planContribution = l1 + l2*cos(theta2) + l3*cos(theta2 + theta3)

    x = cos(theta1) * planContribution
    y = sin(theta1) * planContribution
    z = -(l2 * sin(theta2) + l3 * sin(theta2 + theta3))

    return [x, y, z]

# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def leg_ik(x, y, z, l1=constL1, l2=constL2,l3=constL3) :
    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    theta1 = atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = sqrt(x*x+y*y)-l1
    if (xp < 0) :
        print("Destination point too close")
        xp = 0

    # Distance between the second motor arm and the end of the leg
    d = sqrt(pow(xp,2) + pow(z,2))
    if (d > l2+l3):
        print("Destination point too far away")
        d = l2+l3

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    theta2 = alKashi(l2, d, l3) - atan2(z, xp)
    theta3 = pi - alKashi(l2, l3, d)

    return [modulo180(degrees(theta1)), modulo180(degrees(theta2) + theta2Correction), modulo180(degrees(theta3) + theta3Correction)]

#Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle) :
    if (-180 < angle < 180) :
        return angle

    angle  = angle % 360
    if (angle > 180) :
        return -360 + angle

    return angle

def move1_ik(x,y,z):

    u=leg_ik(138.86+x,y,-90.57+z, 51, 63.7, 93)
    my_robot.leg1[0].goal_position= u[0]
    my_robot.leg1[1].goal_position= u[1]
    my_robot.leg1[2].goal_position= u[2]

   
    u=leg_ik(120.26+x, 69.43+y, -90.57+z, 51, 63.7, 93)
    my_robot.leg5[0].goal_position= u[0]
    my_robot.leg5[1].goal_position= u[1]
    my_robot.leg5[2].goal_position= u[2]

    u=leg_ik(120.26+x, -69.43+y,-90.57+z, 51, 63.7, 93)
    my_robot.leg3[0].goal_position= u[0]
    my_robot.leg3[1].goal_position= u[1]
    my_robot.leg3[2].goal_position= u[2]




def move2_ik(x,y,z):
   
    u=leg_ik(138.86+x, y,-90.57+z, 51, 63.7)
    my_robot.leg4[0].goal_position= u[0]
    my_robot.leg4[1].goal_position= u[1]
    my_robot.leg4[2].goal_position= u[2]

    u=leg_ik(120.26+x, 69.43+y, -90.57+z, 51, 63.7, 93)
    my_robot.leg2[0].goal_position= u[0]
    my_robot.leg2[1].goal_position= u[1]
    my_robot.leg2[2].goal_position= u[2]

    u=leg_ik(120.26+x, -69.43+y, -90.57+z, 51, 63.7, 93)
    my_robot.leg6[0].goal_position= u[0]
    my_robot.leg6[1].goal_position= u[1]
    my_robot.leg6[2].goal_position= u[2]
    

with closing(from_json('Json.json')) as my_robot:
    for m in my_robot.motors:
        m.compliant=False

    y=50
    
    while True:
        time.sleep(0.3)

        move1_ik(0,0,15)
        time.sleep(0.3)
        move1_ik(0,y,15)
        time.sleep(0.3)
        move1_ik(0,y,0)
        time.sleep(0.3)
        move2_ik(0,0,15)
        time.sleep(0.3)
        move2_ik(0,y,15)
        time.sleep(0.3)
        move2_ik(0,y,0)
        time.sleep(0.3)
        move1_ik(0,0,0)
        move2_ik(0,0,0)
