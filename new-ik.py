import math
import turtle

turtle.hideturtle()
turtle.radians()

len1 = 10.0
len2 = 10.0

def DrawArm(angle1, angle2, tX, tY):
    turtle.penup()
    turtle.setpos(tX * 10, tY * 10)
    turtle.stamp()
    turtle.setpos(0,0)
    turtle.setheading(angle1)
    turtle.pendown()
    turtle.forward(len1 * 10)
    turtle.setheading(angle2)
    turtle.forward(len2 * 10)
    turtle.penup()

#law of cosines to solve for one angle
def LawOfCos(a, b, c):
    return math.acos((a*a + b*b - c*c) / (2 * a * b))

def dist(x, y):
    return math.sqrt(x*x+y*y)

while(True):
    targX = float(input())
    targY = float(input())
    D1 = math.atan(targY/targX)
    distance = dist(targX, targY)
    D2 = LawOfCos(distance, len1, len2)
    A1 = D1 + D2
    A2 = LawOfCos(len1, len2, distance)
    A2x = A1 - (math.pi - A2)
    DrawArm(A1, A2x, targX, targY)

