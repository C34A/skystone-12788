import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class sketch_191026a extends PApplet {

final float LEN1 = 100.0f;
final float LEN2 = 100.0f;
float X;
float Y;

boolean keep_going = true;
public void setup() {
    
    frameRate(144);
    //angle1 = (float)Math.atan2(Y, X);
    /*angle1 = HALF_PI;
     angle2 = atan2(Y - LEN1, X);
     updatePositions(angle1, angle2);
     println("first iter:", middle, ",", PVector.add(end, middle));*/
}

public void draw() {
    background(0);

    translate(width / 2, height / 2);
    scale(1, -1);
    ellipseMode(CENTER);
    stroke(255);
    noFill();
    circle(0, 0, LEN1 * 4);
    circle(X, Y, 10);
    //circle(end.x + middle.x, end.y + middle.y, LEN2 * 2);
    stroke(0, 255, 0);
    
    PVector[] points = doIKForDist(X, Y, 5);
    line(0, 0, points[0].x, points[0].y);
    stroke(255, 0, 0);
    line(points[0].x, points[0].y, points[1].x + points[0].x, points[1].y + points[0].y);
}

public void mouseClicked() {
    /*float x = mouseX - width / 2;
     float y = height / 2 - mouseY;
     //float ang = atan2(y, x);
     //println(x, ", ", y, ", ", ang);
     if(distance(x, y) < LEN1 + LEN2)
     doIKForDist(x, y, 10);
     println(distance(PVector.add(end, middle), x, y));
     //updatePositions(ang, ang);*/
    X = mouseX - width / 2;
    Y = height / 2 - mouseY;
}

public double distance(float x, float y) {
    return Math.sqrt(x*x + y*y);
}

public double distance(PVector p, float x, float y) {
    return PVector.sub(p, new PVector(x, y)).mag();
}

private PVector[] doIKForDist(float x, float y, int dist) {
    if(distance(x, y) < LEN1 - LEN2 || distance(x, y) > LEN1 + LEN2) {
        throw new IllegalArgumentException("cannot reach given coordinates");
    }
    
    long before = millis();
    float angle1;
    float angle2;
    PVector middle = new PVector(LEN1, 0);
    PVector end = new PVector(LEN2, 0);

    if (x < 0) {
        angle1 = 0;
    } else {
        angle1 = PI;
    }
    angle2 = atan2(Y - LEN1, X);
    while (distance(PVector.add(end, middle), x, y) > 5) {
        /*println("iter", frameCount, ":", middle, PVector.add(middle, end), x);
         println("iter", frameCount, ":", angle1, angle2, distance(PVector.add(end, middle), x, y));
         println();*/
        if (x > 0) {
            angle1 -= PI / 180;
        } else {
            angle1 += PI / 180;
        }
        angle2 = atan2(y - sin(angle1) * LEN1, x - LEN1 * cos(angle1));

        //update positions
        middle.rotate(angle1 - middle.heading());
        end.rotate(angle2 - end.heading());
    }
    println("time", millis() - before, "\n");
    return new PVector[] {middle, end};
}

    public void settings() {  size(800, 800); }
    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[] { "--present", "--window-color=#666666", "--stop-color=#cccccc", "sketch_191026a" };
        if (passedArgs != null) {
          PApplet.main(concat(appletArgs, passedArgs));
        } else {
          PApplet.main(appletArgs);
        }
    }
}
