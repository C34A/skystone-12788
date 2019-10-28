final float LEN1 = 100.0;
final float LEN2 = 100.0;
float X;
float Y;

boolean keep_going = true;
void setup() {
    size(800, 800);
    frameRate(144);
    //angle1 = (float)Math.atan2(Y, X);
    /*angle1 = HALF_PI;
     angle2 = atan2(Y - LEN1, X);
     updatePositions(angle1, angle2);
     println("first iter:", middle, ",", PVector.add(end, middle));*/
}

void draw() {
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
    try{
        PVector[] points = doIKForDist((mouseX - width / 2), (height / 2 - mouseY), 5);
        line(0, 0, points[0].x, points[0].y);
        stroke(255, 0, 0);
        line(points[0].x, points[0].y, points[1].x + points[0].x, points[1].y + points[0].y);
    } catch (Exception e) {
        println(e);
    }
}

void mouseClicked() {
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

double distance(float x, float y) {
    return Math.sqrt(x*x + y*y);
}

double distance(PVector p, float x, float y) {
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
    angle2 = atan2(y - LEN1, x);
    int i = 0;
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
        i++;
    }
    println("millis:", millis() - before
    );
    println("iterations taken:", i);
    println(middle, end);
    return new PVector[] {middle, end};
}
