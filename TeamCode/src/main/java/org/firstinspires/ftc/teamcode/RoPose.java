package org.firstinspires.ftc.teamcode;


/*
One way to think of a RoPose is as a position and a heading.
But position and heading relative to what? A RoPose is always relative to something.
SupRoPose we have a "global" coordinate system -- maybe we call that the "field".
And then we have an object such as a robot on that field, with RoPose robotRoPoseRelatveToField.
And then we have something like an arm on the robot with RoPose armRoPoseRelativeToRobot.
At this point we can intuit that we should be able to compute the RoPose of the arm relative to the field.

Indeed there is a sort of algebra of RoPoses. In math-speak we would say that RoPoses form a group.
A "group" is a set with
  * an associative binary operation (usually called multiplication), and
  * identity object for the operation,
  * such that every element of the set has an inverse.
The identity RoPose is the one with position (0,0) with heading 0, or [(0, 0), 0] in the toString notation of this class.

Note that the operation needs to be associative, so (A*B)*C = A*(B*C).
However, it doesn't need to be commutative, so B*A may not be equal to A*B.
Indeed, the group for RoPoses is *not* commutative. The order matters.

To describe the multiplication, perhaps it is best to break a RoPose down into a series of instructions.
In this viewpoint, the RoPose [(x, y), theta] can be written as
  1. Drive forward x and strafe left y  -- i.e., shift by (x,y)
  2. Rotate by theta
The "drive forward" and the "strafe left" can be done in either order and we get the same result,
so sometimes it is easier to just compress this to "shift by (x,y)".

However -- and this is very important -- we *cannot* swap the order of shifting and rotating.
With a bit of thought, you should be able to see that
  1. Rotate by theta
  2. Shift by (x,y)
Is equivalent to
  1. Shift by (x,y).rotate(theta)
  2. Rotate by theta
Here, "(x,y).rotate(theta)" means the vector that (x,y) becomes after it is rotated by theta.
And of course that is exactly what the first set of instructions say to do, right? They say
rotate our frame of reference by theta, then shift (x,y) in that rotated from.
But that means we are shifting by the rotated (x,y), or exactly (x,y).rotate(theta).
So in the second set of instructions we do that rotated shift,
and then we have to finish with a RoPose rotation so we end up with the right heading.

All of which tells us that the order of operations is important, but at the same time we now know how to swap
the order of a rotate and a shift!!

The "multiplication" operation can be viewed as simply concatenating the sequence of instructions.
Thus, [(x1, y1), theta1] * [[x2, y2), theta2] is just
  1. Shift by (x1, y1)
  2. Rotate by theta1
  3. Shift by (x2, y2)
  4. Rotate by theta2
We have just seen how to swap the rotate and shift in (2) and (3), so this is equivalent to
  1. Shift by (x1, y1)
  2'. Shift by (x2, y2).rotate(theta1)
  3'. Rotate by theta1
  4. Rotate by theta2
Two shifts in a row can easily by combined, as can two rotates in a row. So we find that
an equivalent set of commands is
  1. Shift by (x1, y1) + (x2, y2).rotate(theta1)
  2. Rotate by theta1 + theta2
In other words,
     [(x1, y1), theta1] * [[x2, y2), theta2] = [(x1, y1) + (x2, y2).rotate(theta1),  theta1 + theta2].

It is also easy to work out the inverse operation.  It should be clear that the following four
instructions end with us back at the original RoPose:
  1. Shift by (x, y)
  2. Rotate by theta
  3. Rotate by -theta
  4. Shift by (-x, -y)
The reason this works is because (3) simply undoes (2), and then (4) undoes (1), leaving us at the beginning.
It follows that the inverse of [(x, y), theta] is given by the instructions
  1. Rotate by -theta
  2. Shift by (-x, -y)
Using the logic for swapping rotate and shift from above, we see that this is equivalent to
the following set of instruction in standard order:
  1. Shift by (-x, -y).rotate(-theta)
  2. Rotate by -theta
It follows that [(x, y), theta].inverse() = [(x, y).neg().rotate(-theta), -theta]

Once we have the basic operations, we can easily define a lot of other useful operations.
For example, we can define the "then do X" operations. Let "this" be a RoPose.
(Here I'll say it in math notation, but it shouldn't be hard translate to Java notation.)

this.thenForward(x) => this * [(x, 0), 0]
this.thenStrafeLeft(y) => this * [(0, y), 0]
this.thenRotate(theta) => this * [(0, 0), theta]
this.thenRotateDegrees(deg) => this.thenRotate(deg * pi / 180)

These functions may seem kind of silly, but when discussing robot motion, it is common to think
in terms of a sequence of actions like this. In that context, these are handy functions to have!

Another useful function is one that says, "If the robot is at "this" relative to the field and we
want to get to "target" relative to the field, then how does the robot have to move?  In other words,
what is target when expressed relative to this?"
Mathematically, this means we want to solve for relRoPose such that this * relRoPose = target.
With a bit of thought, we see that if relRoPose = this.inverse() * target, then
   this * relRoPose = this * this.inverse() * target
                  = (     do nothing    ) * target
                  = target

So what this tells us is that it is useful to have a function something like
this.howToGetTo(target) => this.inverse() * target

Note that we had to be careful about the order of operations again.  If we tried to use
relRoPose2 = target * this.inverse(), then we would have had
   this * relRoPose2 = this * target * this.inverse()
But the operations don't commute, so we can't just say this is target -- most of the time
it won't be equal to target.  We can only cancel something with its inverse if they are
right next to each other. But luckily, the cancellation does work in either order, or
in other words, this * this.inverse() = [(0, 0), 0] = this.inverse() * this, all of
which reduce to the instructions "do nothing".

One last function that can be useful is the "powering" function, which means to compute the RoPose
that is equivalent to multiplying RoPose by itself n times. We will only define this if n is an
integer, and we will assume it isn't a terribly big integer.  But it can be a negative integer!
In math notation, the code for this is something like the following

this.pow(int n) => {
  result = [(0, 0), 0]   // start with the trivial RoPose
  while n > 0 {
    result = result * RoPose
    n = n - 1
  }
  while n < 0 {
    result = result * RoPose.inverse()
    n = n + 1
  }
  return result
}

With the pow function you can write lots of interesting unit tests. For example, you know that
if you drive forward 1 block and then turn 90 degrees to the left 4 times, you end up where you
started.  So you could write a unit test something like
trivial = [(0, 0), 0]
assert [(1, 0), pi/4].pow(4).isCloseTo(trivial)

Of course, we would also need to define the isCloseTo function for this to work!

 */
public class RoPose {
    public final RoVector2D position;
    public final double heading;
    public RoPose(RoVector2D position, double heading) {
        this.position = position;
        this.heading = normalizeAngle(heading);
    }
    
    public static RoPose RoPoseD(double x, double y, double thetaD){
        return new RoPose(x, y, thetaD*Math.PI/180);
    }
    
    public static double normalizeAngle(double theta){
        while (theta < -Math.PI) {
            theta += 2*Math.PI;
        }
        while (theta > Math.PI) {
            theta -= 2*Math.PI;
        }
        return theta;
    }
    public static double normalizeAngleInDegrees(double theta){
        while (theta < -180) {
            theta += 360;
        }
        while (theta > 180) {
            theta -= 360;
        }
        return theta;
    }
    public RoPose(RoPose original) {
        this(original.position, original.heading);
    }
    public RoPose(double x, double y, double heading) {
        this.position = new RoVector2D(x,y);
        this.heading = heading;
    }
    public static RoPose getTrivial(){
        return new RoPose(0, 0, 0);

    }

    public RoPose then(RoPose nextPose) {
        RoVector2D rotatedRelativePosition = nextPose.position.rotate(this.heading);
        RoVector2D newPosition = this.position.add(rotatedRelativePosition);
        double newHeading = this.heading+nextPose.heading;
        return new RoPose(newPosition,newHeading);
    }
    public RoPose then (RoVector2D position, double heading){
        RoPose rel = new RoPose(position, heading);
        return this.then(rel);
    }
    public RoPose then (double x, double y, double heading){
        RoPose rel = new RoPose(x, y, heading);
        return this.then(rel);
    }
    public RoPose thenForward(double x) {
        return this.then(x, 0, 0);
    }


    public RoPose thenStrafeLeft(double y) {
        return this.then(0, y, 0);
    }

    public RoPose thenShift(RoVector2D v) {
        return this.then(v, 0);
    }
    public RoPose thenShift(double x, double y) {
        return this.then(x, y, 0);
    }

    public RoPose thenRotate(double theta) {
        return this.then(0,0, theta);
    }

    public RoPose thenRotateDegrees(double angleInDegrees) {
        return this.then(0,0,angleInDegrees*Math.PI/180.0);
    }

    // if this = RoPose(position, heading), then
    // this is the same as trivial.thenShift(position).thenRotate(heading)
    // this.inverse() is supRoPosed to undo this, so
    // this.inverse() is the same as trivial.thenRotate(-heading).thenShift("-position")
    // but "-position" is written position.neg()
    public RoPose inverse() {
        double inverseHeading = -this.heading;
        RoVector2D inversePosition = this.position.neg().rotate(inverseHeading);
        return new RoPose(inversePosition, inverseHeading);
    }

    public RoPose getRelativeRoPoseFor(RoPose target) {
        return this.inverse().then(target);
    }

    public RoPose toThePower(int n) {
        RoPose result = getTrivial();   // start with the trivial RoPose
        while (n > 0) {
            result = result.then(this);
            n = n - 1;
        }
        while (n < 0) {
            result = result.then(this.inverse());
            n = n + 1;
        }
        return result;
    }


    public double length(double weight) {
        double squaredLength = 0.0;
        squaredLength += Math.pow(this.position.x,2.0);
        squaredLength += Math.pow(this.position.y,2.0);
        squaredLength += Math.pow(weight * this.heading,2.0);
        return Math.sqrt(squaredLength);
    }

    // We think that being off by 1inch is being off by 10 degrees.
    //
    public double length() {
        double tenDegrees = 10*Math.PI/180;
        double weight = 1/tenDegrees;
        return this.length(weight);
    }
    public boolean isWithin(RoPose other, double r) {
        RoPose rel = this.getRelativeRoPoseFor(other);
        return rel.length() <= r;
    }

    public boolean isCloseTo(RoPose other) {
        return this.isWithin(other, 1e-10);
    }

    @SuppressWarnings("DefaultLocale")
    @Override
    public String toString() {
        String tmp = String.format("%5.2f*pi = %7.2f deg", heading/Math.PI, heading*180/Math.PI);
        return "[" + position + ", " + tmp +"]";
    }
    public static void main(String[] args) {
        RoPose trivial = getTrivial();
        RoPose a = new RoPose(2.0,3.0,0);
        RoPose b = new RoPose(0,0, Math.PI/2);
        RoPose c, d;

        c = a.then(b);
        System.out.println(c);
        assert c.isCloseTo(new RoPose(2,3,Math.PI/2));
        assert c.isCloseTo(a.then(b));
        assert c.isCloseTo(trivial.thenForward(2).thenStrafeLeft(3).thenRotate(Math.PI/2));
        assert c.isCloseTo(trivial.thenStrafeLeft(3).thenForward(2).thenRotate(Math.PI/2));
        c = b.then(a);
        System.out.println(c);
        assert c.isCloseTo(new RoPose(-3,2,Math.PI/2));

        d = c.toThePower(2);
        assert d.isCloseTo(c.then(c));

        d = c.inverse();
        assert c.then(d).isCloseTo(trivial);

        // c has a 90 degree angle, so twice should be the inverse
        assert d.isCloseTo(c.inverse());

        // repeat 90 degree angle 4 times to get back to beginning
        assert c.toThePower(4).isCloseTo(trivial);

        // same test for a 60 degree angle
        c = new RoPose(23, 48, Math.PI/3);
        assert c.toThePower(6).isCloseTo(trivial);

        c = new RoPose(23, 48, 1.45);
        assert c.then(c.inverse()).isCloseTo(trivial);

        // go around a rectangle
        c = new RoPose(23, 48, Math.PI/2);
        d = new RoPose(-3, -16, Math.PI/2);
        assert c.then(d).then(c).then(d).isCloseTo(trivial);
    }

    public double getX() {
        return position.x;
    }
    public double getY() {
        return position.y;
    }
}