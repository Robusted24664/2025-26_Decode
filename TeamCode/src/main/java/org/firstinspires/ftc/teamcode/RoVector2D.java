package org.firstinspires.ftc.teamcode;

public class RoVector2D {
    public final double x;
    public final double y;


    // initialize with specific x and y values
    public RoVector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }


    // copy from a different vector
    public RoVector2D(RoVector2D original) {
        this(original.x, original.y);
    }


    // length of the vector
    public double length() {
        return Math.sqrt(x*x + y*y);
    }


    public RoVector2D neg() {
        return new RoVector2D(-x,-y);
    }


    public RoVector2D add(RoVector2D other) {
        return new RoVector2D(x+other.x, y+other.y);
    }


    public RoVector2D sub(RoVector2D other) {
        return new RoVector2D(x-other.x, y-other.y);
    }


    public double dot(RoVector2D other) {
        return x*other.x + y*other.y;
    }


    public RoVector2D rotateByDegrees(double deg) {
        return rotate(deg*Math.PI/180.0);
    }


    public RoVector2D rotate(double theta) {
        final double cosTheta = Math.cos(theta);
        final double sinTheta = Math.sin(theta);
        return new RoVector2D(x*cosTheta-y*sinTheta, x*sinTheta+y*cosTheta);
    }


    public boolean isWithin(RoVector2D other, double r) {
        return this.sub(other).length() <= r;
    }


    public boolean isCloseTo(RoVector2D other) {
        return this.isWithin(other, 1e-10);
    }




    @SuppressWarnings("DefaultLocale")
    @Override
    public String toString() {
        //return "(" + x+" " + y + ")";
        return String.format("(%7.2f, %7.2f)",x,y);
    }


    // testing
    public static void main(String[] args) {
         //assert false : "asserts are working";


        RoVector2D v = new RoVector2D(1,3);
        RoVector2D w = new RoVector2D(3,10);
        RoVector2D zero = new RoVector2D(0,0);
        System.out.println("v: " + v);
        System.out.println("w: " + w);
        System.out.println("-w: " + w.neg());
        assert w.neg().isCloseTo(zero.sub(w));


        System.out.println("v+w: " + v.add(w));
        assert v.add(w).isCloseTo(w.add(v));


        System.out.println("v+(-w): " + v.add(w.neg()));
        System.out.println("v-w: " + v.sub(w));
        assert v.add(w.neg()).isCloseTo(v.sub(w));
        assert v.add(w).sub(v).sub(w).isCloseTo(zero);


        System.out.println("[dot product] v.w: " + v.dot(w));
        System.out.println("rotate v by 90 degrees: " + v.rotateByDegrees(90));
        assert Math.abs(v.dot(v.rotateByDegrees(90))) <= 1e-10;


        int i;
        double originalLength = v.length();
        for (i=0; i<=90; i+=10) {
            RoVector2D tmp = v.rotateByDegrees(i);
            double tmpLength = tmp.length();
            System.out.println("rotate v by " + i + " degrees: " + tmp + "  rotated length = " + tmpLength);
            assert Math.abs(originalLength-tmpLength) < 1e-10 : "rotated vectors should have the same length";
        }
    }


}
