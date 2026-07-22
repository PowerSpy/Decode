package org.firstinspires.ftc.teamcode.utils;

public class Vector2 {
    public double x;
    public double y;
    private double magcache;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(Vector2 other)
    {
        this.x = other.x;
        this.y = other.y;
    }

    public Vector2() {
        x = y = 0;
    }

    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    public double mag() {
        if (magcache == 0 && (x != 0 || y != 0)) {
            magcache = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }

        return magcache;
    }

    public void mul(double a) {
        x *= a;
        y *= a;
        magcache *= a;
    }


    public static double dot(Vector2 a,Vector2 b) {
        return (a.x*b.x + a.y*b.y);
    }

    public void norm() {
        double mag = mag();

        if (mag == 0) {
            return;
        }

        x /= mag;
        y /= mag;
        magcache = 1;
    }

    public void add(Vector2 a) {
        x += a.x;
        y += a.y;
        magcache = 0;
    }

    public void subtract(Vector2 a) {
        x -= a.x;
        y -= a.y;
        magcache = 0;
    }

    public double theta() {
        return Math.atan2(y, x);
    }

    public String toString() {
        return String.format("(%f, %f)", x, y);
    }

    public static Vector2 rotate(Vector2 vector, double angle) {
        double x = vector.x;
        double y = vector.y;
        x = x*Math.cos(angle) + y*Math.sin(angle);
        y = x*-Math.sin(angle) + y*Math.cos(angle);
        return new Vector2(x, y);
    }
    public void rotateAround(double angle, double x, double y) {
        this.x -= x;
        this.y -= y;
        rotate(angle); //idk if is scuffed or not lmao -Kyle
        this.x +=x;
        this.y +=y;
    }
    public static Vector2 rotateAround(Vector2 vector, double angle, double x, double y) {
        double vx = vector.x-x;
        double vy = vector.y-y;
        Vector2 temp = Vector2.rotate(new Vector2(vx,vy), angle);
        return new Vector2(temp.x+x, temp.y+y);
    }

    public static Vector2 subtract(Vector2 a, Vector2 b) {
        return new Vector2(a.x - b.x, a.y - b.y);
    }

    public static double distance(Vector2 v0, Vector2 v1) {
        return Math.sqrt(Math.pow(v0.x - v1.x, 2) + Math.pow(v0.y - v1.y, 2));
    }

    public void rotate(double rad) {
        double nx = x * Math.cos(rad) - y * Math.sin(rad);
        double ny = x * Math.sin(rad) + y * Math.cos(rad);

        x = nx;
        y = ny;
    }

    public static Vector2 staticrotate(Vector2 v, double rad) {
        return new Vector2(
            v.x * Math.cos(rad) - v.y * Math.sin(rad),
            v.x * Math.sin(rad) + v.y * Math.cos(rad)
        );
    }
}