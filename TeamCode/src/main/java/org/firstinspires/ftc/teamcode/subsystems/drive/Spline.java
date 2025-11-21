package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

public class Spline {
    public final static double MAX_RADIUS = 144.0;

    double[] xCoeff = new double [4];
    double[] yCoeff = new double [4];

    public double t = 0;

    public Spline(Pose2d start, Pose2d end) {
        double arbitraryVelo = 1.5 * Math.sqrt(Math.pow(end.x - start.x, 2) + Math.pow(end.y - start.y, 2));

        xCoeff[0] = start.x;
        xCoeff[1] = arbitraryVelo * Math.cos(start.heading);
        xCoeff[2] = 3*end.x - arbitraryVelo * Math.cos(end.heading) - 2 * xCoeff[1] - 3 * xCoeff[0];
        xCoeff[3] = end.x - xCoeff[0] - xCoeff[1] - xCoeff[2];

        yCoeff[0] = start.y;
        yCoeff[1] = arbitraryVelo * Math.sin(start.heading);
        yCoeff[2] = 3 * end.y - arbitraryVelo * Math.sin(end.heading) - 2 * yCoeff[1] - 3 * yCoeff[0];
        yCoeff[3] = end.y - yCoeff[0] - yCoeff[1] - yCoeff[2];
    }

    public double getT (Pose2d pos) {
        double last_t = t;
        double shift = 1.0;

        // Newton's method to determine the closest point
        // Min-Max Clip of -0.2 to 0.2 to avoid drastic shifts domain of t is 0 to 1
        for(int i = 0; i < 10; i++) {
            Vector2 p = getPos (last_t);
            Vector2 v = getVel (last_t);
            Vector2 a = getAccel (last_t);

            shift = (v.x * (p.x - pos.x) + v.y * (p.y - pos.y)) / (v.x * v.x + a.x * (p.x - pos.x) + v.y * v.y + a.y * (p.y - pos.y));
            t = last_t - Math.max(Math.min(shift, 0.2), -0.2);
            t = Math.max(Math.min(t,  1), 0);
            last_t = t;

            // if there's reduced impact of shifts no need to keep on iterating
            if(Math.abs(shift) < 0.001) {
                break;
            }
        }

        return t;
    }

    public double getR (double t) {
        Vector2 v = getVel (t);
        Vector2 a = getAccel (t);

        if(a.y * v.x - a.x * v.y != 0) {
            double r = Math.pow(v.x*v.x + v.y*v.y, 1.5)/(a.y*v.x - a.x*v.y); // radius of curvature in planar motion formula
            return Math.max(Math.min(r, MAX_RADIUS), -MAX_RADIUS);
        }

        return MAX_RADIUS;
    }

    public Vector2 getPos (double t) {
        return new Vector2 (
                xCoeff[0] + xCoeff[1] * t + xCoeff[2] * t * t + xCoeff[3] * t * t * t,
                yCoeff[0] + yCoeff[1] * t + yCoeff[2] * t * t + yCoeff[3] * t * t * t
        );
    }

    public Vector2 getVel (double t) {
        return new Vector2 (
                xCoeff[1] + 2 * xCoeff[2] * t + 3 * xCoeff[3] * t * t,
                yCoeff[1] + 2 * yCoeff[2] * t + 3 * yCoeff[3] * t * t
        );
    }

    public Vector2 getAccel (double t) {
        return new Vector2 (
                2 * xCoeff[2] + 6 * xCoeff[3] * t,
                2 * yCoeff[2] + 6 * yCoeff[3] * t
        );
    }
}