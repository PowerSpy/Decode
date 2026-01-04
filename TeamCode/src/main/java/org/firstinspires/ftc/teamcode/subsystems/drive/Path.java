package org.firstinspires.ftc.teamcode.subsystems.drive;

import android.util.Log;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;

class PathSegment {
    public final Spline spline;
    public final boolean reversed;
    public boolean decel;
    public final double power;

    PathSegment (Spline s, boolean r, boolean d, double p){
        spline = s;
        reversed = r;
        decel = d;
        power = p;
    }
}

class PathData {
    Vector2 vel, accel;
    double r, power;
    boolean reversed;
    int index;

    public PathData (Vector2 vel, Vector2 accel, double r, double power, boolean reversed, int index) {
        this.vel = vel;
        this.accel = accel;
        this.r = r;
        this.power = power;
        this.reversed = reversed;
        this.index = index;
    }
}

class InstantaneousData  {
    Vector2 vel;
    double mag;

    public InstantaneousData (Vector2 v, double m) {
        vel = v;
        mag = m;
    }
}

public class Path {
    ArrayList <PathSegment> pathSegments;
    ArrayList <RepulsionPoint> repel;
    boolean reversed, decel, completed;
    double power;
    int lastReachedIndex = 0;
    Pose2d lastPose;

    public Path (Pose2d p, ArrayList<RepulsionPoint> repel) {
        this.repel = repel;
        pathSegments = new ArrayList <>();
        reversed = false;
        decel = false;
        completed = false;
        power = 1.0;
        lastPose = p.clone();
    }

    public Path addPoint(Pose2d p) {
        if(reversed) {
            p.heading += Math.PI;
        }

        Spline s = new Spline (lastPose, p);
        pathSegments.add(new PathSegment(s, reversed, decel, 1.0));
        lastPose = p.clone();

        return this;
    }

    public Path addRepel(RepulsionPoint point){
        this.repel.add(point);
        return this;
    }

    public Path setReversed (boolean rev) {
        if (rev != reversed) {
            lastPose.heading += Math.PI;
        }
        reversed = rev;

        return this;
    }

    public Path setDecel (boolean dec) {
        if (dec != decel && pathSegments.size() != 0) {
            pathSegments.get(pathSegments.size() - 1).decel = dec;
        }
        decel = dec;

        return this;
    }

    public static double k_p = 0.1666;

    public InstantaneousData getVelocity (Spline s, double tau, Vector2 robot) {
        Vector2 v_t = s.getVel(tau);
        v_t.norm();
        Log.i("Path v_t", v_t + "");

        Vector2 v_p = new Vector2(s.getPos(tau).x - robot.x, s.getPos(tau).y - robot.y);
        v_p.mul(k_p);
        Log.i("Path v_p", v_p + "");

        Vector2 v_rep = new Vector2(0, 0);
        for(RepulsionPoint rep : repel) {
            Vector2 trep = new Vector2(robot.x - rep.x, robot.y - rep.y);
            double scale = Math.pow(Math.E, 1 - Math.pow(trep.mag()/rep.weight,2));
            trep.norm();
            trep.mul(scale);

            v_rep.add(trep);
        }
        Log.i("Path v_rep", v_rep + "");

        return new InstantaneousData(Vector2.add(v_t, Vector2.add(v_p, v_rep)), v_t.mag());
    }

    public PathData update (Pose2d robot) {
        PathSegment curr;
        int index = lastReachedIndex;

        while(index < pathSegments.size() && pathSegments.get(index).spline.getT(robot) == 1.0) {
            index++;
        }
        Log.i("Path chosen index", index + "");
        lastReachedIndex = index;

        completed = index == pathSegments.size();
        if (completed) {
            return null;
        }

        curr = pathSegments.get(index);

        InstantaneousData current = getVelocity(
                curr.spline,
                curr.spline.getT(robot),
                new Vector2(robot.x, robot.y));

        InstantaneousData next = getVelocity(
                curr.spline,
                curr.spline.getT(new Pose2d(robot.x + current.vel.x * 0.001, robot.y + current.vel.y * 0.001)),
                new Vector2(robot.x + current.vel.x * 0.001, robot.y + current.vel.y * 0.001));

        Vector2 accel = new Vector2((next.vel.x * next.mag - current.vel.x * current.mag) / 0.001, (next.vel.y * next.mag - current.vel.y * current.mag));

        return new PathData(current.vel, accel, (current.vel.mag() * current.vel.mag() * current.vel.mag()) / (current.vel.x * accel.y - current.vel.y * accel.x), curr.power, curr.reversed, index);
    }
}
