package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;

class PathSegment {
    public final Spline spline;
    public final boolean reversed, decel;
    public final double power;

    public PathSegment (Spline s, boolean rev, boolean dec, double pow) {
        spline = s;
        reversed = rev;
        decel = dec;
        power = pow;
    }
}

class PathData {
    public Vector2 velocity, acceleration;
    public double radius, power;
    public boolean reversed, decel;
    public int index;

    public PathData (Vector2 vel, Vector2 accel, double r, double pow, boolean rev, boolean dec, int ind) {
        velocity = new Vector2(vel.x, vel.y);
        acceleration = new Vector2(accel.x, accel.y);
        radius = r;
        power = pow;
        reversed = rev;
        decel = dec;
        index = ind;
    }
}

class GuidingVectors {
    public Vector2 tangential, pull, repulsion;

    public GuidingVectors(Vector2 v_t, Vector2 v_p, Vector2 v_r) {
        tangential = new Vector2(v_t.x, v_t.y);
        pull = new Vector2(v_p.x, v_p.y);
        pull.mul(tangential.mag());
        repulsion = new Vector2(v_r.x, v_r.y);
    }

    public Vector2 theoreticalVel() { return Vector2.add(tangential, Vector2.add(pull, repulsion)); }

    public Vector2 getAccel(GuidingVectors predict) { return new Vector2((predict.theoreticalVel().x - this.theoreticalVel().x) / 0.001, (predict.theoreticalVel().y - this.theoreticalVel().y) / 0.001); }

    public double getRadius(GuidingVectors predict) { return (this.theoreticalVel().mag() * this.theoreticalVel().mag() * this.theoreticalVel().mag()) / (this.theoreticalVel().x * this.getAccel(predict).y - this.theoreticalVel().y * this.getAccel(predict).x); }
}

public class Path {
    ArrayList <PathSegment> segments;
    ArrayList <RepulsionPoint> repulsion;
    private Pose2d lastPose;

    public Path (Pose2d p, ArrayList <RepulsionPoint> rp) {
        segments = new ArrayList<>();
        repulsion = rp;
        lastPose = p.clone();
    }

    public Path (Pose2d p) {
        segments = new ArrayList<>();
        repulsion = new ArrayList<RepulsionPoint>();
        lastPose = p.clone();
    }


    public Path addPoint(Pose2d p, boolean rev, boolean dec) {
        if (rev) p.heading += Math.PI;
        segments.add(new PathSegment (new Spline (lastPose, p), rev, dec, 1.0));
        lastPose = p.clone();
        return this;
    }

    public Path addPoint(Pose2d p, boolean rev, boolean dec, double pow) {
        if (rev) p.heading += Math.PI;
        segments.add(new PathSegment (new Spline (lastPose, p), rev, dec, pow));
        lastPose = p.clone();
        return this;
    }

    public Pose2d getSegLast(int index) { return new Pose2d(segments.get(index).spline.getPos(1).x, segments.get(index).spline.getPos(1).y); }

    public Pose2d getLastPose() { return lastPose.clone(); }

    public static double k_p = 0.1666; // 1 / 6

    private GuidingVectors calculate(PathSegment currSegment, Pose2d robot) {
        Spline s = currSegment.spline;
        double tau = s.getT(robot);

        Vector2 v_t = s.getVel(tau);

        Vector2 v_p = new Vector2(s.getPos(tau).x - robot.x, s.getPos(tau).y - robot.y);
        v_p.mul(k_p);

        Vector2 v_r = new Vector2(0, 0);
        for(RepulsionPoint rp : repulsion) {
            v_r.add(rp.getInfluence(robot));
        }


        return new GuidingVectors(v_t, v_p, v_r);
    }

    private int lastIndex = 0;

    public PathData update(Pose2d robot) {
        int index = lastIndex;
        while (index < segments.size() - 1 && segments.get(index).spline.getT(robot) > 0.99) {
            index++;
        }

        if (index == segments.size()) return null;

        lastIndex = index;

        PathSegment currSeg = segments.get(index);
        GuidingVectors currentGVF = calculate(currSeg, robot);
        double t = currSeg.spline.getT(robot);
        double blendThreshold = 0.95;
        Vector2 finalVel = currentGVF.theoreticalVel();

        GuidingVectors current;
        GuidingVectors predict;

        if (index < segments.size() - 1 && t > blendThreshold) {
            PathSegment nextSeg = segments.get(index + 1);
            GuidingVectors nextGVF = calculate(nextSeg, robot);

            double weight = (t - blendThreshold) / (1.0 - blendThreshold);

            finalVel.x = finalVel.x * (1 - weight) + nextGVF.theoreticalVel().x * weight;
            finalVel.y = finalVel.y * (1 - weight) + nextGVF.theoreticalVel().y * weight;

            currSeg = segments.get(index+1);
            currentGVF = calculate(currSeg, robot);
        }

        finalVel = currentGVF.theoreticalVel();
        current = calculate(segments.get(index), robot);
        predict = calculate(segments.get(index), new Pose2d(robot.x + current.theoreticalVel().x * 0.001, robot.y + current.theoreticalVel().y * 0.001));

        return new PathData(
            current.theoreticalVel(),
            current.getAccel(predict),
            current.getRadius(predict),
            segments.get(index).power,
            segments.get(index).reversed,
            segments.get(index).decel,
            index
        );
    }
}

