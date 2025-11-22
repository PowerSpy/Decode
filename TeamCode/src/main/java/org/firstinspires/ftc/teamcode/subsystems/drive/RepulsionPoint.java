package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class RepulsionPoint extends Pose2d {
    public final double weight;

    public RepulsionPoint(double x, double y, double w) {
        super (x, y);
        weight = w;
    }
}
