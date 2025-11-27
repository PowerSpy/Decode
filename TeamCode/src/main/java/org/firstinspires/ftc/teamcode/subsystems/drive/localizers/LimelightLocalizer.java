package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class LimelightLocalizer extends Localizer{
    public LimelightLocalizer (Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);
    }

    private LLResult res = null;
    private final Pose2d redTag = new Pose2d(-58, 58);
    private final Pose2d blueTag = new Pose2d(-58, -58);
    private final double tagHeight = 29.5, staleness = 0.0;

    @Override
    public void update(){
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime - lastTime)/1.0E9;
        lastTime = currentTime;

        if(drivetrain.vision.isDetected()) {
            res = drivetrain.vision.getResult();

            double D = (tagHeight - drivetrain.vision.cameraHeight) / Math.tan(drivetrain.vision.cameraAngle + res.getTy());
            // TODO: sensors.getHeading() is a placeholder, replace once turret PID is written
            x = (Globals.isRed ? redTag.x : blueTag.x) - D * Math.cos(sensors.getHeading() - res.getTx());
            y = (Globals.isRed ? redTag.y : blueTag.y) - D * Math.sin(sensors.getHeading() - res.getTx());
            heading = sensors.getHeading() - res.getTx();

            relHistory.add(0, new Pose2d(x - currentPose.x, y - currentPose.y, heading - currentPose.heading));
            nanoTimes.add(0, currentTime);
            poseHistory.add(0, currentPose.clone());

            updateVelocity();
            updateExpected();
            updateField();
        }
    }
}
