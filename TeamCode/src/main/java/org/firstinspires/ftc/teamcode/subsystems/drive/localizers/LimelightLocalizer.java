package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

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
    private final Pose2d redTag = new Pose2d(-58.3414795, 55.6424675);
    private final Pose2d blueTag = new Pose2d(-58.3414795, -55.6424675);
    private final double tagHeight = 29.5;

    @Override
    public void update(){
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime - lastTime)/1.0E9;

        if(drivetrain.vision.isDetected()) {
            res = drivetrain.vision.getResult();
            lastTime = currentTime;

            double D = (tagHeight - drivetrain.vision.cameraHeight) / Math.tan(drivetrain.vision.cameraAngle + res.getTy());
            // TODO: sensors.getHeading() is a placeholder, replace once turret PID is written
            x = (Globals.isRed ? redTag.x : blueTag.x) - D * Math.cos(ROBOT_POSITION.getHeading() - res.getTx());
            y = (Globals.isRed ? redTag.y : blueTag.y) - D * Math.sin(ROBOT_POSITION.getHeading() - res.getTx());
            heading = ROBOT_POSITION.getHeading() - res.getTx();
        }

        Pose2d relDelta = new Pose2d (x - currentPose.x, y - currentPose.y, heading - currentPose.heading);
        constAccelMath.calculate(loopTime, relDelta, currentPose);

        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0, new Pose2d(x - currentPose.x, y - currentPose.y, heading - currentPose.heading));
        nanoTimes.add(0, currentTime);
        poseHistory.add(0, currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public double getStaleness() {return res.getStaleness();}
}
