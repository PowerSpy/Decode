package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class MergeLocalizer extends Localizer{
    public MergeLocalizer (Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);
    }

    private LLResult res = null;
    private final Pose2d redTag = new Pose2d(-58.3414795, 55.6424675);
    private final Pose2d blueTag = new Pose2d(-58.3414795, -55.6424675);
    private final double tagHeight = 29.5;

    public void update(){
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime - lastTime)/1.0E9;
        lastTime = currentTime;

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        //This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        //This gives us deltaY because the back minus theta*R is the amount moved to the left minus the amount of movement in the back encoder due to change in heading
        relDeltaY = deltaBack - deltaHeading*backX;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        Pose2d relOdoDelta = new Pose2d(relDeltaX,relDeltaY,deltaHeading);

        res = drivetrain.vision.getResult();

        double D = (tagHeight - drivetrain.vision.cameraHeight) / Math.tan(drivetrain.vision.cameraAngle + res.getTy());
        // TODO: ROBOT_POSITION.getHeading() is a placeholder, replace once turret PID is written
        Pose2d relLimelightDelta = new Pose2d(
                (Globals.isRed ? redTag.x : blueTag.x) - D * Math.cos(ROBOT_POSITION.getHeading() - res.getTx()) - currentPose.x,
                (Globals.isRed ? redTag.y : blueTag.y) - D * Math.sin(ROBOT_POSITION.getHeading() - res.getTx()) - currentPose.y,
                (ROBOT_POSITION.getHeading() - res.getTx()) - currentPose.heading
        );

        // remove weight of limelight when scanning for obelisk as the values will be bogus
        double scalar = 1 - 2 / Math.PI * Math.atan(3 / 10.0 * (drivetrain.vision.obelisk ? 100000 : res.getStaleness()));
        Pose2d relMergedDelta = new Pose2d (relOdoDelta.x * (1 - scalar) + relLimelightDelta.x * scalar, relOdoDelta.y * (1 - scalar) + relLimelightDelta.y * scalar, relOdoDelta.heading * (1 - scalar) + relLimelightDelta.heading * scalar);
        constAccelMath.calculate(loopTime, relMergedDelta, currentPose);

        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relMergedDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }
}
