package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.vision.Vision;

class Pinpoint implements Runnable {
    private final GoBildaPinpointDriver pp;
    public Pose2d currPose, lastPose;
    public double lastUpdateTime;

    public Pinpoint (GoBildaPinpointDriver pp) {
        this.pp = pp;
        currPose = new Pose2d(pp.getPosX(), pp.getPosY(), pp.getHeading());
        lastPose = currPose.clone();
        lastUpdateTime = System.currentTimeMillis();
    }

    @Override
    public void run() {
        pp.update();
        lastPose = currPose.clone();
        currPose = new Pose2d(pp.getPosX(), pp.getPosY(), pp.getHeading());
        lastUpdateTime = System.currentTimeMillis();
    }

    public Pose2d getRelPP () { return new Pose2d (currPose.x - lastPose.x, currPose.y - lastPose.y, currPose.heading - lastPose.heading); }

    public double getStaleness() { return System.currentTimeMillis() - lastUpdateTime; }
}

class Limelight implements Runnable {
    private final Vision vision;
    public LLResult result = null;
    public double lastStaleness = 0.0;

    public Limelight (Vision vision) {
        this.vision = vision;
    }

    @Override
    public void run() {
        vision.update();
        result = vision.getResult();
    }

    public boolean isValid() { return result != null && result.isValid(); }
}

public class MergeLocalizer extends Localizer{
    public MergeLocalizer (HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(72, -160, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pp = new Pinpoint (pinpoint);
        ll = new Limelight (drivetrain.vision);
    }

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;
    private Pinpoint pp;
    private Pose2d lastUsedPose = null;

    // Limelight
    private Limelight ll;
    private boolean limelightToggle = false;
    private final Pose2d redTag = new Pose2d(-58.3414795, 55.6424675);
    private final Pose2d blueTag = new Pose2d(-58.3414795, -55.6424675);
    private final double tagHeight = 29.5;

    public void update(){
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime - lastTime)/1.0E9;
        lastTime = currentTime;

        pp.run();
        if (limelightToggle) { ll.run();}

        // 3 WHEEL ODOMETRY

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

        // currently this only represents the relOdoDelta but everything will get merged into this so its fine
        Pose2d relDelta = new Pose2d (relDeltaX,relDeltaY,deltaHeading);

        // PINPOINT

        if (lastUsedPose == null || (lastUsedPose.x != pp.currPose.x || lastUsedPose.y != pp.currPose.y || lastUsedPose.heading != pp.currPose.heading)){
            Pose2d relPP = pp.getRelPP();
            double staleness = pp.getStaleness();

            double posScalar = 1 - 2 / Math.PI * Math.atan (0.75 * staleness);
            // extreme weighting for heading because flywheel vibration probably mess with pinpoint imu too much
            double headingScalar = 1 - 2 / Math.PI * Math.atan (3 * staleness);

            relDelta.x = relPP.x * posScalar + relDelta.x * (1 - posScalar);
            relDelta.y = relPP.y * posScalar + relDelta.y * (1 - posScalar);
            relDelta.heading = relPP.heading * headingScalar + relDelta.heading * (1 - headingScalar);
        }


        // LIMELIGHT

        if (limelightToggle && ll.isValid() && ll.lastStaleness < ll.result.getStaleness()){
            double D = (tagHeight - drivetrain.vision.cameraHeight) / Math.tan(drivetrain.vision.cameraAngle + ll.result.getTy());

            // TODO: the ROBOT_POSITION.heading is a placeholder for turret global heading
            Pose2d relLL = new Pose2d (
                    (Globals.isRed ? redTag.x : blueTag.x) - D * Math.cos(ROBOT_POSITION.heading - ll.result.getTx()) - currentPose.x,
                    (Globals.isRed ? redTag.x : blueTag.x) - D * Math.cos(ROBOT_POSITION.heading - ll.result.getTy()) - currentPose.y,
                    ROBOT_POSITION.getHeading()  - ll.result.getTx()
            );

            // extreme weighing again because that camera was vibrating jesus
            double posScalar = 1 - 2 / Math.PI * Math.atan (3 * ll.result.getStaleness());
            double headingScalar = 1 - 2 / Math.PI * Math.atan (3 * ll.result.getStaleness());

            relDelta.x = relLL.x * posScalar + relDelta.x * (1 - posScalar);
            relDelta.y = relLL.y * posScalar + relDelta.y * (1 - posScalar);
            relDelta.heading = relLL.heading * headingScalar + relDelta.x * (1 - headingScalar);
        }

        // COMPUTE
        constAccelMath.calculate(loopTime, relDelta, currentPose);
        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public void setPoseEstimate(Pose2d pose) {
        super.setPoseEstimate(pose);
        pinpoint.setPosition(new Pose2D (DistanceUnit.INCH, pose.x, pose.y, AngleUnit.RADIANS, pose.heading));
    }

    public void setLimelightToggle (boolean toggle) { limelightToggle = toggle; }
}
