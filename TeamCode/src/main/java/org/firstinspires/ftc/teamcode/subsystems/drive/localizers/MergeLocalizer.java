package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class MergeLocalizer extends Localizer{
    public MergeLocalizer (HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(72, -160, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    // Pinpoint
    private Thread ppUpdate = new Thread();
    private GoBildaPinpointDriver pinpoint;
    private boolean ppUpdateRequest = false;
    private long ppLastUpdateTime;
    private Pose2d ppLastPose;

    // Limelight
    private Thread llUpdate = new Thread();
    private LLResult res = null;
    private final Pose2d redTag = new Pose2d(-58.3414795, 55.6424675);
    private final Pose2d blueTag = new Pose2d(-58.3414795, -55.6424675);
    private final double tagHeight = 29.5;

    public void update(){
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime - lastTime)/1.0E9;
        lastTime = currentTime;

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

        Pose2d relOdoDelta = new Pose2d (relDeltaX,relDeltaY,deltaHeading);

        // PINPOINT

        if(ppUpdateRequest) {
            ppLastPose = new Pose2d(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getHeading(AngleUnit.RADIANS));
            pinpoint.update();
            ppLastUpdateTime = System.currentTimeMillis();
        }

        // LIMELIGHT

        res = drivetrain.vision.getResult();

        double D = (tagHeight - drivetrain.vision.cameraHeight) / Math.tan(drivetrain.vision.cameraAngle + res.getTy());
        Pose2d llEstimatedPose = new Pose2d(
                // todo: add turret heading to ROBOT_POSITION.getHeading() such that the heading is the turret in global coordinates
                (Globals.isRed ? redTag.x : blueTag.x) - D * Math.cos(ROBOT_POSITION.getHeading() - res.getTx()),
                (Globals.isRed ? redTag.y : blueTag.y) - D * Math.sin(ROBOT_POSITION.getHeading() - res.getTx()),
                ROBOT_POSITION.getHeading() - res.getTx()
        );

        // MERGE


    }
}
