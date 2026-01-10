package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
public class MergeLocalizer extends Localizer {
    private String color;

    public MergeLocalizer (HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);
        this.color = color;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // these offsets refer to the center of the turret
        pinpoint.setOffsets(74, -80, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;
    private Pose2d lastPinpointPose = null, lastPinpointMergePose = null;
    public static boolean constantCorrection = false;
    public static boolean usePinpoint = true;

    // Limelight
    private LLResult result = null;
    private boolean limelightToggle = false;
    private double lastStaleness = 100.0;

    public void update() {
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

        Pose2d relDelta = new Pose2d (relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime, relDelta, currentPose);

        // PINPOINT

        if ((usePinpoint && lastPinpointPose != null && currentPose.getDistanceFromPoint(lastPinpointPose) >= 24.0) || constantCorrection) {
            Log.i("Localization Test", "pinpoint in use");
            pinpoint.update();

            Pose2d globalPinpointDelta = new Pose2d (
                    pinpoint.getPosX() - lastPinpointPose.x,
                    pinpoint.getPosY() - lastPinpointPose.y,
                    pinpoint.getHeading() - lastPinpointPose.heading
            );

            Pose2d relPinpointDelta = new Pose2d (
                    Math.cos(lastPinpointPose.heading) * globalPinpointDelta.x + Math.sin(lastPinpointPose.heading) * globalPinpointDelta.y,
                    -Math.sin(lastPinpointPose.heading) * globalPinpointDelta.x + Math.cos(lastPinpointPose.heading) * globalPinpointDelta.y,
                    globalPinpointDelta.heading
            );

            Pose2d globalPinpointEstimate = new Pose2d (
                    lastPinpointMergePose.x + Math.cos(lastPinpointMergePose.heading) * relPinpointDelta.x - Math.sin(lastPinpointMergePose.heading) * relPinpointDelta.y,
                    lastPinpointMergePose.y + Math.sin(lastPinpointMergePose.heading) * relPinpointDelta.x + Math.cos(lastPinpointMergePose.heading) * relPinpointDelta.y,
                    lastPinpointMergePose.heading + relPinpointDelta.heading
            );

            lastPinpointPose = new Pose2d (pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading());
            currentPose = globalPinpointEstimate.clone();
            lastPinpointMergePose = globalPinpointEstimate.clone();
        }

        if (lastPinpointPose != null) {
            Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
            DashboardUtil.drawRobot(fieldOverlay, lastPinpointPose, this.expectedColor);
        }

        // LIMELIGHT

        if (limelightToggle) {
            drivetrain.vision.update();
            result = drivetrain.vision.getResult();

            if (result != null && result.isValid()) {
                double D = (Globals.tagHeight - Vision.cameraHeight) / Math.tan(Math.toRadians(0.97 - 0.729 * result.getTx() + 9.37 * 0.001 * result.getTx() * result.getTx()));
                double thetaLime = AngleUtil.clipAngle(ROBOT_POSITION.heading - Math.toRadians(2.88 + 0.249 * result.getTy() + 0.0325 * result.getTy() * result.getTy()));
                Pose2d tag = (result.getFiducialResults().get(0).getFiducialId() == 24) ? Globals.redTag.clone() : Globals.blueTag.clone();

                Pose2d estimatedLLPose = new Pose2d(
                        tag.x - D * Math.cos(thetaLime),
                        tag.y - D * Math.sin(thetaLime),
                        Math.atan2(tag.y - D * Math.sin(thetaLime), tag.x - D * Math.cos(thetaLime))
                );

                Pose2d globalLLEstimate = new Pose2d(
                        estimatedLLPose.x - 6.4 * Math.cos(estimatedLLPose.heading) + 5.5 * Math.sin(estimatedLLPose.heading),
                        estimatedLLPose.y - 6.4 * Math.sin(estimatedLLPose.heading) + 5.5 * Math.cos(estimatedLLPose.heading),
                        estimatedLLPose.heading
                );
            }
        }


        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0, currentPose.clone());

        updateVelocity();
        updateExpected();
        updateField();
    }

    public void setPoseEstimate(Pose2d pose) {
        super.setPoseEstimate(pose);
        pinpoint.setPosition(new Pose2D (DistanceUnit.INCH, pose.x, pose.y, AngleUnit.RADIANS, pose.heading));
        lastPinpointPose = pose.clone();
        lastPinpointMergePose = pose.clone();
    }

    public void setLimelightToggle (boolean toggle) { limelightToggle = toggle; }

    public double getInstantaneousAngularVel () { return poseHistory.size() >= 2 ? (poseHistory.get(0).heading - poseHistory.get(1).heading) / (nanoTimes.get(0) - nanoTimes.get(1)) : 0; }

    public void updateField() {
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" x", x);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" y", y);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" heading (deg)", Math.toDegrees(heading));
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" distance", distanceTraveled);
        TelemetryUtil.packet.put("Pinpoint x", pinpoint.getPosX());
        TelemetryUtil.packet.put("Pinpoint y", pinpoint.getPosY());
        TelemetryUtil.packet.put("Pinpoint heading", pinpoint.getHeading());

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate(), this.color);
    }
}
