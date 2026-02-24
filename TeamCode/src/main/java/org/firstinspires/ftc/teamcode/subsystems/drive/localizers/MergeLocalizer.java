package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Lerp;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Config
public class MergeLocalizer extends Localizer {
    private String color;

    public MergeLocalizer (HardwareMap hardwareMap, Sensors sensors, Drivetrain drivetrain, String color, String expectedColor){
        super(sensors, drivetrain, color, expectedColor);
        this.color = color;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // these offsets refer to the center of the turret
        pinpoint.setOffsets(3.391, 0.582, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.update();
        Pose2d p = new Pose2d (pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading());
        TelemetryUtil.packet.put("Pinpoint start", String.format(Locale.US, "%.3f %.3f %.3f", p.x, p.y, p.heading));
        super.setPoseEstimate(p);
        lastPinpointMergePose = currentPose.clone();
        lastPinpointPose = p;
    }

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;
    private Pose2d lastPinpointPose = null, lastPinpointMergePose = null;
    public static boolean constantCorrection = false;
    public static boolean usePinpoint = true;
    public static double pinpointPollDist = 6;

    // Camera


    private Pose2d estimatedCameraPose = null;
    private Pose2d lastCameraPose = null;
    private Pose2d poseWithoutCamera = new Pose2d(0,0,0);
    public static boolean useCamera = false;
    public int numberOfTimesRelocalizedWithCamera = 0;

    public static double a = 0.4;
    public static double b = 0.4;
    public static double c = 0.4;

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
        constAccelMath.calculate(loopTime, relDelta, poseWithoutCamera);




        // PINPOINT

        if ((usePinpoint && lastPinpointPose != null && currentPose.getDistanceFromPoint(lastPinpointPose) >= pinpointPollDist) || constantCorrection) {
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
            currentPose = new Pose2d (pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading());
            poseWithoutCamera = globalPinpointEstimate.clone();
            lastPinpointMergePose = globalPinpointEstimate.clone();
        }

        if (lastPinpointPose != null) {
            Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
            DashboardUtil.drawRobot(fieldOverlay, new Pose2d(pinpoint.getPosX(), pinpoint.getPosY(), pinpoint.getHeading()), this.expectedColor); // pink / magenta
        }

        // Camera

        if(currentPose.heading % (Math.PI * 2) > Math.PI / 2 && currentPose.heading % (Math.PI * 2) < Math.PI * 3 / 2) {
            drivetrain.vision.visionPortal.setProcessorEnabled(drivetrain.vision.aprilTagProcessor, true);
        } else {
            drivetrain.vision.visionPortal.setProcessorEnabled(drivetrain.vision.aprilTagProcessor, false);
        }

        if (useCamera && drivetrain.vision != null && drivetrain.vision.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && drivetrain.vision.visionPortal.getProcessorEnabled(drivetrain.vision.aprilTagProcessor)) {
            estimatedCameraPose = drivetrain.vision.update();
            if(estimatedCameraPose != null && lastCameraPose != null) {
                //10 ms delay maximum
                if(drivetrain.vision.timeSinceLastFrame < 1e-7) {
                    numberOfTimesRelocalizedWithCamera++;

                    //low pass filter

                    estimatedCameraPose.x = Lerp.lerp(estimatedCameraPose.x, lastCameraPose.x, a);
                    estimatedCameraPose.y = Lerp.lerp(estimatedCameraPose.y, lastCameraPose.y, b);
                    estimatedCameraPose.heading = Lerp.lerpAngle(estimatedCameraPose.heading, lastCameraPose.heading, c);

                    //merging camera with odometry/pinpoint

                    currentPose.x = Lerp.lerp(currentPose.x, estimatedCameraPose.x, 0.5);
                    currentPose.y = Lerp.lerp(currentPose.y, estimatedCameraPose.y, 0.5);
                    currentPose.heading = Lerp.lerpAngle(currentPose.heading, estimatedCameraPose.heading, 0.5);

                    lastCameraPose = estimatedCameraPose.clone();
                }
            } else if (estimatedCameraPose != null){
                if(drivetrain.vision.timeSinceLastFrame < 1e-7) {
                    numberOfTimesRelocalizedWithCamera++;

                    currentPose.x = Lerp.lerp(currentPose.x, estimatedCameraPose.x, 0.5);
                    currentPose.y = Lerp.lerp(currentPose.y, estimatedCameraPose.y, 0.5);
                    currentPose.heading = Lerp.lerpAngle(currentPose.heading, estimatedCameraPose.heading, 0.5);

                    lastCameraPose = estimatedCameraPose.clone();
                }
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
        lastPinpointPose = lastPinpointMergePose = pose.clone();
    }

    public double getInstantaneousAngularVel () { return poseHistory.size() >= 2 ? (poseHistory.get(0).heading - poseHistory.get(1).heading) / (nanoTimes.get(0) - nanoTimes.get(1)) : 0; }

    public void updateField() {
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" x", x);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" y", y);
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" heading (deg)", Math.toDegrees(heading));
        TelemetryUtil.packet.put(this.getClass().getSimpleName()+" distance", distanceTraveled);
        TelemetryUtil.packet.put("Pinpoint x", pinpoint.getPosX());
        TelemetryUtil.packet.put("Pinpoint y", pinpoint.getPosY());
        TelemetryUtil.packet.put("Pinpoint heading", pinpoint.getHeading());
        TelemetryUtil.packet.put("Number of times Relocalized with Camera", numberOfTimesRelocalizedWithCamera);

        if (estimatedCameraPose != null) {
            TelemetryUtil.packet.put("Camera x", estimatedCameraPose.x);
            TelemetryUtil.packet.put("Camera y", estimatedCameraPose.y);
            TelemetryUtil.packet.put("Camera heading", estimatedCameraPose.heading);

        }

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, getPoseEstimate(), this.color); // blue
        DashboardUtil.drawRobot(fieldOverlay, estimatedCameraPose, "#90d5ff");
    }
}
