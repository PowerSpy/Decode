package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public class Vision {

    public ArrayList<AprilTagDetection> detections = null;
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public Vision (HardwareMap hardwareMap) {
        init(hardwareMap);

    }

    public void init(HardwareMap HardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
//                .setDrawAxes(true)
//                .setDrawTagOutline(true)
//                .setDrawCubeProjection(true)
                .setNumThreads(3)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644) // 640x480: 549.651, 549.651, 317.108, 236.644; 320x240: 281.5573273, 281.366942, 156.3332591, 119.8965271
                .setCameraPose(
                        new Position(DistanceUnit.MM, 0, 0, 0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0))
                .build();


        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(HardwareMap.get(WebcamName.class, "camera"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor);



        visionPortal = builder.build();

        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(100);
        } catch (Exception e) {
            Log.i("WHAT A TERRIBLE FAILURE.", "Camera Exposure/Gain Control got fried \n" + e);
        }
    }

    public Pose2d update() {

        visionPortal.setProcessorEnabled(aprilTagProcessor, true);

        detections = aprilTagProcessor.getDetections();

        Log.i("Number of apriltags", "0");
        if (detections != null && !detections.isEmpty()) {
            Log.i("Number of apriltags", String.valueOf(detections.size()));

            if(detections.size() > 1 && Globals.fullField == true) {
                AprilTagDetection detection1 = detections.get(0);
                AprilTagDetection detection2 = detections.get(1);

                if(detection1 != null && detection2 != null) {


                    Pose3D robotPose1 = detection1.robotPose;
                    Pose3D robotPose2 = detection2.robotPose;

                    Pose2d botPose1 = Pose2d.from3D(robotPose1);
                    Pose2d botPose2 = Pose2d.from3D(robotPose2);

                    double d = detection1.decisionMargin / (detection2.decisionMargin * 2);

                    Pose2d overallPose = new Pose2d(botPose1.x * d + botPose2.x * d, botPose1.y * d + botPose2.y * d, detection1.decisionMargin >= detection2.decisionMargin ? botPose1.heading : botPose2.heading);
                    return overallPose;
                }
            } else {
                AprilTagDetection detection = detections.get(0);

                TelemetryUtil.packet.put("Decision Margin", String.valueOf(detection.decisionMargin));

                if (detection != null) {
                    Pose3D robotPose = detection.robotPose;

                    if (robotPose != null) {
                        Pose2d p = Pose2d.from3D(robotPose);

                        p.heading += Math.PI / 2;
                        return p;
                    }
                }
            }
        }



        return null;
    }



}
