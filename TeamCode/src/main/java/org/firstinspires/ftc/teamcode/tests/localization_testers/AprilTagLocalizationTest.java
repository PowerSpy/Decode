package org.firstinspires.ftc.teamcode.tests.localization_testers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.annotation.SuppressLint;
import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp(group = "Test")
@Config

public class AprilTagLocalizationTest extends LinearOpMode {
    private Vision vision;
    Pose2d botPose;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        TelemetryUtil.setup();

        vision = new Vision(hardwareMap);

        while (opModeInInit()) {
            vision.update();
        }

        while (!isStopRequested()) {

            botPose = vision.update();

            if (botPose != null) {

                telemetry.addData("x", botPose.x);

                Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
                DashboardUtil.drawRobot(fieldOverlay, botPose, "#000000");

                TelemetryUtil.sendTelemetry();

            } else {
                Log.i("Vision", "Null");
            }
        }
    }



}
