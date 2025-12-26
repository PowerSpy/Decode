package org.firstinspires.ftc.teamcode.tests.localization_testers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Config
@TeleOp
public class LocalizationTest extends LinearOpMode {
    public static boolean constantCorrection = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Globals.RUNMODE = RunMode.TESTER;
        ButtonToggle bty = new ButtonToggle();

        waitForStart();

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        File file = AppUtil.getInstance().getSettingsFile("deceldata.csv");
        FileWriter fw;
        try {
            fw = new FileWriter(file);
        } catch (IOException e) {
            System.out.println("BAD BAD BAD BAD BAD");
            return;
        }

        while(!isStopRequested()) {
            robot.drivetrain.drive(gamepad1);
            robot.drivetrain.mergeLocalizer.setConstantPinpoint(constantCorrection);

            Pose2d pos = robot.drivetrain.getPoseEstimate();

            Log.i ("Localization Test - x", pos.x + "");
            Log.i ("Localization Test - y", pos.y + "");
            Log.i ("Localization Test - heading", pos.heading + "");
            Log.i ("Localization Test - odo 0", ((PriorityMotor) robot.hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition() + "");
            Log.i ("Localization Test - odo 1", ((PriorityMotor) robot.hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition() + "");
            Log.i ("Localization Test - odo 2", ((PriorityMotor) robot.hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition() + "");

            /*
            TelemetryUtil.packet.put("Localizer x: ", pos.x);
            TelemetryUtil.packet.put("Localizer y: ", pos.y);
            TelemetryUtil.packet.put("Localizer heading: ", pos.heading / Math.PI * 180);
            TelemetryUtil.packet.put("odo encoder 0", ((PriorityMotor) robot.hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition());
            TelemetryUtil.packet.put("odo encoder 1", ((PriorityMotor) robot.hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition());
            TelemetryUtil.packet.put("odo encoder 2", ((PriorityMotor) robot.hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition());
            */

            robot.update();

            String buffer = "";

            try {
                fw.write(buffer);
            } catch (IOException e) {
                System.out.println("bad :(");
                return;
            }
            if (bty.isClicked(gamepad1.y)) {
                try {
                    fw.flush();
                } catch (IOException e) {
                    System.out.println("BAD BAD BAD BAD BAD");
                }
            }
        }

    }
}
