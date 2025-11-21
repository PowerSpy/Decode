package org.firstinspires.ftc.teamcode.tests.localization_testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@TeleOp
public class EncoderPoseTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Sensors sensors = robot.sensors;

        double leftInitial = robot.drivetrain.leftRear.motor[0].getCurrentPosition();
        double rightInitial = robot.drivetrain.rightRear.motor[0].getCurrentPosition();
        double backInitial = robot.drivetrain.rightFront.motor[0].getCurrentPosition();
        double theta;

        waitForStart();

        while (!isStopRequested()) {
            drivetrain.drive(gamepad1);
            theta = Math.PI * 20; // 10 rotations

            robot.update();

            telemetry.addData("leftOdoRadius", (robot.drivetrain.leftRear.motor[0].getCurrentPosition() - leftInitial) * robot.drivetrain.localizers[0].encoders[0].ticksToInches/theta + "");
            telemetry.addData("rightOdoRadius", (robot.drivetrain.leftFront.motor[0].getCurrentPosition() - rightInitial) * robot.drivetrain.localizers[0].encoders[1].ticksToInches/theta + "");
            telemetry.addData("backOdoRadius", (robot.drivetrain.rightFront.motor[0].getCurrentPosition() - backInitial) * robot.drivetrain.localizers[0].encoders[2].ticksToInches/theta + "");
            telemetry.update();
        }
    }
}
