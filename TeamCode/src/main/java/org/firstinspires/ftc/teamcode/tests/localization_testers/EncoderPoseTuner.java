package org.firstinspires.ftc.teamcode.tests.localization_testers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp
public class EncoderPoseTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        //Sensors sensors = new Sensors(hardwareMap);
        //HardwareQueue hardwareQueue = new HardwareQueue();
        //Vision vision = new Vision(hardwareMap);
        //Drivetrain drivetrain = new Drivetrain(hardwareMap, sensors, hardwareQueue, vision);

        double leftInitial = drivetrain.leftFront.motor[0].getCurrentPosition();
        double rightInitial = drivetrain.rightFront.motor[0].getCurrentPosition();
        double backInitial = drivetrain.leftRear.motor[0].getCurrentPosition();
        double theta;

        waitForStart();

        while (!isStopRequested()) {
            START_LOOP();

            drivetrain.drive(gamepad1);
            theta = Math.PI * 20; // 10 rotations

            robot.sensors.update();
            drivetrain.update();

            telemetry.addData("leftOdoRadius", (drivetrain.leftFront.motor[0].getCurrentPosition() - leftInitial) * drivetrain.nMergeLocalizer.encoders[0].ticksToInches/theta + "");
            telemetry.addData("rightOdoRadius", (drivetrain.rightFront.motor[0].getCurrentPosition() - rightInitial) * drivetrain.nMergeLocalizer.encoders[1].ticksToInches/theta + "");
            telemetry.addData("backOdoRadius", (drivetrain.leftRear.motor[0].getCurrentPosition() - backInitial) * drivetrain.nMergeLocalizer.encoders[2].ticksToInches/theta + "");
            telemetry.addData("heading", ROBOT_POSITION.heading);
            telemetry.update();
        }
    }
}
