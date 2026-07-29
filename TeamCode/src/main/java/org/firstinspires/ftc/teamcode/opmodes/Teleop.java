package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.MergeLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Locale;

@Config
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap, true);

        //robot.drivetrain.vision.start();
        robot.setStopChecker(this::isStopRequested);

        robot.deposit.state = Deposit.State.IDLE;

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle lt1 = new ButtonToggle();
        ButtonToggle rt1 = new ButtonToggle();
        ButtonToggle back1 = new ButtonToggle();

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();
        ButtonToggle back2 = new ButtonToggle();
        ButtonToggle h2 = new ButtonToggle();
        ButtonToggle v2 = new ButtonToggle();
        ButtonToggle lb2 = new ButtonToggle();
        ButtonToggle rb2 = new ButtonToggle();
        ButtonToggle guide2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean flywheelOn = false;
        boolean atSpeedRumble = false;
        boolean confirmation = true;
        final double triggerThresh = 0.2;

        while (opModeInInit()) {
            robot.sensors.update();

            if (x1.isHeld(gamepad1.x, 1000)) {
                LogUtil.DISABLED = true;
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
            if (y1.isHeld(gamepad1.y, 1000)) {
                LogUtil.DISABLED = false;
                gamepad1.rumble(200);
                gamepad2.rumble(200);
            }
            TelemetryUtil.sendTelemetry();
            telemetry.addData("CAT", LogUtil.DISABLED ? "DISABLED" : "ENABLED");
            telemetry.update();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        //robot.drivetrain.vision.start();
        //robot.shooter.setManual(false);

        while (!isStopRequested()) {
            robot.update();
            robot.drivetrain.drive(gamepad1, true);

            if (back2.isClicked(gamepad2.back)) {
                isRed = !isRed;
            }

            // INTAKE

            if (lb1.isClicked(gamepad1.left_bumper)) {
                if (robot.intake.state == NewIntake.State.IDLE) robot.intake.requestIntake(true);
                else robot.intake.reqOff(true);
                robot.intake.setRollerDirection(false);
            }

            if (a1.isClicked(gamepad1.a && !gamepad1.start)) {
                intakeReversed = robot.intake.state == NewIntake.State.IDLE || !intakeReversed;
                robot.intake.reqOff(true);
                robot.intake.setRollerDirection(intakeReversed);
            }

            if (rb1.isClicked(gamepad1.right_bumper)) {
                if (robot.deposit.state == Deposit.State.RAISED)
                {
                    robot.deposit.requestDump = true;
                }
                else if(robot.deposit.state == Deposit.State.IDLE)
                {
                    robot.intake.requestTransfer(true);
                }
            }

            // LOCALIZER

            if (x2.isHeld(gamepad2.x, 500)) { // localize to origin
                robot.sensors.setOdometryPosition(Pose2d.fromSensorsPose2d(new Pose2d(0,0,0)));
                gamepad1.rumble(1200);
                gamepad2.rumble(1200);
            }

            if (h2.isClicked(gamepad2.dpad_left || gamepad2.dpad_right)) { // localize to left/right edge (unchanged x, auto y, auto h)
                double h = Utils.headingClip(ROBOT_POSITION.heading);
                if (h < Math.toRadians(-135)) h = -Math.PI;
                else if (h < Math.toRadians(-45)) h = -Math.PI / 2;
                else if (h > Math.toRadians(135)) h = Math.PI;
                else if (h > Math.toRadians(45)) h = Math.PI / 2;
                else h = 0;
                robot.sensors.setOdometryPosition(Pose2d.fromSensorsPose2d(new Pose2d(ROBOT_POSITION.x, (ROBOT_POSITION.y > 0 ? 1 : -1) * (71 - 6.5), h)));
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
            }

            if (v2.isClicked(gamepad2.dpad_up || gamepad2.dpad_down)) { // localize to top/bottom edge (auto x, unchanged y, auto h)
                double h = Utils.headingClip(ROBOT_POSITION.heading);
                if (h < Math.toRadians(-135)) h = -Math.PI;
                else if (h < Math.toRadians(-45)) h = -Math.PI / 2;
                else if (h > Math.toRadians(135)) h = Math.PI;
                else if (h > Math.toRadians(45)) h = Math.PI / 2;
                else h = 0;
                robot.sensors.setOdometryPosition(Pose2d.fromSensorsPose2d(new Pose2d((ROBOT_POSITION.x > 0 ? 1 : -1) * (71 - 6.5), ROBOT_POSITION.y, h)));
                gamepad1.rumble(800);
                gamepad2.rumble(800);
            }
            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("intakePower", robot.intake.getRollerPower());
            telemetry.addData("deposit state", robot.deposit.state.toString());
            //telemetry.addData("flywheelOn", flywheelOn);
            //telemetry.addData("Shooter auto shoot when in zone", Shooter.autoShootIfInZone);

            telemetry.addData("Robot position (deg)", String.format(Locale.US, "(%.2f, %.2f, %.2f)", ROBOT_POSITION.x, ROBOT_POSITION.y, Math.toDegrees(ROBOT_POSITION.heading)));
            telemetry.addData("CAT", LogUtil.DISABLED ? "DISABLED" : "ENABLED");
            //telemetry.addData("Vision : relocalize count", robot.drivetrain.nMergeLocalizer.numberOfTimesRelocalizedWithCamera); TODO: Implement
            telemetry.addData("Vision : use camera", MergeLocalizer.useCamera);

            telemetry.update();
        }
        robot.drivetrain.drive(gamepad1, false);
        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
}
