package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
    public enum State{
        CLOSE,
        MID,
        FAR
    } State state = State.FAR;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(AUTO_ENDING_POSE);

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;

        robot.intake.state = Intake.State.TEST;

        while (opModeInInit()) robot.update();

        if (!isStopRequested()) LogUtil.init();

        LogUtil.drivePositionReset = true;
        // robot.shooter.goalDetector.start();

        while (!isStopRequested()) {
            robot.update();

            if (lb1.isClicked(gamepad1.left_bumper)) {
                intakeReversed = false;
                intakeOn = !intakeOn;
            }
            if (a1.isClicked(gamepad1.a)) {
                intakeReversed = intakeOn && !intakeReversed;
                intakeOn = true;
            }
            robot.intake.roller.setTargetPowerSmooth(intakeOn ? (intakeReversed ? -0.8 : 0.8) : 0);

            if (b1.isHeld(gamepad1.b, 500) || b2.isHeld(gamepad2.b, 500)
            || y1.isHeld(gamepad1.y, 500) || y2.isHeld(gamepad2.y, 500)
            || x1.isHeld(gamepad1.x, 500) || x2.isHeld(gamepad2.x, 500)) { // Off
                flywheelOn = false;
                robot.shooter.setTargetVelocity(0);
                robot.shooter.setHoodAngle(0);
            } else if (b1.isClicked(gamepad1.b) || b2.isClicked(gamepad2.b)) { // Close
                flywheelOn = true;
                robot.shooter.setTargetVelocity(65);
                robot.shooter.setHoodAngle(0.7);
                state = State.CLOSE;
            } else if (y1.isClicked(gamepad1.y) || y2.isClicked(gamepad2.y)) { // Middle
                flywheelOn = true;
                robot.shooter.setTargetVelocity(75);
                robot.shooter.setHoodAngle(1.0);
                state = State.MID;
            } else if (x1.isClicked(gamepad1.x) || x2.isClicked(gamepad2.x)) { // Far
                flywheelOn = true;
                robot.shooter.setTargetVelocity(90);
                robot.shooter.setHoodAngle(1.34);
                state = State.FAR;
            }

            // activate feed / toggling flywheel blocker
            if (gamepad1.right_bumper) {
                robot.intake.feed.setTargetPower(0.6);
                robot.shooter.setShooterBlocker(false);
            } else {
                robot.intake.feed.setTargetPower(0);
                robot.shooter.setShooterBlocker(true);
            }

            // toggle alliance

            if (a2.isClicked(gamepad2.a)) {
                Globals.isRed = !Globals.isRed;
            }

            robot.drivetrain.drive(gamepad1);

            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");

            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("shooter distance", state);
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheel target velocity", robot.shooter.getTargetVelocity());
            telemetry.addData("flywheel current velocity", robot.shooter.getFilteredVelocity());
            telemetry.addData("turretPos", robot.shooter.turret.getTargetPos());

            telemetry.update();
        }
    }
}
