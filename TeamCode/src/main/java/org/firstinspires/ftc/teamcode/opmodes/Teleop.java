package org.firstinspires.ftc.teamcode.opmodes;

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

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;
        double turretAngle = 0;

        robot.intake.state = Intake.State.TEST;

        while (opModeInInit()) {
            robot.update();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;
        robot.shooter.goalDetector.start();

        while (!isStopRequested()) {
            robot.update();

            if (lb1.isClicked(gamepad1.left_bumper)) intakeOn = !intakeOn;
            if (a1.isClicked(gamepad1.a)) {
                intakeReversed = intakeOn && !intakeReversed;
                intakeOn = true;
            }
            if (intakeOn) {
                if (intakeReversed) robot.intake.roller.setTargetPower(-1);
                else robot.intake.roller.setTargetPower(1);
            } else robot.intake.roller.setTargetPower(0);

            if (b1.isHeld(gamepad1.b, 500)) { // Close
                flywheelOn = false;
                robot.shooter.setTargetVelocity(0);
                robot.shooter.setHoodAngle(0);
            } else if (b1.isClicked(gamepad1.b)) { // Close
                flywheelOn = true;
                robot.shooter.setTargetVelocity(60);
                robot.shooter.setHoodAngle(0.7);
                state = State.CLOSE;
            } else if (y1.isClicked(gamepad1.y)) { // Middle
                flywheelOn = true;
                robot.shooter.setTargetVelocity(70);
                robot.shooter.setHoodAngle(1.0);
                state = State.MID;
            } else if (x1.isClicked(gamepad1.x)) { // Far
                flywheelOn = true;
                robot.shooter.setTargetVelocity(95);
                robot.shooter.setHoodAngle(1.34);
                state = State.FAR;
            }

            if (gamepad1.right_bumper) {
                robot.intake.feed.setTargetPower(0.6);
                robot.shooter.setShooterBlocker(0);
            } else {
                robot.intake.feed.setTargetPower(0);
                robot.shooter.setShooterBlocker(1.5);
            }


            if (gamepad1.left_stick_button) {
                robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
                LogUtil.drivePositionReset = true;
            }
            robot.drivetrain.drive(gamepad1);

            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheel target velocity", robot.shooter.getTargetVelocity());
            telemetry.addData("flywheel current velocity", robot.shooter.getFilteredVelocity());
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("turretPos", robot.shooter.turret.getTargetPos());
            telemetry.update();
        }
    }
}
