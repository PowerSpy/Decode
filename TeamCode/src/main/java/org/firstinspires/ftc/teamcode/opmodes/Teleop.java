package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.isRed;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.Locale;

@Config
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        //robot.shooter.state = Shooter.State.TEST;

        robot.drivetrain.setPoseEstimate(AUTO_ENDING_POSE);

        robot.shooter.setShooterBlocker(true);

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle rt1 = new ButtonToggle();

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();
        ButtonToggle back2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;
        boolean atSpeedRumble = false;
        boolean confirmation = true;
        boolean manualToggled = false;
        final double triggerThresh = 0.2;

        while (opModeInInit()) {
            robot.sensors.update();
            TelemetryUtil.sendTelemetry();
        }

        if (!isStopRequested()) LogUtil.init();

        LogUtil.drivePositionReset = true;

        while (!isStopRequested()) {
            robot.update();

            if (back2.isClicked(gamepad2.back)) {
                isRed = !isRed;
                robot.shooter.updateBallTarget();
            }

            // INTAKE
            if (lb1.isClicked(gamepad1.left_bumper)) {
                intakeOn = !intakeOn;
                intakeReversed = false;
                if (intakeOn) {
                    robot.intake.reqIntake(true);
                } else {
                    robot.intake.reqOff(true);
                }
                robot.intake.setRollerDirection(false);
                //robot.sensors.light0G.setState(!intakeOn);
                //robot.sensors.light0P.setState(true);
            }

            if (a1.isClicked(gamepad1.a)) {
                intakeReversed = !intakeOn || !intakeReversed;
                intakeOn = true;
                robot.intake.reqIntake(true);
                robot.intake.setRollerDirection(intakeReversed);
                //robot.sensors.light0G.setState(intakeReversed);
                //robot.sensors.light0P.setState(!intakeReversed);
            }

            // SHOOTER

            if (a2.isHeld(gamepad2.a, 500)) {
                if (!manualToggled) {
                    manualToggled = true;
                    robot.shooter.setManual(robot.shooter.state != Shooter.State.TEST);
                    robot.shooter.setShooter(Shooter.Dist.OFF);
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                }
            } else {
                manualToggled = false;
            }

            if (robot.shooter.state == Shooter.State.TEST) {
                rb1.isClicked(gamepad1.right_bumper);

                if (rt1.isClicked(gamepad1.right_trigger > triggerThresh) || b1.isHeld(gamepad1.b, 500) || b2.isHeld(gamepad2.b, 500)
                        || y1.isHeld(gamepad1.y, 500) || y2.isHeld(gamepad2.y, 500)
                        || x1.isHeld(gamepad1.x, 500) || x2.isHeld(gamepad2.x, 500)) { // Off
                    flywheelOn = false;
                    robot.shooter.setShooter(Shooter.Dist.OFF);
                    atSpeedRumble = false;
                } else if (b1.isClicked(gamepad1.b) || b2.isClicked(gamepad2.b)) { // Close
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.CLOSE);
                    atSpeedRumble = true;
                    confirmation = true;
                } else if (y1.isClicked(gamepad1.y) || y2.isClicked(gamepad2.y)) { // Middle
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.MID);
                    atSpeedRumble = true;
                    confirmation = true;
                } else if (x1.isClicked(gamepad1.x) || x2.isClicked(gamepad2.x)) { // Far
                    flywheelOn = true;
                    robot.shooter.setShooter(Shooter.Dist.FAR);
                    atSpeedRumble = true;
                    confirmation = true;
                }

                if (atSpeedRumble && confirmation) {
                    confirmation = false;
                } else if (atSpeedRumble && robot.shooter.atVel()) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                    atSpeedRumble = false;
                }

                if (gamepad1.right_bumper) {
                    rb1.isReleased(gamepad1.right_bumper);
                    robot.shooter.setShooterBlocker(false);
                    robot.intake.reqShoot(true);
                } else if (rb1.isReleased(gamepad1.right_bumper)) {
                    robot.shooter.setShooterBlocker(true);
                    intakeOn = false;
                    robot.intake.reqOff(true);
                }
            } else {
                x1.isClicked(gamepad1.x);
                x2.isClicked(gamepad2.x);

                if (y1.isClicked(gamepad1.y) || y2.isClicked(gamepad2.y)) {
                    robot.shooter.reqAim(true);
                }

                if (robot.shooter.state == Shooter.State.READY) {
                    if (confirmation) {
                        gamepad1.rumble(150);
                        gamepad2.rumble(150);
                        confirmation = false;
                    }
                } else {
                    confirmation = true;
                }

                if (rt1.isClicked(gamepad1.right_trigger > triggerThresh) || b1.isClicked(gamepad1.b) || b2.isClicked(gamepad2.b)) {
                    robot.shooter.reqStop(true);
                }

                if (rb1.isClicked(gamepad1.right_bumper)) {
                    if (robot.shooter.state != Shooter.State.READY) {
                        robot.shooter.reqStop(true);
                    } else {
                        robot.shooter.reqShoot(true);
                    }
                }
            }

            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                gamepad1.rumble(1000);
                gamepad2.rumble(1000);
                robot.drivetrain.setPoseEstimate(new Pose2d(72 - 6, (72 - 6) * (isRed ? -1 : 1), Math.PI));
            }

            robot.drivetrain.drive(gamepad1);

            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");
            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("feedPower", robot.intake.feed.getPower());
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheel target velocity", robot.shooter.getTargetVelocity());
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("shooter state", robot.shooter.state.toString());
            telemetry.addData("Robot position (deg)", String.format(Locale.US, "(%.2f, %.2f, %.2f)", ROBOT_POSITION.x, ROBOT_POSITION.y, Math.toDegrees(ROBOT_POSITION.heading)));

            telemetry.update();
        }
    }
}
