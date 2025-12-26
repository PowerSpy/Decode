package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(name = "B. nTeleop")
public class nTeleop extends LinearOpMode {

    // DO NOT USE TS
    // THIS IS COMPLETELY DYSFUNCTIONAL

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.shooter.state = Shooter.State.TEST;

        robot.drivetrain.setPoseEstimate(AUTO_ENDING_POSE);

        robot.shooter.setShooterBlocker(true);

        ButtonToggle lb1 = new ButtonToggle(); // intake stuff
        ButtonToggle rb1 = new ButtonToggle(); // shoot stuff
        ButtonToggle a1 = new ButtonToggle(); // intake stuff
        ButtonToggle b1 = new ButtonToggle(); // cancel shooter into IDLE
        ButtonToggle y1 = new ButtonToggle(); // roller
        ButtonToggle x1 = new ButtonToggle(); // aim

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;
        boolean atSpeedRumble = false;
        boolean firstLoop = true;
        Shooter.Dist dist = Shooter.Dist.OFF;

        while (opModeInInit());

        if (!isStopRequested()) LogUtil.init();

        LogUtil.drivePositionReset = true;

        while (!isStopRequested()) {
            robot.update();

            // INTAKE
            if (lb1.isClicked(gamepad1.left_bumper)) {
                intakeOn = !intakeOn;
                if (intakeOn) {
                    robot.intake.reqIntake(true);
                } else {
                    robot.intake.reqOff(true);
                }
                robot.intake.setRollerDirection(false);
            }

            if (a1.isClicked(gamepad1.a)) {
                intakeReversed = intakeOn && !intakeReversed;
                intakeOn = true;
                robot.intake.reqIntake(true);
                robot.intake.setRollerDirection(intakeReversed);
            }

            // SHOOTER

            if (atSpeedRumble && firstLoop) {
                firstLoop = false;
            } else if (atSpeedRumble && robot.shooter.atVel()) {
                gamepad1.rumble(100);
                gamepad2.rumble(100);
                atSpeedRumble = false;
            }

            if (gamepad1.right_bumper) {
                rb1.isReleased(gamepad1.right_bumper);
                robot.shooter.setShooterBlocker(false);
                // if (robot.shooter.flywheelBlocker.inPosition())
                robot.intake.reqShoot(true);
                // robot.intake.feed.setTargetPower(robot.shooter.flywheelBlocker.inPosition() ? feedPower : 0);
            } else if (rb1.isReleased(gamepad1.right_bumper)) {
                robot.shooter.setShooterBlocker(true);
                intakeOn = false;
                robot.intake.reqOff(true);
                // robot.intake.feed.setTargetPower(intakeOn ? (intakeReversed ? -idleFeedPower : idleFeedPower) : 0);
            }
            /*
            if (b1.isClicked(gamepad1.b)) {
                robot.shooter.reqAim(true);
            }

            // rumble when ready to shoot
            if (robot.shooter.state == Shooter.State.READY && firstLoop) {
                gamepad1.rumble(150);
                gamepad2.rumble(150);
                firstLoop = false;
            } else if (robot.shooter.state != Shooter.State.READY) {
                firstLoop = true;
            }

            if (rb1.isClicked(gamepad1.right_bumper)) {
                if (robot.shooter.state == Shooter.State.SHOOT) {
                    robot.shooter.reqStop(true);
                } else {
                    robot.shooter.reqShoot(true);
                }
            }
             */

            robot.drivetrain.drive(gamepad1);

            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");
            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("shooter preset", dist);
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("feedPower", robot.intake.feed.getPower());
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheel target velocity", robot.shooter.getTargetVelocity());
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("robot thonker its at", ROBOT_POSITION.x + ", " + ROBOT_POSITION.y + ", " + ROBOT_POSITION.heading);

            telemetry.update();
        }
    }
}
