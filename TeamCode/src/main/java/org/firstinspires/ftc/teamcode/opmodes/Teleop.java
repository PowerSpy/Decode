package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;

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
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {

    public enum State {
        CLOSE,
        MID,
        FAR
    } State state = State.FAR;

    public static double feedPower = 0.6, idleFeedPower = 0.4, intakePower = 0.8;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(AUTO_ENDING_POSE);

        ButtonToggle lb1 = new ButtonToggle(); // intake stuff
        ButtonToggle rb1 = new ButtonToggle(); // shoot stuff
        ButtonToggle a1 = new ButtonToggle(); // intake stuff
        ButtonToggle b1 = new ButtonToggle(); // close
        ButtonToggle y1 = new ButtonToggle(); // middle
        ButtonToggle x1 = new ButtonToggle(); // far

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

        while (opModeInInit()) robot.update();

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
            if (b1.isHeld(gamepad1.b, 500) || b2.isHeld(gamepad2.b, 500)
                    || y1.isHeld(gamepad1.y, 500) || y2.isHeld(gamepad2.y, 500)
                    || x1.isHeld(gamepad1.x, 500) || x2.isHeld(gamepad2.x, 500)) { // Off
                flywheelOn = false;
                robot.shooter.setShooter(Shooter.Dist.OFF);
                dist = Shooter.Dist.OFF;
            } else if (b1.isClicked(gamepad1.b) || b2.isClicked(gamepad2.b)) { // Close
                flywheelOn = true;
                robot.shooter.setShooter(Shooter.Dist.CLOSE);
                state = State.CLOSE;
                dist = Shooter.Dist.CLOSE;
                atSpeedRumble = true;
                firstLoop = true;
            } else if (y1.isClicked(gamepad1.y) || y2.isClicked(gamepad2.y)) { // Middle
                flywheelOn = true;
                robot.shooter.setShooter(Shooter.Dist.MID);
                state = State.MID;
                dist = Shooter.Dist.MID;
                atSpeedRumble = true;
                firstLoop = true;
            } else if (x1.isClicked(gamepad1.x) || x2.isClicked(gamepad2.x)) { // Far
                flywheelOn = true;
                robot.shooter.setShooter(Shooter.Dist.FAR);
                state = State.FAR;
                dist = Shooter.Dist.FAR;
                atSpeedRumble = true;
                firstLoop = true;
            }
            if(atSpeedRumble && firstLoop) {
                    firstLoop = false;
            } else if (atSpeedRumble && robot.shooter.atVelBootleg()) {
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                atSpeedRumble = false;
            }


            if ((gamepad1.right_bumper || gamepad2.right_bumper) && dist != Shooter.Dist.OFF) {
                robot.shooter.setShooterBlocker(false);
                robot.intake.feed.setTargetPower(robot.shooter.flywheelBlocker.inPosition() ? feedPower : 0);
            } else {
                robot.intake.feed.setTargetPower(intakeOn ? (intakeReversed ? -idleFeedPower : idleFeedPower) : 0);
                robot.shooter.setShooterBlocker(true);
            }
            /*
            if (b1.isClicked(gamepad1.b)) {
                robot.shooter.reqAim(true);
            }

            // rumble when ready to shoot
            if (robot.shooter.state == Shooter.State.READY && firstLoop) {
                gamepad1.rumble(150);
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
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("feedPower", robot.intake.feed.getPower());
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("flywheelAtVelBootleg", robot.shooter.atVelBootleg());
            telemetry.addData("flywheel target velocity", robot.shooter.getTargetVelocity());

            telemetry.update();
        }
    }
}
