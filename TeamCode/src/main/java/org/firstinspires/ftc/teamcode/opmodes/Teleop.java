package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(AUTO_ENDING_POSE);

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;
        boolean firstLoop = true;


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

            robot.drivetrain.drive(gamepad1);

            telemetry.addData("Alliance", Globals.isRed ? "Red" : "Blue");
            telemetry.addData("intakeOn", intakeOn);
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("feedPower", robot.intake.feed.getPower());
            telemetry.addData("flywheelOn", flywheelOn);
            telemetry.addData("flywheelAtVel", robot.shooter.atVel());
            telemetry.addData("flywheel target velocity", robot.shooter.getTargetVelocity());

            telemetry.update();
        }
    }
}
