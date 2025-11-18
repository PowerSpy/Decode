package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@TeleOp(name = "A. Teleop")
public class Teleop extends LinearOpMode {
    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;
        double angle = 0.46;

        robot.intake.state = Intake.State.TEST;

        while (opModeInInit()) {
            robot.update();
        }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        while (!isStopRequested()) {
            robot.update();

            if (lb1.isClicked(gamepad1.left_bumper)) intakeOn = !intakeOn;
            if (a1.isClicked(gamepad1.a)) {
                intakeReversed = intakeOn && !intakeReversed;
                intakeOn = true;
            }
            if (intakeOn) {
                if (intakeReversed) robot.intake.roller.setTargetPower(-0.8);
                else robot.intake.roller.setTargetPower(0.8);
            } else robot.intake.roller.setTargetPower(0);

            if (b1.isClicked(gamepad1.b)) {
                flywheelOn = !flywheelOn;
                robot.shooter.setTargetVelocity(flywheelOn ? 85 : 0);
            }
            if (gamepad1.right_bumper) {
                robot.intake.feed.setTargetPower(0.5);
                robot.shooter.setShooterBlocker(0);
            } else {
                robot.intake.feed.setTargetPower(0);
                robot.shooter.setShooterBlocker(1);
            }
            if (gamepad1.dpad_up) {
                angle += 0.05;
                robot.shooter.setTurretAngle(angle);
            } else if (gamepad1.dpad_down){
                angle -= 0.05;
                robot.shooter.setTurretAngle(angle);
            }

            robot.drivetrain.drive(gamepad1);

            telemetry.addData("intakeOn", flywheelOn);
            telemetry.addData("intakeReversed", intakeReversed);
            telemetry.addData("intakePower", robot.intake.roller.getPower());
            telemetry.addData("turretPos", robot.shooter.turret.getCurrentAngle());
            telemetry.update();
        }
    }
}
