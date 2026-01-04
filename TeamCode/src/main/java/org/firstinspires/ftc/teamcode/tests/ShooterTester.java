package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(group = "Test")
public class ShooterTester extends LinearOpMode {
    public static double turretAngle = 0.0, hoodAngle = 0.7, flywheelVelocity = 0.0, rollerPower = 0.8, feedPower = 0.6;
    public static boolean latchBlock = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);

        robot.intake.state = Intake.State.TEST;
        robot.shooter.state = Shooter.State.TEST;

        ButtonToggle feedBtn = new ButtonToggle();
        ButtonToggle intakeBtn = new ButtonToggle();

        rollerPower = feedPower = 0;
        flywheelVelocity = 0;

        while (opModeInInit()) {
            robot.update();
        }

        while (!isStopRequested()) {
            robot.shooter.setTurretAngle(turretAngle);
            robot.shooter.setHoodAngle(hoodAngle);
            robot.intake.roller.setTargetPower(intakeBtn.isToggled(gamepad1.x) ? 0 : rollerPower);
            robot.intake.feed.setTargetPower(feedBtn.isToggled(gamepad1.y) ? 0 : feedPower);
            robot.shooter.setShooterBlocker(latchBlock);
            robot.shooter.setTargetVelocity(flywheelVelocity);

            robot.update();

        }
    }
}
