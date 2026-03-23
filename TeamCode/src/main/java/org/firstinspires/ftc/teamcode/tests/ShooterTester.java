package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(group = "Test")
public class ShooterTester extends LinearOpMode {
    public static double turretTargetDeg = 0.0, hoodAngle = 40, flywheelVelocity = 0.0, rollerPower = 0.8, feedPower = 0.7;
    public static boolean hoodUsesLaunchAngle = true;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.intake.state = Intake.State.TEST;
        robot.shooter.state = Shooter.State.TEST;
        robot.drivetrain.setPoseEstimate(new Pose2d(0,0,0));

        ButtonToggle feedToggleBtn = new ButtonToggle();
        ButtonToggle intakeToggleBtn = new ButtonToggle();
        ButtonToggle latchToggleBtn = new ButtonToggle();
        ButtonToggle feedBtn = new ButtonToggle();

        //rollerPower = feedPower = 0;
        flywheelVelocity = 0;
        double flywheelDrop = 0;

        while (opModeInInit()) {
            robot.update();
        }

        while (!isStopRequested()) {
            if (feedBtn.isClicked(gamepad1.right_bumper)) flywheelDrop = 1000;
            if (gamepad1.right_bumper) flywheelDrop = Math.min(flywheelDrop, robot.sensors.getFlywheelVelocity());
            TelemetryUtil.packet.put("Flywheel velocity dropped to (in/s)", flywheelDrop);

            robot.shooter.turret.setTargetAngle(Math.toRadians(turretTargetDeg));
            robot.shooter.setHoodAngle(hoodUsesLaunchAngle ? (Math.toRadians(hoodAngle) - Shooter.hoodSweep) * Shooter.hoodGearRatio : hoodAngle);
            robot.intake.roller.setTargetPower(intakeToggleBtn.isToggled(gamepad1.x) || gamepad1.left_bumper ? rollerPower : 0);
            robot.intake.feed.setTargetPower(feedToggleBtn.isToggled(gamepad1.y) || gamepad1.right_bumper ? feedPower : 0);
            robot.shooter.setShooterBlocker(latchToggleBtn.isToggled(gamepad1.b));
            robot.shooter.flywheel.setTargetVelocity(flywheelVelocity);
            robot.update();
        }
    }
}
