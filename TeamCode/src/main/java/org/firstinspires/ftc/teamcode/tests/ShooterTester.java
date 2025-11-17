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
    public static double turretAngle = 0.0, hoodAngle = 0.7, flywheelVelocity = 0.0, rollerPower = 0.8, feedPower = 0.6, latchAngle = 0;
    public static boolean updateVelocity = false;
    public static boolean updateLatch = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Shooter shooter = robot.shooter;
        robot.intake.state = Intake.State.TEST;
        ButtonToggle up = new ButtonToggle();
        ButtonToggle down = new ButtonToggle();

        while (opModeInInit()) {
            robot.update();
        }

        while (!isStopRequested()) {
            shooter.setTurretAngle(turretAngle);
            shooter.setHoodAngle(hoodAngle);
            robot.intake.roller.setTargetPower(rollerPower);
            robot.intake.feed.setTargetPower(feedPower);
            shooter.setShooterBlocker(latchAngle);

            if (updateVelocity) {
                shooter.setTargetVelocity(flywheelVelocity);
                updateVelocity = false;
            }

            robot.update();
        }
    }
}
