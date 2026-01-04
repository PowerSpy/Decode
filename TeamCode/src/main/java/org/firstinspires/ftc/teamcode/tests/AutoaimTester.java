package org.firstinspires.ftc.teamcode.tests;

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

@Config
@TeleOp(group = "Test")
public class AutoaimTester extends LinearOpMode {
    public static double rollerPower = 0.8, feedPower = 0.6, minV0Factor = 1.07, minV0SuperThresh = 0.0;
    public static boolean latchBlock = false, aimReq = false, shootReq = false, stopReq = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);

        robot.intake.state = Intake.State.TEST;

        rollerPower = feedPower = 0;
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0.75 * Math.PI));

        while (opModeInInit());

        while(!isStopRequested()) {

            robot.intake.roller.setTargetPower(rollerPower);
            robot.intake.feed.setTargetPower(feedPower);
            robot.shooter.minV0Superthresh = minV0SuperThresh;
            robot.shooter.minV0factor = minV0Factor;
            robot.shooter.setShooterBlocker(latchBlock);
            robot.shooter.reqAim(aimReq);
            if (aimReq) aimReq = false;
            robot.shooter.reqShoot(shootReq);
            if (shootReq) shootReq = false;
            robot.shooter.reqStop(stopReq);
            if (stopReq) stopReq = false;

            if (aimReq) {
                robot.shooter.reqShoot(aimReq);
                aimReq = false;
            }

            robot.update();
        }

    }

}
