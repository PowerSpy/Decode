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
    public static double rollerPower = 0.8, feedPower = 0.6, minV0FactorClose = Shooter.minV0factorClose, minV0FactorFar = Shooter.minV0factorFar, minV0SuperThresh = 0.0, flywheelEfficiency = 0.63, flywheelEfficiencyConstantFarAddition = -0.05;
    public static double posX = -24.0, posY = 24, posHeading = 0.75 * Math.PI;
    public static boolean latchBlock = false, aimReq = false, shootReq = false, stopReq = false;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);

        robot.intake.state = Intake.State.TEST;

        rollerPower = feedPower = 0;
        robot.drivetrain.setPoseEstimate(new Pose2d(posX, posY, posHeading));

        while (opModeInInit());

        while(!isStopRequested()) {

            robot.drivetrain.setPoseEstimate(new Pose2d(posX, posY, posHeading));
            robot.intake.roller.setTargetPower(rollerPower);
            robot.intake.feed.setTargetPower(feedPower);
            robot.shooter.minV0Superthresh = minV0SuperThresh;
            Shooter.minV0factorClose = minV0FactorClose;
            Shooter.minV0factorFar = minV0FactorFar;
            Shooter.flywheelEfficiency = flywheelEfficiency;
            Shooter.flywheelEfficiencyConstantFarAddition = flywheelEfficiencyConstantFarAddition;
            robot.shooter.setShooterBlocker(latchBlock);

            robot.shooter.reqAim(aimReq);
            if (aimReq) aimReq = false;
            robot.shooter.reqShoot(shootReq);
            if (shootReq) shootReq = false;
            robot.shooter.reqStop(stopReq);
            if (stopReq) stopReq = false;

            robot.update();
        }

    }

}
