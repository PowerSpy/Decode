package org.firstinspires.ftc.teamcode.opmodes;


import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
@Config
@Autonomous(name = "RedTunnelParkAuto")
public class RedTunnelParkAuto extends LinearOpMode {
    private Robot robot;

    private double timer = System.currentTimeMillis();

    public static double heading = 2.71;


    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(72 - ROBOT_LENGTH / 2, ROBOT_WIDTH / 2, Math.PI));

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);


        while (opModeInInit()) { robot.update(); }

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        robot.shooter.setShooter(Shooter.Dist.FAR);


        shoot(heading);
        intake(34, 54, true);
        shoot(heading - 0.03);
        intake(8, 54, true);
        shoot(heading - 0.05);
        intake(-15 , 54, false);
        shoot(heading - 0.07);

        robot.drivetrain.goToPoint(new Pose2d(0, 48 - ROBOT_WIDTH / 2, Math.PI/2), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading) {

        robot.drivetrain.goToPoint(new Pose2d(48, 13, heading), 1.0);
        robot.waitWhile(() -> {
            //robot.shooter.turretTrackTarget();
            robot.shooter.targetTurretAngle = 0.0;
            return robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel() || Math.abs(robot.shooter.targetTurretAngle - robot.sensors.getTurretAngle()) > 10;
        });

        robot.shooter.setShooterBlocker(false);
        timer = System.currentTimeMillis();
        robot.intake.reqShoot(true);
        robot.waitWhile(() -> {
            //robot.shooter.turretTrackTarget();
            robot.shooter.targetTurretAngle = 0.0;
            return System.currentTimeMillis() - timer <= 1000;
        });

        robot.shooter.setShooterBlocker(true);
        robot.intake.reqOff(true);
    }
    private void intake(double x, double y, boolean far) {
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.toRadians(90)), 1.0);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);

        timer = System.currentTimeMillis();
        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.toRadians(90)), 0.6);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT && System.currentTimeMillis() - timer <= 3500);
        if(far) {
            robot.drivetrain.goToPoint(new Pose2d(x, 60, Math.toRadians(90)), 0.8, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
            robot.intake.reqOff(true);
        }
    }

}
