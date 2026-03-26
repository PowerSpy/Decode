package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_FORWARD_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.vision.BallDetection;

@Config
@Autonomous(name = "XX Blue Far Vision Auto", group = "Auto", preselectTeleOp = "A. Teleop")
public class BlueFarVisionAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 700, intakeDuration = 1500, intakeMoveTimeout = 2000;
    BallDetection b;

    Vector2 visionBall = null;


    public void runOpMode() {
        Globals.isRed = false;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        b = new BallDetection(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(71 - ROBOT_WIDTH / 2, -23.75 + ROBOT_FORWARD_LENGTH, -Math.PI / 2));
        robot.shooter.state = Shooter.State.IDLE;
        robot.shooter.setShooterBlocker(true);

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0P.set(System.currentTimeMillis() % 500 < 100);
        }
        robot.sensors.light0P.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        shoot(-Math.PI / 2, true, false);
        intake(62, -60);
        shoot(-Math.PI / 2, false, false);
        intake(35, -60);
        shoot(-Math.PI / 2, false, true);
        for (int i = 0; i < 2; ++i) {
            intake(50, -60);
            shoot(-Math.PI / 2, false, true);
        }

        robot.shooter.setShooter(Shooter.Dist.OFF);
        robot.shooter.turret.setTargetAngle(0.0);
        robot.drivetrain.goToPoint(new Pose2d(62, -60, -Math.PI / 2), 1.0);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }

    private void shoot(double heading, boolean firstShot, boolean vision) {

        if(vision) b.start();
        //robot.shooter.reqAim(true);

        robot.drivetrain.goToPoint(new Pose2d(firstShot ? 63 : 60, -16, heading), 1);

        robot.waitWhile(() -> {
            visionBall = b.getBestBall();
            return robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY;
        });

        robot.waitFor(firstShot ? 200 : 100);

        robot.shooter.reqShoot(true);
        robot.waitFor(shootDuration);
        robot.shooter.reqStop(true);
        robot.shooter.reqAim(true);

        b.stop();
        if(!vision) visionBall = null;
    }

    private void intake(double x, double y) {
        Log.i("the ball we are aiming for is", visionBall == null ? "The pose is null" : visionBall.toString());
        if(visionBall == null) {
            robot.drivetrain.goToPoint(new Pose2d(x, -22, -Math.PI / 2), 1);
            robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, intakeMoveTimeout);
            robot.intake.reqIntake(true);

            robot.drivetrain.goToPoint(new Pose2d(x, y, Math.PI / 2), 1);
            robot.waitFor(intakeDuration);
            robot.intake.reqOff(true);

            robot.drivetrain.goToPoint(new Pose2d(x, -16, -Math.PI / 2), 1, true);
            robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, intakeMoveTimeout);
        } else {
            robot.intake.reqIntake(true);
            Path path = new Path(Globals.ROBOT_POSITION.clone())
                    .addPoint(new Pose2d(visionBall.x, visionBall.y, -Math.PI/2), false, true);
            robot.drivetrain.setPath(path);
            robot.update();
            robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, 4000);

            path = new Path(Globals.ROBOT_POSITION.clone())
                    .addPoint(new Pose2d(60, -16, -Math.PI/2), false, true);
            robot.drivetrain.setPath(path);
            robot.update();
            robot.intake.reqOff(true);
            robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, 4000);


        }
    }
}
