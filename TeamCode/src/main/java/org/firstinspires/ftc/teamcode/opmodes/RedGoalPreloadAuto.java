package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@Autonomous(name = "* Red Goal Preload Auto", group = "Auto", preselectTeleOp = "A. Teleop")
public class RedGoalPreloadAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 700, intakeDuration = 1000, gateIntakeDuration = 700, timeout = 2500;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-71 + ROBOT_BACK_LENGTH, 24.25 + ROBOT_WIDTH / 2, 0));

        robot.shooter.state = Shooter.State.TEST;
        robot.shooter.setShooterBlocker(true);
        robot.shooter.turretTrackInManual = true;

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 350);
        }
        robot.sensors.light0G.set(false);

        if (!isStopRequested()) LogUtil.init();
        LogUtil.drivePositionReset = true;

        //robot.drivetrain.goToPoint(new Pose2d(-40, 40, 0), 1.0);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        //robot.shooter.setShooter(Shooter.Dist.AUTO_POSITION);

        long t = System.currentTimeMillis();
        robot.shooter.setManual(false);
        //robot.shooter.reqAim(true);
        //robot.drivetrain.goToPoint(new Pose2d(-18, 18, 0), 1, true);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        //preloads
        shoot(Math.toRadians(126), 1, true, true);
        intake(11.5, 60, false, false);
        open_gate(500);

        //middle spikemarks
        shoot(Math.toRadians(90), 1, true, false);

        intake(-13, 51, true, false);
        shoot(Math.toRadians(80), 2, true, false);
        //gate intake and shoot
        gate_intake(true);
        shoot(Math.toRadians(80), 1, false, false);
//gate intake and shoot        gate_intake(true);
//gate intake and shoot
        gate_intake(true);
        shoot(Math.toRadians(80), 1, false, false);
        //gate intake and shoot
        //gate_intake(true);
        //shoot(Math.toRadians(80), 1, false, false);
        //robot.shooter.reqStop(true);
        //robot.shooter.turret.setTargetAngle(0.0);
        robot.shooter.setManual(true);
        robot.drivetrain.goToPoint(new Pose2d(0, 40, Math.PI / 2), 1.0);

        long x = System.currentTimeMillis() - t;
        TelemetryUtil.packet.put("Time : ", x);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
    private void open_gate(long duration) {
        // align
        robot.drivetrain.goToPoint(new Pose2d(0, 48, -Math.PI / 2), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        // hit gate
        robot.drivetrain.goToPoint(new Pose2d(0, 53, -Math.PI / 2), 0.5);
        robot.waitFor(duration);

        // back off
        robot.drivetrain.goToPoint(new Pose2d(0, 30, -Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
    }


    private void shoot(double heading, int shotType, boolean move, boolean first) {
        robot.shooter.reqAim(true);
        if (move) {
            if (first) {
                robot.drivetrain.goToPoint(new Pose2d(-36, 36, heading), 1, false);
                //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel() || !robot.shooter.turret.inPosition());
                robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);
                robot.waitFor(200);
            } else {
                robot.drivetrain.goToPoint(new Pose2d(shotType == 1 ? -6 : -18, shotType == 1 ? 10 : 18, heading), 1);
                //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || !robot.shooter.atVel() || !robot.shooter.turret.inPosition());
                robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY);
                robot.waitFor(200);
            }
        } else {
            robot.waitWhile(() -> robot.shooter.state != Shooter.State.READY);
            robot.waitFor(200);
        }
        robot.shooter.reqShoot(true);

        //robot.shooter.setShooterBlocker(false);
        //robot.intake.reqShoot(true);
        robot.waitFor(shootDuration);
        //robot.shooter.setShooterBlocker(true);
        //robot.intake.reqOff(true);

        robot.shooter.reqStop(true);
        if (first) {
            robot.drivetrain.goToPoint(new Pose2d(-6, 10, Math.toRadians(126)), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

            robot.drivetrain.goToPoint(new Pose2d(11.5, 20, Math.toRadians(90)), 1, false);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        } else {
            robot.waitFor(50);

        }
        //if (recharge) robot.shooter.reqAim(true);
    }

    private void intake(double x, double y, boolean skipLast, int spike) {
        // align
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);
        int[] balls = new int[3];
        if (spike != 0) balls[3 - spike] = 1;
        int[] pattern = new int[3];
        pattern[Globals.BALL_PATTERN - 21] = 1;
        for (int i = 0; i < 3; i++) {
            if (balls[i] != pattern[i]) {

            }
        }

        // intake
        robot.drivetrain.goToPoint(new Pose2d(x, 40, Math.PI / 2), 1);
        robot.waitFor(intakeDuration);

        robot.drivetrain.goToPoint(new Pose2d(x, 40, Math.PI / 2), 1);

        robot.drivetrain.goToPoint(new Pose2d(x, 40, Math.PI / 2), 1);

        // back off
        if (!skipLast) {
            robot.drivetrain.goToPoint(new Pose2d(x, 30, Math.PI / 2), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }
        robot.intake.reqOff(true);
    }

    private void gate_intake(boolean skipLast) {
        //-7, 7
        // align

        /*
        robot.drivetrain.goToPoint(new Pose2d(4, 22, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);
        */

        //-7, 7
        //-5. 17
        //-3. 27
        //0, 37
        //2, 47
        //4, 54

        robot.intake.reqIntake(true);

        // hit gate
        robot.drivetrain.goToPoint(new Pose2d(4, 51, Math.toRadians(80)), 1);
        robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, timeout);
        //robot.waitFor(gateOpenDuration);

        //robot.drivetrain.goToPoint(new Pose2d(15, 53, Math.PI / 2), 1, true);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        // gate intake
        /*
        robot.drivetrain.goToPoint(new Pose2d(5,63, Math.toRadians(118)), 0.7, true);
        robot.waitFor(gateOpenDuration);
        */

        //go behind the gate to intake the balls
        //start farther from the gate
        //robot.intake.reqIntake(true);
        robot.drivetrain.goToPoint(new Pose2d(16, 59, Math.toRadians(140)), 1);
        robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, timeout);
        robot.waitFor(gateIntakeDuration);

        robot.drivetrain.goToPoint(new Pose2d(12, 59, Math.toRadians(80)), 1, true);
        robot.waitWhileWithTimeout(() -> robot.drivetrain.state != Drivetrain.State.WAIT, timeout);
        robot.intake.reqOff(true);

        robot.drivetrain.goToPoint(new Pose2d(-6, 10, Math.toRadians(80)), 1, false);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        if (!skipLast) {
            robot.drivetrain.goToPoint(new Pose2d(15, 36, Math.PI / 2), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }
    }
}
