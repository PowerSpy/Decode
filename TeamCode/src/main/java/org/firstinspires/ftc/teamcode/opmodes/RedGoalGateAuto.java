package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@Autonomous(name = "* Red Goal Gate Auto", group = "Auto", preselectTeleOp = "A. Teleop")
public class RedGoalGateAuto extends LinearOpMode {
    private Robot robot;
    public static long shootDuration = 700, intakeDuration = 1000, gateIntakeDuration = 1000, gateOpenDuration = 700;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;
        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.drivetrain.setPoseEstimate(new Pose2d(-71 + ROBOT_BACK_LENGTH, 24.25 + ROBOT_WIDTH / 2, 0));

        robot.shooter.state = Shooter.State.IDLE;
        robot.shooter.setShooterBlocker(true);
        robot.shooter.autoToggle();
        //robot.shooter.turretTrackInManual = true;

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
        robot.shooter.reqAim(true);
        //robot.drivetrain.goToPoint(new Pose2d(-18, 18, 0), 1, true);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        //preloads
        shoot(Math.toRadians(45), 1);
        intake(11, 60, true, false);
        //middle spikemarks
        shoot(Math.toRadians(90), 1);
        //gate intake and shoot
        gate_intake(true);
        shoot(Math.toRadians(90), 1);
        //close spikemarks
        intake(-13, 53, true, false);
        shoot(Math.toRadians(90), 2);
        //robot.shooter.setShooter(Shooter.Dist.CLOSE);
        //robot.drivetrain.goToPoint(new Pose2d(18, 22, Math.PI), 1.0, true);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        //far spikemarks
        intake(35, 60, true, false);
        shoot(Math.toRadians(90), 1);

        robot.shooter.reqStop(true);
        robot.shooter.turret.setTargetAngle(0.0);
        robot.drivetrain.goToPoint(new Pose2d(0, 40, Math.PI / 2), 1.0);

        long x = System.currentTimeMillis() - t;

        TelemetryUtil.packet.put("Time : ", x);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            robot.shooter.flywheel.setTargetVelocity(0);
            robot.shooter.autoToggle();
            return true;
        });
    }

    private void shoot(double heading, int shotType) {
        /*if (shotType > 0) {
            robot.drivetrain.goToPoint(new Pose2d(shotType == 2 ? -32 : -13, 20, heading), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        } else {
            robot.drivetrain.goToPoint(new Pose2d(-18, 18, 0), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }*/

        robot.drivetrain.goToPoint(new Pose2d(shotType == 1 ? -10 : -18, shotType == 1 ? 10 : 18, heading), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT || robot.shooter.state != Shooter.State.READY || !robot.shooter.turret.inPosition());
        robot.waitFor(200);

        robot.shooter.reqShoot(true);


        robot.waitFor(shootDuration);

        robot.shooter.reqStop(true);
        //robot.shooter.reqStop(true);
        //if (recharge) robot.shooter.reqAim(true);
    }

    private void intake(double x, double y, boolean skipLast, boolean straight) {

        if (straight) {
            robot.drivetrain.goToPoint(new Pose2d(x - 15, 20, 0), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }
        // align
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        robot.intake.reqIntake(true);


        // intake
        robot.drivetrain.goToPoint(new Pose2d(x, y, Math.PI / 2), 1);
        robot.waitFor(intakeDuration);

        // back off

        if(skipLast != true) {
            robot.drivetrain.goToPoint(new Pose2d(x, 40, Math.PI / 2), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }

        robot.intake.reqOff(true);

        robot.shooter.reqAim(true);
    }

    private void gate_intake(boolean skipLast) {



        // align
        robot.drivetrain.goToPoint(new Pose2d(4, 22, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);
        // hit gate
        robot.drivetrain.goToPoint(new Pose2d(4, 54, Math.PI / 2), 1);
        //robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.waitFor(gateOpenDuration);

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
        robot.drivetrain.goToPoint(new Pose2d(15, 59, Math.toRadians(135)), 1);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.waitFor(gateIntakeDuration);

        if(skipLast != true) {

            robot.drivetrain.goToPoint(new Pose2d(15, 36, Math.PI / 2), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }

        robot.intake.reqOff(true);

        robot.shooter.reqAim(true);
    }
}
