package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import android.util.Log;

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
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@Autonomous(name = "* Red Goal Sort Auto", group = "Auto", preselectTeleOp = "A. Teleop")
public class RedGoalSortAuto extends LinearOpMode {
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


        long t = System.currentTimeMillis();
        robot.shooter.setManual(false);

        shoot(Math.toRadians(180), 1, true, true);
        intake(11.5, 60, false, 1);

        open_gate(500);

        shoot(Math.toRadians(90), 1, true, false);
        intake(-13, 51, true, 2);

        shoot(Math.toRadians(80), 2, true, false);
        intake(36, 51, true, 2);

        shoot(Math.toRadians(80), 1, false, false);

        robot.shooter.setManual(true);

        robot.intake.setLeftBlocker(false);
        robot.intake.setRightBlocker(false);

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
                robot.drivetrain.goToPoint(new Pose2d(-12, 12, heading), 1, false);
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
        Log.i("Ball Pattern Int", String.valueOf(Globals.BALL_PATTERN));
        robot.shooter.reqShoot(true);


        robot.waitFor(shootDuration);


        robot.shooter.reqStop(true);
        if (first) {
            robot.drivetrain.goToPoint(new Pose2d(-6, 10, Math.toRadians(126)), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

            robot.drivetrain.goToPoint(new Pose2d(11.5, 20, Math.toRadians(90)), 1, false);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        } else {
            robot.waitFor(50);

        }
    }

    private void intake(double x, double y, boolean skipLast, int spike) {
        // align
        robot.drivetrain.goToPoint(new Pose2d(x, 22, Math.PI / 2), 1, true);
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        robot.intake.reqIntake(true);
        int[] pattern = new int[3];
        pattern[Globals.BALL_PATTERN - 21] = 1;
        if (spike != 0) {
            // sorted intake
            int[] balls = new int[3];
            balls[3 - spike] = 1; // tells you where the green ball is
            int[] slots = new int[]{-1, -1}; // [0] is left slot, [1] is right slot
            int slotsInUse = 0;
            for (int i = 0; i < 3; i++) {
                if (slots[0] == pattern[i - slotsInUse]) { // checks if the next ball we wanna intake is in left slot
                    // if yes, we intake it
                    robot.intake.setLeftBlocker(false);
                    robot.waitWhile(() -> !robot.intake.lindex.inPosition());
                    slots[0] = -1;
                    slotsInUse--;
                }
                if (slots[1] == pattern[i - slotsInUse]) { // checks if the next ball we want is in the right slot
                    // if yes, then we intake it
                    robot.intake.setRightBlocker(false);
                    robot.waitWhile(() -> !robot.intake.rindex.inPosition());
                    slots[1] = -1;
                    slotsInUse--;
                }
                if (balls[i] != pattern[i - slotsInUse]) {
                    // in the event that we don't have the ball we want
                    // in front of us or in our slots, we put the front ball in a slot
                    // it should be impossible for us to have all slots
                    // occupied & unwanted and also have the ball in front
                    // of us not be the one we want

                    // prioritizes the use of the right slot since that's less distance for red auto
                    int deltaX = 3 * (slots[1] == -1 ? -1 : 1);
                    if (deltaX < 0) robot.intake.setRightBlocker(true);
                    else robot.intake.setLeftBlocker(true);
                    slotsInUse++;
                    slots[(deltaX < 0 ? 1 : 0)] = balls[i];
                    robot.drivetrain.goToPoint(new Pose2d(x + deltaX, y + i * 5, Math.PI / 2), 1);
                    robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
                } else {
                    robot.drivetrain.goToPoint(new Pose2d(x, y + i * 5, Math.PI / 2), 1);
                    robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
                }

                // but also to deal with the case where we still have slotted balls after the 3rd ball intaken,
                // we'll just check again if the balls we want are in the slots after slotting or intaking!
                // this is the jank solution
                if (slots[0] == pattern[i - slotsInUse]) { // checks if the next ball we wanna intake is in left slot
                    // if yes, we intake it
                    robot.intake.setLeftBlocker(false);
                    robot.waitWhile(() -> !robot.intake.lindex.inPosition());
                    slots[0] = -1;
                    slotsInUse--;
                }
                if (slots[1] == pattern[i - slotsInUse]) { // checks if the next ball we want is in the right slot
                    // if yes, then we intake it
                    robot.intake.setRightBlocker(false);
                    robot.waitWhile(() -> !robot.intake.rindex.inPosition());
                    slots[1] = -1;
                    slotsInUse--;
                }
            }
        } else {
            // regular intake

            robot.drivetrain.goToPoint(new Pose2d(x, 20, Math.PI / 2), 1);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

            // why do we have 2 extra go to's?? -Nikhil
            robot.drivetrain.goToPoint(new Pose2d(x, 50, Math.PI / 2), 1);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);


            robot.drivetrain.goToPoint(new Pose2d(x, 20, Math.PI / 2), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        }

        // back off
        if (!skipLast) {
            robot.drivetrain.goToPoint(new Pose2d(x, 30, Math.PI / 2), 1, true);
            robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);
        }
        robot.intake.reqOff(true);
    }
}