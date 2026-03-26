package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.vision.BallDetection;

    import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name = "BallDetectionAutoTest", group = "Test")
public class BallDetectionAutoTest extends LinearOpMode {
    private Robot robot;
    BallDetection b;


    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;

        robot = new Robot(hardwareMap);
        b = new BallDetection(hardwareMap);
        b.start();


        robot.setStopChecker(this::isStopRequested);
        robot.intake.state = Intake.State.TEST;
        robot.shooter.state = Shooter.State.TEST;

        do {
            robot.drivetrain.setPoseEstimate(new Pose2d(48, 0, Math.PI / 2));
            robot.update();
        } while (opModeInInit());

        Vector2 ballPos;
        b.update();

        double t = System.currentTimeMillis();

        do {
            b.update();
            ballPos = b.getBestBall(b.getBallPoses());

        } while(System.currentTimeMillis() - t < 1000);


        Log.i("Ball Detection Auto Target", ballPos.toString());

        robot.drivetrain.goToPoint(new Pose2d(ballPos.x, ballPos.y - 10, Math.PI / 2), 1, false);

        robot.update();
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
}
