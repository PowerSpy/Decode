package org.firstinspires.ftc.teamcode.tests;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.vision.BallDetection;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

@TeleOp(group = "Test")
public class BallDetectionTest extends LinearOpMode {
    BallDetection b;

    private ArrayList<Vector2> p = new ArrayList<>();

    private double[] w;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        robot.drivetrain.setPoseEstimate(new Pose2d(48,0,Math.PI / 2));
        robot.shooter.setManual(true);

        b = new BallDetection(hardwareMap);
        b.start();

        while (!isStopRequested()) {
            robot.update();

            b.update();
            p = b.getBallPoses();
            //w = b.getWeights(p);
            telemetry.addData("Ball Poses", p);
            telemetry.update();
            Log.i("Number of BallPoses", String.valueOf(p.size()));

            Canvas canvas = TelemetryUtil.packet.fieldOverlay();
            canvas.setStroke("#ff4000"); // Todo - Need to change the color based on what color the ball is
            canvas.setStrokeWidth(2);

            for (int i = 0; i < p.size(); i++) {
                //the bigger the radius of the circle, the higher the weight
                canvas.strokeCircle(p.get(i).x, p.get(i).y, 5);
            }
            canvas.setStroke("#000000");
            if(b.getBestBall(p) != null) {
                canvas.strokeCircle(b.getBestBall(p).x, b.getBestBall(p).y, 5);
                Log.i("Ball Detection", "Best Ball is" + b.getBestBall(p).toString());
            }
        }
    }
}
