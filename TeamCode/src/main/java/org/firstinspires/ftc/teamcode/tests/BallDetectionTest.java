package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.vision.BallDetection;

import java.util.ArrayList;

@TeleOp
public class BallDetectionTest extends LinearOpMode {
    BallDetection b;

    private ArrayList<Vector2> p = new ArrayList<>();

    public void runOpMode() {
        TelemetryUtil.setup();

        b = new BallDetection(hardwareMap);
        b.start();

        while (!isStopRequested()) {
            b.update();
            p = b.getBallPoses();
            telemetry.addData("Ball Poses", p);
            telemetry.update();

            Canvas canvas = TelemetryUtil.packet.fieldOverlay();
            canvas.setStroke("#ff4000"); // Todo - Need to change the color based on what color the ball is
            canvas.setStrokeWidth(2);

            for (int i = 0; i < p.size(); i++) {
                canvas.strokeCircle(p.get(i).x, p.get(i).y, 5);
            }

            TelemetryUtil.sendTelemetry();
        }
    }
}
