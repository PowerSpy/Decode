package org.firstinspires.ftc.teamcode.tests.localization_testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.drive.RepulsionPoint;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

import java.util.ArrayList;

public class GVFTester extends LinearOpMode{
    Robot robot;
    public static double x1 = 12.0,y1= 12.0,h1=Math.PI/2,x2 = 36,y2 = 24,h2 = Math.PI/4;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        robot.drivetrain.setPoseEstimate(new Pose2d (0, 0, 0));

        ArrayList<RepulsionPoint> repel = new ArrayList<>();
        repel.add(new RepulsionPoint(-24, 0, 1.0));
        repel.add(new RepulsionPoint(-24, -48, 1.0));
        repel.add(new RepulsionPoint(-24, 48, 1.0));

        Path testPath = new Path(new Pose2d(0, 0, 0), repel);
        testPath.addPoint(new Pose2d(24, 24, Math.PI / 2));
        testPath.addPoint(new Pose2d(48, 24, Math.PI * 3 / 2.0));
        testPath.addPoint(new Pose2d(24, 0, Math.PI));
        testPath.setReversed(true);
        testPath.addPoint(new Pose2d(0, 0, 0));

        waitForStart();

        while(opModeInInit()){
            robot.update();
        }

        robot.drivetrain.setPath(testPath);

        while(!isStopRequested()){
            robot.update();
        }
    }
}
