package org.firstinspires.ftc.teamcode.tests.localization_testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.drive.RepulsionPoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

import java.util.ArrayList;

@Autonomous
public class GVFTester extends LinearOpMode{
    private Robot robot;

    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.shooter.state = Shooter.State.TEST;

        robot.drivetrain.setPoseEstimate(new Pose2d (0, 0, 0));

        ArrayList<RepulsionPoint> repel = new ArrayList<>();
        repel.add(new RepulsionPoint(24, 54, 8));
        repel.add(new RepulsionPoint(0, 54, 8));
        repel.add(new RepulsionPoint(-24, 54, 8));

        Path testPath = new Path(new Pose2d(0, 0, 0), repel)
                .addPoint(new Pose2d(0, 48, 0))
                .setDecel(true);

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
