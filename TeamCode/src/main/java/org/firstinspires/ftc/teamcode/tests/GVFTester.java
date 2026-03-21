package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_BACK_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

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

@Autonomous(name = "GVF Tester", group = "Test")
public class GVFTester extends LinearOpMode {
    private Robot robot;

    public void runOpMode() {
        Globals.isRed = true;
        Globals.RUNMODE = RunMode.AUTO;

        robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);
        robot.intake.state = Intake.State.TEST;
        robot.shooter.state = Shooter.State.TEST;

        do {
            robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
            robot.update();
        } while (opModeInInit());

        Path path = new Path(Globals.ROBOT_POSITION.clone())
                .addPoint(new Pose2d(48, 0, 0), false, true);
        robot.drivetrain.setPath(path);
        robot.update();
        Log.i("GVF", robot.drivetrain.getCurrentPathTarget().toString());
        robot.waitWhile(() -> robot.drivetrain.state != Drivetrain.State.WAIT);

        Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
        robot.waitWhile(() -> {
            Globals.AUTO_ENDING_POSE = Globals.ROBOT_POSITION.clone();
            return true;
        });
    }
}
