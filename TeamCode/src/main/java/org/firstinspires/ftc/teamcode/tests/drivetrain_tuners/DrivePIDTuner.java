package org.firstinspires.ftc.teamcode.tests.drivetrain_tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@TeleOp
@Config
public class DrivePIDTuner extends LinearOpMode {
    private Robot robot;
    public static double targetX = 0.0, targetY = 0.0, targetH = 0.0;
    public static boolean updateNewPoint = false;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);

        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
        while(opModeInInit()){
            robot.update();
        }

        while(!isStopRequested()){
            robot.update();

            if(updateNewPoint){
                robot.drivetrain.goToPoint(new Pose2d(targetX, targetY, Math.toRadians(targetH)), 1.0);
                updateNewPoint = false;
            }
        }
    }
}
