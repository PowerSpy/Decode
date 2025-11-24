package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@TeleOp(name = "A. DriveTrainTestTeleop")
public class DriveTrainTestTeleop extends LinearOpMode {


    public void runOpMode() {
        Globals.RUNMODE = RunMode.TELEOP;
        Robot robot = new Robot(hardwareMap);
        robot.setStopChecker(this::isStopRequested);

        //robot.drivetrain.setPoseEstimate(new Pose2d(0 , 0, 0));

        ButtonToggle lb1 = new ButtonToggle();
        ButtonToggle rb1 = new ButtonToggle();
        ButtonToggle a1 = new ButtonToggle();
        ButtonToggle b1 = new ButtonToggle();
        ButtonToggle x1 = new ButtonToggle();
        ButtonToggle y1 = new ButtonToggle();

        ButtonToggle a2 = new ButtonToggle();
        ButtonToggle b2 = new ButtonToggle();
        ButtonToggle x2 = new ButtonToggle();
        ButtonToggle y2 = new ButtonToggle();

        boolean intakeReversed = false;
        boolean intakeOn = false;
        boolean flywheelOn = false;

        //when u detect a movement in the joystick start timer
        //once pose is greater 36 stop the timer
        robot.intake.state = Intake.State.TEST;

        while (opModeInInit()) robot.update();

        if (!isStopRequested()) LogUtil.init();

        LogUtil.drivePositionReset = true;
        // robot.shooter.goalDetector.start();

        ElapsedTime ElapsedTime = new ElapsedTime();
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        while (!isStopRequested()) {
            robot.update();
            robot.drivetrain.drive(gamepad1);



            if(rb1.isClicked(gamepad1.right_bumper)) {

                ElapsedTime.reset();

                robot.drivetrain.setPoseEstimate(new Pose2d(0 , 0, 0));


            }

            if(ROBOT_POSITION.y > 36) {

                double time = ElapsedTime.milliseconds();

                robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

                TelemetryUtil.packet.put("Time to move 36 in left is: ", time / 1000);
                telemetry.addData("Time: ", time / 1000);
                telemetry.update();



            }


        }


    }
}
