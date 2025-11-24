package org.firstinspires.ftc.teamcode.opmodes;

import static org.checkerframework.checker.units.UnitsTools.m;
import static org.firstinspires.ftc.teamcode.utils.Globals.AUTO_ENDING_POSE;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@Autonomous(name = "TestDrivetrainAuto", preselectTeleOp = "A. Teleop")
public class TestDrivetrainAuto extends LinearOpMode {
    Robot robot;
    long shooterTimer;

    @Override
    public void runOpMode(){
        robot = new Robot(hardwareMap);
        robot.intake.state = Intake.State.TEST;
        robot.setStopChecker(this::isStopRequested);

        //zero the current pose
        robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));

        //ROBOT_POSITION = robot.sensors.getOdometryPosition();

        while(opModeInInit()){
            //TelemetryUtil.packet.put("X-Pos:  ", ROBOT_POSITION.x);
            robot.update();
        }

        ElapsedTime ElapsedTime = new ElapsedTime();

        ElapsedTime.reset();
        // move forward until reachedPosition is true
        robot.drivetrain.setMoveVector(new Vector2(0,1), 0);
        robot.waitWhile(this::reachedPosition);

        // move 36 inches forward
        //robot.drivetrain.goToPoint(new Pose2d(36, 0, 0), 1);

        // get time in miliseconds
        double time = ElapsedTime.milliseconds();

        time = time / 1000;
        TelemetryUtil.packet.put("Time to move 36 in left is: ", time);

        robot.update();

        telemetry.addData("Time to move 36 in: ", String.valueOf(time));

        telemetry.update();

        robot.waitWhile(this::isStopRequested);
    }

    public boolean reachedPosition() {
        return ROBOT_POSITION.y < 36;
    }
}
