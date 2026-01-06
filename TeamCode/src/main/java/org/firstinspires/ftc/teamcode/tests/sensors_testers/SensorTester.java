package org.firstinspires.ftc.teamcode.tests.sensors_testers;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.Locale;

@TeleOp(group = "Test")
@Config
public class SensorTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.RUNMODE = RunMode.TESTER;
        Globals.TESTING_DISABLE_CONTROL = true;

        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;

        //robot.update();
        waitForStart();

        while (!isStopRequested()) {
            robot.sensors.update();

            Pose2d pos = ROBOT_POSITION;
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(), pos.getY(), pos.getHeading());

            if (gamepad1.left_stick_button) {
                robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
            }

            telemetry.addData("Robot Position", data);
            telemetry.addData("Flywheel Encoder", robot.drivetrain.rightRear.motor[0].getCurrentPosition());
            telemetry.addData("Turret Encoder", robot.sensors.turretEncoder.getVoltage());
            telemetry.update();
        }
    }
}
