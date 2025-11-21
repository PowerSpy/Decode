package org.firstinspires.ftc.teamcode.tests.sensors_testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
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

        robot.update();

        waitForStart();

        while (!isStopRequested()) {
            robot.sensors.update();

            Pose2d pos = sensors.getOdometryPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(), pos.getY(), pos.getHeading());

            if (gamepad1.left_stick_button) {
                robot.drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
            }

            telemetry.addData("Robot Position", data);
            //telemetry.addData("Limelight connection", robot.vision.isConnected() ? "everything is fine" : "freaking packet yo");
            telemetry.addData("flywheel encoder", robot.shooter.flywheel.motor[0].getCurrentPosition());
            telemetry.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
