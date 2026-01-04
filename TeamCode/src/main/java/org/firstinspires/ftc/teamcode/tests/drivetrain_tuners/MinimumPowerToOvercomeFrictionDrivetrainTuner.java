package org.firstinspires.ftc.teamcode.tests.drivetrain_tuners;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.ArrayList;

@Autonomous
public class MinimumPowerToOvercomeFrictionDrivetrainTuner extends LinearOpMode {

    double[] sums = new double[4];
    int iterations = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Sensors sensors = robot.sensors;
        Localizer localizer = new Localizer(sensors, robot.drivetrain, "ff00ff", "ffff00");
        HardwareQueue hardwareQueue = robot.hardwareQueue;

        ArrayList<PriorityMotor> motors = new ArrayList<>();

        double[] minPowersToOvercomeFriction = new double[4];

        motors.add(robot.drivetrain.leftFront);
        motors.add(robot.drivetrain.leftRear);
        motors.add(robot.drivetrain.rightFront);
        motors.add(robot.drivetrain.rightRear);

        Pose2d robotPose;
        robot.drivetrain.resetMinPowersToOvercomeFriction();

        robot.shooter.state = Shooter.State.TEST;

        waitForStart();

        for (int i = 0; i < 4; i++) {

            for (int a = 0; a < iterations; a++) {
                long start = System.currentTimeMillis();
                for (double j = 0; j < 1; j = (double) (System.currentTimeMillis() - start) / (15000.0)) {
                    Globals.START_LOOP();
                    robot.update();
                    TelemetryUtil.sendTelemetry();

                    motors.get(i).setTargetPower(j);

                    robotPose = robot.drivetrain.getPoseEstimate();
                    if (Math.abs(robotPose.x) > 0.1 || Math.abs(robotPose.y) > 0.1 || Math.abs(robotPose.heading) > Math.toRadians(7)) {
                        minPowersToOvercomeFriction[i] = j;
                        break;
                    }
                    telemetry.addData(motors.get(i).name + " current power: ", j);
                    telemetry.update();
                }

                motors.get(i).setTargetPower(0.0);

                sums[i] += minPowersToOvercomeFriction[i] * (12/sensors.getVoltage());

                long waitStart = System.currentTimeMillis();
                while (System.currentTimeMillis() - waitStart < 1000) {
                    robot.update();
                }
            }

            Log.e(motors.get(i).name + " AVERAGE min power with voltage correction", sums[i]/iterations + "");

        }
    }
}
