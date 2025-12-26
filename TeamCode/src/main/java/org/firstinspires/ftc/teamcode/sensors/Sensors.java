package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

@Config
public class Sensors {
    private final Robot robot;
    public double loopTime;
    private long currentTime, lastTime;

    private final int[] odoWheelPositions = {0, 0, 0};

    // Enocder Resolution: 28 PPR
    private double flywheelAngularVel = 0, flywheelVelocity = 0;

    private double voltage;
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    public Sensors(Robot robot) {
        this.robot = robot;
        currentTime = System.nanoTime();
        voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update() {
        lastTime = currentTime;
        currentTime = System.nanoTime();
        loopTime = (currentTime - lastTime) / 1e9;

        odoWheelPositions[0] = robot.drivetrain.leftFront.motor[0].getCurrentPosition(); // left
        odoWheelPositions[1] = robot.drivetrain.rightFront.motor[0].getCurrentPosition(); // right
        odoWheelPositions[2] = robot.drivetrain.leftRear.motor[0].getCurrentPosition(); // back

        double flywheelPos = robot.drivetrain.rightRear.motor[0].getCurrentPosition();

        // (flywheelPos - flywheelLastPos) / 28.0 = delta revolutions
        flywheelAngularVel = robot.drivetrain.rightRear.getVelocity() / 28.0;
        flywheelVelocity = flywheelAngularVel * 96.0 * Math.PI / 25.4;

        robot.drivetrain.localizer.updateEncoders(odoWheelPositions);
        robot.drivetrain.localizer.update();

        robot.drivetrain.mergeLocalizer.updateEncoders(odoWheelPositions);
        robot.drivetrain.mergeLocalizer.update();

        ROBOT_POSITION = robot.drivetrain.mergeLocalizer.getPoseEstimate();
        ROBOT_POSITION.heading = AngleUtil.clipAngle(ROBOT_POSITION.heading);

        ROBOT_VELOCITY = robot.drivetrain.mergeLocalizer.getRelativePoseVelocity();

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis();
        }

        updateTelemetry();
    }

    // Odometry
    public int[] getOdometry() {return odoWheelPositions;}

    /**
     *
     * @return radians / sec
     */
    public double getFlywheelAngularVel () { return flywheelAngularVel;}

    public double getFlywheelVelocity() { return flywheelVelocity; }

    public double getVoltage() {
        return voltage;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Voltage", voltage);
        TelemetryUtil.packet.put("Shooter : Flywheel Angular Velocity", flywheelAngularVel);
        TelemetryUtil.packet.put("Shooter : Flywheel Current Velocity", flywheelVelocity);
        TelemetryUtil.packet.put("Shooter : Hood top angle (deg)", Math.toDegrees(robot.shooter.hood.getCurrentAngle()) * 30 / 48 + 34);

        Pose2d currentPose = Globals.ROBOT_POSITION;
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPose, "#00ff00");

        LogUtil.flywheelVelocity.set(flywheelVelocity);
        LogUtil.driveCurrentX.set(currentPose.x);
        LogUtil.driveCurrentY.set(currentPose.y);
        LogUtil.driveCurrentAngle.set(currentPose.heading);
    }
}
