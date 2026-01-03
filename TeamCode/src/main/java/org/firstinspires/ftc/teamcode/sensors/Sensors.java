package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Sensors {
    private final HardwareMap hardwareMap;

    public double loopTime;
    private long currentTime, lastTime;

    private final int[] odoWheelPositions = {0, 0, 0};

    // Enocder Resolution: 28 PPR
    private double flywheelAngularVel = 0, flywheelVelocity = 0;

    private double voltage;
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    public Sensors(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        currentTime = System.nanoTime();
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update() {
        lastTime = currentTime;
        currentTime = System.nanoTime();
        loopTime = (currentTime - lastTime) / 1e9;

        odoWheelPositions[0] = ((PriorityMotor) hardwareMap.get(DcMotorEx.class, "leftFront")).motor[0].getCurrentPosition();
        odoWheelPositions[1] = ((PriorityMotor) hardwareMap.get(DcMotorEx.class, "rightFront")).motor[0].getCurrentPosition();
        odoWheelPositions[2] = ((PriorityMotor) hardwareMap.get(DcMotorEx.class, "leftRear")).motor[0].getCurrentPosition();

        double flywheelPos = ((PriorityMotor) hardwareMap.get(DcMotorEx.class, "rightRear")).motor[0].getCurrentPosition();

        // (flywheelPos - flywheelLastPos) / 28.0 = delta revolutions
        flywheelAngularVel = ((PriorityMotor) hardwareMap.get(DcMotorEx.class, "rightRear")).motor[0].getVelocity() / 28.0;
        flywheelVelocity = flywheelAngularVel * 96.0 * Math.PI / 25.4;



        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis();
        }

        updateTelemetry();
    }

    // Odometry
    public int[] getOdometry() {return odoWheelPositions;}

    public double getFlywheelVelocity() { return flywheelVelocity; }

    public double getVoltage() {
        return voltage;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Voltage", voltage);

        Pose2d currentPose = Globals.ROBOT_POSITION;
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPose, "#ff0000");

        LogUtil.flywheelVelocity.set(flywheelVelocity);
        LogUtil.driveCurrentX.set(currentPose.x);
        LogUtil.driveCurrentY.set(currentPose.y);
        LogUtil.driveCurrentAngle.set(currentPose.heading);
    }
}
