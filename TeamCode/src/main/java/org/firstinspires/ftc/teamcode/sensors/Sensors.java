package org.firstinspires.ftc.teamcode.sensors;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.ArrayList;
import java.util.Queue;

@Config
public class Sensors {
    private final Robot robot;
    private GoBildaPinpointDriver odometry;

    private Pose2d currentPose = new Pose2d(0, 0, 0), lastPose;
    private long currentTime, lastTime = System.currentTimeMillis();
    public double loopTime;
    private Vector2 vel = new Vector2();
    public int[] odoWheelPositions = {0, 0, 0};

    // 426 mm (16.772 in) is belt circumference
    // start: 42638
    // end: 43062
    private double flywheelVelocity, ticksToInches = 16.772 / (43062 - 42638), flywheelVelo2, flywheelLastPos = 0;

    /*
    Index Key
    0 -> intake side
    2 -> shooter side

    Color Key
    0 -> no ball
    1 -> green ball
    2 -> purple ball
     */
    private ArrayList<Integer> balls;
    private int[] colors = {0, 0, 0};
    private REVColorSensorV3 colorSensorV3;
    private boolean colorToggle = false, fullChamber = false;

    private double voltage;
    private final double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    public double[] analogVoltages = new double[analogEncoders.length];

    public Sensors(Robot robot) {
        this.robot = robot;
        currentTime = System.nanoTime();

        odometry = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.setOffsets(72, -160);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
        colorSensorV3 = robot.hardwareMap.get(REVColorSensorV3.class, "indexSensor");
        colorSensorV3.configureLS(REVColorSensorV3.LSResolution.SIXTEEN, REVColorSensorV3.LSMeasureRate.m25s, REVColorSensorV3.LSGain.THREE);
        colorSensorV3.sendControlRequest(new REVColorSensorV3.ControlRequest()
                .enableFlag(REVColorSensorV3.ControlFlag.LIGHT_SENSOR_ENABLED)
                .enableFlag(REVColorSensorV3.ControlFlag.RGB_ENABLED));
        */

        balls = new ArrayList<Integer>();
        balls.add(0);
        balls.add(0);
        balls.add(0);

        voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update() {
        lastTime = currentTime;
        currentTime = System.nanoTime();
        loopTime = (currentTime - lastTime) / 1e9;

        odoWheelPositions[0] = robot.drivetrain.leftRear.motor[0].getCurrentPosition(); // left
        odoWheelPositions[1] = robot.drivetrain.leftFront.motor[0].getCurrentPosition(); // right
        odoWheelPositions[2] = robot.drivetrain.rightFront.motor[0].getCurrentPosition(); // back
        robot.drivetrain.updateLocalizer();

        //odometry.update();

        lastPose = currentPose.clone();
        currentPose = robot.drivetrain.localizers[0].getPoseEstimate();

        vel.x = (currentPose.x - lastPose.x) / loopTime;
        vel.y = (currentPose.y - lastPose.y) / loopTime;
        stopConfidence();

        double flywheelPos = robot.drivetrain.rightRear.motor[0].getCurrentPosition();
        flywheelVelocity = robot.drivetrain.rightRear.getVelocity() * ticksToInches;
        flywheelVelo2 = (flywheelPos - flywheelLastPos) / loopTime * ticksToInches;
        flywheelLastPos = flywheelPos;

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }

        updateTelemetry();
    }

    // Odometry

    public Pose2d getOdometryPosition() { return currentPose; }

    public double getHeading() { return currentPose.heading; }

    public Vector2 getVelocity() { return vel; }

    boolean isStopped = true;
    double confidence = 0.0;
    final double confidenceAlpha = 0.125, confidenceThresh = 0.75;

    public void stopConfidence() {
        confidence *= (1 - confidenceAlpha);

        if (vel.mag() < 0.5) {
            confidence += confidenceAlpha;
        }

        isStopped = confidence >= confidenceThresh;
    }

    public boolean stopped() {return isStopped;}

    public double getFlywheelVelocity () { return flywheelVelocity; }

    public double getVoltage() {
        return voltage;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Voltage", voltage);
        TelemetryUtil.packet.put("Shooter : Flywheel velocity", flywheelVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel velocity 2", flywheelVelo2);

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPose, "#00ff00");

        LogUtil.driveCurrentX.set(currentPose.x);
        LogUtil.driveCurrentY.set(currentPose.y);
        LogUtil.driveCurrentAngle.set(currentPose.heading);
        LogUtil.flywheelVelocity.set(flywheelVelocity);
    }
}
