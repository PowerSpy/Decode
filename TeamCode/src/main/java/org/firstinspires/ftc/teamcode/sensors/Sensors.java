package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LEDWrapper;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.RelativeEncoder;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.List;

@Config
public class Sensors {
    private final Robot robot;
    private final List<LynxModule> allHubs;

    public double loopTime;
    private long currentTime, lastTime, initialTime;

    private final int[] odoWheelPositions = {0, 0, 0};

    // Enocder Resolution: 28 PPR
    private double flywheelVelocity = 0;

    public AnalogInput turretAnalogEncoder;
    private double turretAngle;
    private double turretAngleEncoderOffset, turretAngleEncoderPosition;
    public static double turretAnalogEncoderOffsetDeg = 172;
    public static double turretAngleFilter = 0.6;
    public static double turretLimitLeft = Math.toRadians(330), turretLimitRight = Math.toRadians(-20), turretWrapMid = Math.toRadians(155);
    public static boolean resetTurretAngleEncoder = true;
    private double turretAnalogEncoderVoltage;

    private double lightSensorFilteredVoltage = 0;
    public static double lightSensorFilter = 0.5;
    public AnalogInput lightSensor0;
    public final LEDWrapper light0G, light0P;
    private boolean isGreen = false, isPurple = false;

    private double voltage;
    public static long voltageUpdateTime = 5000, colorSensorUpdateTime = 250;
    private long lastVoltageUpdatedTime = 0;
    private long lastColorSensorUpdatedTime = 0;
    private final VoltageSensor voltageSensor;

    public Sensors(Robot robot) {
        this.robot = robot;

        initialTime = currentTime = System.nanoTime();
        voltageSensor = robot.hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();

        turretAnalogEncoder = robot.hardwareMap.get(AnalogInput.class, "turret_encoder");
        turretAngleEncoderOffset = turretAngleEncoderPosition = turretAngle = 0;
        resetTurretAngleEncoder = true;

        lightSensor0 = robot.hardwareMap.get(AnalogInput.class, "lightSensor0");
        light0G = new LEDWrapper(robot.hardwareMap, "light0G");
        light0P = new LEDWrapper(robot.hardwareMap, "light0P");

        allHubs = robot.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void update() {
        lastTime = currentTime;
        currentTime = System.nanoTime();
        loopTime = (currentTime - lastTime) / 1e9;

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        light0G.update();
        light0P.update();

        odoWheelPositions[0] = robot.drivetrain.leftFront.motor[0].getCurrentPosition(); // left
        odoWheelPositions[1] = robot.drivetrain.rightFront.motor[0].getCurrentPosition(); // right
        odoWheelPositions[2] = robot.drivetrain.leftRear.motor[0].getCurrentPosition(); // back

        double flywheelAngularVel = robot.drivetrain.rightRear.motor[0].getVelocity() / 28.0;
        flywheelVelocity = flywheelAngularVel * 3.0 * Math.PI;

        robot.drivetrain.localizer.updateEncoders(odoWheelPositions);
        robot.drivetrain.localizer.update();
        robot.drivetrain.mergeLocalizer.updateEncoders(odoWheelPositions);
        robot.drivetrain.mergeLocalizer.update();

        ROBOT_POSITION = robot.drivetrain.mergeLocalizer.getPoseEstimate();
        ROBOT_VELOCITY = robot.drivetrain.mergeLocalizer.getRelativePoseVelocity();
        ROBOT_GLOBAL_VELOCITY = robot.drivetrain.mergeLocalizer.getGlobalVelocity();

        if (currentTime - initialTime < 200_000_000) resetTurretAngleEncoder = true;
        turretAngleEncoderPosition = robot.intake.feed.motor[0].getCurrentPosition() * (Math.PI) / -8192 / 2;
        double newTurretAngle = (turretAngleEncoderPosition - turretAngleEncoderOffset) % (2 * Math.PI);
        if (resetTurretAngleEncoder) {
            turretAnalogEncoderVoltage = turretAnalogEncoder.getVoltage();
            if (turretAnalogEncoderVoltage > 0.1) {
                newTurretAngle = Utils.headingClip(RelativeEncoder.normalizeVoltage(turretAnalogEncoderVoltage) - Math.toRadians(turretAnalogEncoderOffsetDeg) - turretWrapMid) + turretWrapMid;
                turretAngleEncoderOffset = turretAngleEncoderPosition - newTurretAngle;
                resetTurretAngleEncoder = false;
            }
        }
        turretAngle = turretAngle * (1 - turretAngleFilter) + newTurretAngle * turretAngleFilter;

        if (Globals.RUNMODE != RunMode.AUTO && currentTime - lastColorSensorUpdatedTime > colorSensorUpdateTime * 1e6) {
            double lightSensorRawVoltage = lightSensor0.getVoltage();
            lightSensorFilteredVoltage = lightSensorFilteredVoltage * (1 - lightSensorFilter) + lightSensorRawVoltage * lightSensorFilter;
            isGreen = lightSensorFilteredVoltage > 0.01;
            isPurple = !isGreen && lightSensorFilteredVoltage > 0.005;
            light0G.set(isGreen);
            light0P.set(isPurple);
            TelemetryUtil.packet.put("Intake : Light Raw Voltage", lightSensorRawVoltage);
            TelemetryUtil.packet.put("Intake : Light Filtered Voltage", lightSensorFilteredVoltage);
            //TelemetryUtil.packet.put("Intake : Light Voltage Green Thresh", 0.009);
            //TelemetryUtil.packet.put("Intake : Light Voltage Purple Thresh", 0.004);
            lastColorSensorUpdatedTime = currentTime;
        }

        if (currentTime - lastVoltageUpdatedTime > voltageUpdateTime * 1e6) {
            voltage = voltageSensor.getVoltage();
            lastVoltageUpdatedTime = currentTime;
        }

        updateTelemetry();
    }

    public double getFlywheelVelocity() { return flywheelVelocity; }

    public double getVoltage() {
        return voltage;
    }

    /**
     * Clips a turret angle
     * @param angle a robot-relative angle
     * @return the wrapped and clipped turret angle
     */
    public static double turretAngleClip(double angle) { return Utils.minMaxClip(Utils.headingClip(angle - turretWrapMid) + turretWrapMid, turretLimitRight, turretLimitLeft); }

    /**
     * Gets the turret angle
     * @return the wrapped turret angle
     */
    public double getTurretAngle() { return turretAngle; }

    //position of the park slides

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Sensors: Voltage", voltage);

        TelemetryUtil.packet.put("Sensors : Flywheel Velocity (in/s)", flywheelVelocity);
        TelemetryUtil.packet.put("Sensors : Turret Angle (deg)", Math.toDegrees(turretAngle));
        TelemetryUtil.packet.put("Sensors : Hood Angle (deg)", Math.toDegrees(robot.shooter.hood.getCurrentAngle() / Shooter.hoodGearRatio + Shooter.hoodSweep));

        TelemetryUtil.packet.put("Sensors : Ball Color", isPurple ? "purple" : isGreen ? "green" : "none");

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, ROBOT_POSITION, "#00ff00", turretAngle, "#00e000c0", robot.shooter.turret.getTargetAngle(), "#8000ff");

        LogUtil.turretAngle.set(turretAngle);
        LogUtil.flywheelVelocity.set(flywheelVelocity);
        LogUtil.driveCurrentX.set(ROBOT_POSITION.x);
        LogUtil.driveCurrentY.set(ROBOT_POSITION.y);
        LogUtil.driveCurrentAngle.set(ROBOT_POSITION.heading);
    }
}
