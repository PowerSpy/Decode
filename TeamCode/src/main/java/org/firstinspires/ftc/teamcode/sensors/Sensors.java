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
        odometry.setOffsets(70, 65);
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

        odometry.update();

        lastPose = currentPose.clone();
        currentPose = odometry.getPosition();

        vel.x = (currentPose.x - lastPose.x) / loopTime;
        vel.y = (currentPose.y - lastPose.y) / loopTime;
        stopConfidence();

        double flywheelPos = robot.shooter.flywheel.motor[0].getCurrentPosition();
        flywheelVelocity = robot.shooter.flywheel.getVelocity() * ticksToInches;
        flywheelVelo2 = (flywheelPos - flywheelLastPos) / loopTime * ticksToInches;
        flywheelLastPos = flywheelPos;

        /*
        if(colorToggle){
            colors = colorSensorV3.readLSRGBRAW();
        }
         */

        ballConfidence();
        if(!fullChamber){
            // so ugly but i dont want to write a loop :/
            balls.set(balls.get(2) == 0 ? 2 : (balls.get(1) == 0 ? 1 : 0), greenCounter == 3 ? 1 : (purpleCounter == 3 ? 1 : 0));
            // reset to check case of same color in a row
            greenCounter = greenCounter == 3 ? 0 : greenCounter;
            purpleCounter = purpleCounter == 3 ? 0 : purpleCounter;
            fullChamber = balls.get(0) != 0;
        }

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }

        updateTelemetry();
    }

    // Odometry

    public void resetPosAndIMU() {
        odometry.resetPosAndIMU();
    }

    public void recalibrate() { odometry.recalibrateIMU(); }

    public void setOdometryPosition(Pose2d pose2d) {
        odometry.setPosition(pose2d);
    }

    public Pose2d getOdometryPosition() { return currentPose; }

    public double getHeading() {
        return currentPose.heading;
    }

    public Vector2 getVelocity(){ return vel;}

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

    // Indexing

    // TODO: Probably need differnet alpha values, I'm thinking ~3 loops of detection should yield a positive
    double greenConfidence = 0.0, purpleConfidence = 0.0, greenCounter = 0.0, purpleCounter = 0.0;
    final double greenAlpha = 0.125, purpleAlpha = 0.125, greenThresh = 0.75, purpleThresh = 0.75;
    boolean isGreen = false, isPurple = false;

    public void ballConfidence() {
        greenConfidence *= (1 - greenAlpha);
        purpleConfidence *= (1 - purpleAlpha);

        // TODO: Tune these values to be accurate to balls
        // TODO: Consider integrating in ball-chasing camera detected values as a way to increase / decrease confidence
        if(colors[1] >= 192) {
            greenConfidence += greenConfidence;
        }

        if(colors[0] >= 128 && colors[2] >= 128) {
            purpleConfidence += purpleAlpha;
        }

        isGreen = greenConfidence >= greenThresh;
        isPurple = purpleConfidence >= purpleThresh;

        greenCounter = isGreen ? Math.min(greenCounter + 1, 3) : 0;
        purpleCounter = isPurple ? Math.min(purpleCounter + 1, 3) : 0;
    }

    public boolean isBall(){
        return greenConfidence >= greenThresh || purpleConfidence >= purpleThresh;
    }

    public void shot(){
        balls.set(2, balls.get(1));
        balls.set(1, balls.get(0));
        balls.set(0, 0);

        fullChamber = false;
    }

    public void toggleColor (boolean on) {colorToggle = on;}

    public double getFlywheelVelocity () { return flywheelVelocity; }

    public double getVoltage() {
        return voltage;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("voltage", voltage);
        TelemetryUtil.packet.put("Shooter : Flywheel velocity", flywheelVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel velocity 2", flywheelVelo2);
        TelemetryUtil.packet.put("Pinpoint : X", currentPose.x);
        TelemetryUtil.packet.put("Pinpoint : Y", currentPose.y);
        TelemetryUtil.packet.put("Pinpoint : Angle (deg)", Math.toDegrees(currentPose.heading));
        TelemetryUtil.packet.put("Pinpoint : Velocity X (in/s)", vel.x);
        TelemetryUtil.packet.put("Pinpoint : Velocity Y (in/s)", vel.y);
        TelemetryUtil.packet.put("Pinpoint : Velocity Angle (deg/s)", Math.toDegrees(Math.atan2(vel.x, vel.y)));
        TelemetryUtil.packet.put("Balls", balls.toString());

        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(fieldOverlay, currentPose, "#00ff00");

        LogUtil.driveCurrentX.set(currentPose.x);
        LogUtil.driveCurrentY.set(currentPose.y);
        LogUtil.driveCurrentAngle.set(currentPose.heading);
        LogUtil.flywheelVelocity.set(flywheelVelocity);
    }
}
