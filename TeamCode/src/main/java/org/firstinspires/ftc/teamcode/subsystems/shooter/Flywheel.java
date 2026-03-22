package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Flywheel {
    private final Robot robot;
    public final PriorityMotor flywheel;

    /*
    Vel - FF - Tuned 2/25/26
    0.09 - 40
    0.15 - 87
    0.22 - 175
    0.28 - 222
    0.33 - 276
    0.39 - 329
    0.47 - 417
    0.55 - 491
    0.64 - 565
    0.69 - 606
    0.74 - 670
    0.82 - 706
    0.91 - 767
    0.96 - 807
    1 - 848
     */

    // velocity is in inches / second
    public static PID velocityPID = new PID (0.03, 0.0003, 0.0001);
    public static double velocityFFm = 0.00124059 * 20 / 16;
    public static double velocityFFb = 0.0264087;
    public static double velocityFilterLow = 0.05;
    public static double velocityFilterHigh = 0.5;
    public static double velocityFilterThresh = 60;
    public static double velocityHighPowerThresh = 15;
    public static double velocityNoSkipThresh = 150;
    public static double velocityNoSkipAccel = 1.5;
    public static double flywheelScaleVoltage = 12.5;
    public static double atVelThresh = 20;
    private double targetVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double prevPow = 0;

    public Flywheel(Robot robot) {
        this.robot = robot;

        DcMotorEx ms1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx ms2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        ms1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ms2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel = new PriorityMotor(new DcMotorEx[]{ms1, ms2},"flywheel",3, 5, new double[] {1, -1}, robot.sensors);

        robot.hardwareQueue.addDevice(flywheel);
    }

    public void update() {
        // Flywheel Velocity PIDF
        double actualVelocity = robot.sensors.getFlywheelVelocity();
        if (Math.abs(actualVelocity - filteredVelocity) <= velocityFilterThresh) {
            filteredVelocity = filteredVelocity * (1 - velocityFilterLow) + actualVelocity * velocityFilterLow;
        } else {
            filteredVelocity = filteredVelocity * (1 - velocityFilterHigh) + actualVelocity * velocityFilterHigh;
        }
        double error = targetVelocity - filteredVelocity;
        if (targetVelocity <= 1 || error > velocityFilterThresh) velocityPID.resetIntegral();
        else velocityPID.clipIntegral(-1, 1);

        double pidpow = velocityPID.update(error, -1.0, 1.0);
        double ffpow = targetVelocity * velocityFFm + velocityFFb;
        double pow = Math.max(0, pidpow + ffpow) * flywheelScaleVoltage / robot.sensors.getVoltage();
        if (error > velocityHighPowerThresh) pow = 1;
        else if (error < -velocityHighPowerThresh) pow = 0;
        if (filteredVelocity < velocityNoSkipThresh) {
            pow = Math.min(pow, prevPow + velocityNoSkipAccel * robot.sensors.loopTime);
        }
        flywheel.setTargetPower(pow);
        prevPow = pow;

        TelemetryUtil.packet.put("Flywheel : Power Applied", pow * 100);
        TelemetryUtil.packet.put("Flywheel : Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Flywheel : Filtered Velocity", filteredVelocity);
        LogUtil.flywheelTarget.set(targetVelocity);
    }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
    public double getFilteredVelocity() { return filteredVelocity; }

    public boolean atVel() { return Math.abs(targetVelocity - filteredVelocity) <= atVelThresh; }
    public boolean atVel(double thresh) { return Math.abs(targetVelocity - filteredVelocity) <= thresh; }
}
