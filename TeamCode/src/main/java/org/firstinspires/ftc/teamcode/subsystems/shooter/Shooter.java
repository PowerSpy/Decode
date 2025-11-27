package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Shooter {
    public enum State {
        CLOSE(0.7, 65),
        MID(1.0, 75),
        FAR(1.34, 100),
        OFF(0.0, 0.0);

        private final double hoodAngle, flywheelVel;

        State(double hoodAngle, double flywheelVel){
            this.hoodAngle = hoodAngle;
            this.flywheelVel = flywheelVel;
        }
    } State state = State.CLOSE;

    private final Robot robot;
    private final DcMotorEx ms1, ms2;
    public final PriorityMotor flywheel;
    public final nPriorityServo flywheelBlocker, turret, hood/*, cloth*/;

    private double turretError;
    private long lastUpdateTime = System.currentTimeMillis();
    public static double limelightThresh = 5.0, limelightTimeDelay = 10, limelightScalar = 0.05;

    // velocity is in inches / second
    public static PID velocityPID = new PID (0.0, 0.001, 0.001);
    public static double velocityFFm = 0.0086733;
    public static double velocityFFb = 0.0414964;
    public static double velocityFilterLow = 0.05;
    public static double velocityFilterHigh = 0.5;
    public static double velocityFilterThresh = 10;
    public static double velocityHighPowerThresh = 4;
    public static double velocityNoSkipThresh = 45;
    public static double velocityNoSkipAccel = 0.7;
    private double targetVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double prevPow = 0;

    /*
    Hood / Velo
    Far: 1.34 / 100
    Middle: 1.0 / 70
    Close: 0.7 / 60
     */


    public Shooter(Robot robot) {
        this.robot = robot;

        this.ms1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        this.ms2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheel = new PriorityMotor(new DcMotorEx[]{ms1, ms2},"flywheel",3, 5, new double[] {1, -1}, robot.sensors);

        hood = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood1"), robot.hardwareMap.get(Servo.class,"hood2")},
            "hood", nPriorityServo.ServoType.AXON_MINI,
            0.03, 0.38, 0.03,
            new boolean[] {false, true},
            2, 5
        );

        turret = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "turret1"), robot.hardwareMap.get(Servo.class,"turret2")},
            "turret", nPriorityServo.ServoType.AXON_MINI,
            0.49, 0.51, 0.5,
            new boolean[] {false, false},
            2, 5
        );

        flywheelBlocker = new nPriorityServo(
                new Servo[]{robot.hardwareMap.get(Servo.class, "flywheelBlocker")},
                "flywheelBlocker", nPriorityServo.ServoType.AXON_MICRO,
                0, 0.7, 0.1,
                new boolean[] {false},
                2, 5
        );
        robot.hardwareQueue.addDevices(flywheel, hood, turret, flywheelBlocker);

        flywheel.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    double error;
    public void update() {
        // Flywheel Velocity PID
        if (targetVelocity <= 1) velocityPID.resetIntegral();
        else velocityPID.clipIntegral(-1, 1);
        double actualVelocity = robot.sensors.getFlywheelVelocity();
        if (Math.abs(actualVelocity - filteredVelocity) < velocityFilterThresh) {
            filteredVelocity = filteredVelocity * (1 - velocityFilterLow) + actualVelocity * velocityFilterLow;
        } else {
            filteredVelocity = filteredVelocity * (1 - velocityFilterHigh) + actualVelocity * velocityFilterHigh;
        }
        error = targetVelocity - filteredVelocity;
        double pow = velocityPID.update(error, 0.0, 1.0) + targetVelocity * velocityFFm + velocityFFb;
        if (error > velocityHighPowerThresh) pow = 1;
        if (filteredVelocity < velocityNoSkipThresh) {
            pow = Math.min(pow, prevPow + velocityNoSkipAccel * robot.sensors.loopTime);
        }
        flywheel.setTargetPower(pow);
        prevPow = pow;

        // Auto-aim (Disabled)
        /*
        goalDetector.update();
        if(goalDetector.isTagDetected() && Math.abs(goalDetector.getTx()) > limelightThresh && System.currentTimeMillis() - lastUpdateTime >= limelightTimeDelay){
            turretError = turret.getCurrentAngle() - Math.signum(goalDetector.getTx()) * limelightScalar;
            lastUpdateTime = System.currentTimeMillis();
        } else if (!goalDetector.isTagDetected()){
            turretError = 0;
        }
        turret.setTargetAngle(turretError);
         */

        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel PID Power", pow * 100);
        TelemetryUtil.packet.put("Shooter : Turret Target Angle", turret.getTargetAngle());
    }

    public void setTurretAngle(double target_angle) {
        turret.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : turretAngle", target_angle);
        LogUtil.turretAngle.set(target_angle);
    }

    public void setHoodAngle(double target_angle) {
        hood.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : hoodAngle", target_angle);
        LogUtil.hoodAngle.set(target_angle);
    }


    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
    public double getTargetVelocity() { return targetVelocity; }
    public double getFilteredVelocity() { return filteredVelocity; }

    public void setShooter(State mode){
        targetVelocity = mode.flywheelVel;
        hood.setTargetAngle(mode.hoodAngle);
    }

    public void setShooterBlocker (boolean on) {flywheelBlocker.setTargetAngle (on ? 1.5 : 0);}

    public boolean atVel () {return error < 1.0;}
}
