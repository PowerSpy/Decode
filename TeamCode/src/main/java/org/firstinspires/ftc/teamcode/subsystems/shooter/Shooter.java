package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.blueTag;
import static org.firstinspires.ftc.teamcode.utils.Globals.redTag;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Polynomial;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

import java.util.ArrayList;
import java.util.List;

@Config
public class Shooter {
    public enum State {
        IDLE,
        AIMING,
        READY,
        SHOOT,
        TEST
    } public State state = State.IDLE;

    private final Robot robot;
    private final Sensors sensors;
    private final DcMotorEx ms1, ms2;
    public final PriorityMotor flywheel;
    public final nPriorityServo turret, hood, flywheelBlocker, net;

    public static ArrayList<Long> nanoTimes;
    public static ArrayList<Double> turretHistory;

    private boolean aimRequest = false, shootRequest = false, stopRequest = false;

    // velocity is in inches / second
    public static PID velocityPID = new PID (0.0, 0.0001, 0.0001);
    public static double velocityFFm = 0.000838827;
    public static double velocityFFb = 0.0530646;
    public static double velocityFilterLow = 0.05;
    public static double velocityFilterHigh = 0.5;
    public static double velocityFilterThresh = 100;
    public static double velocityHighPowerThresh = 25;
    public static double velocityNoSkipThresh = 400;
    public static double velocityNoSkipAccel = 0.8;
    public static double latchBlockAngle = 2.7;
    private double targetVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double prevPow = 0;

    // auto-aim
    private final double dLauncher = Math.sqrt(66.632 * 66.632 + 229.61 * 229.61) / 25.4;
    private final double g = 9.805 * 100 / 2.54;
    private final double launcherHeight = 330.14203 / 25.4;
    public Vector3 ballTarget = new Vector3(-60, 60, 38.75 + 3); // +3 for safety of ball going in, clearance = 3 - 2.5 = 0.5
    public Vector3 distance = new Vector3(0,0,0);
    public Vector3 ballExit2DSpd = new Vector3(0, 0, 0);
    public Vector3 tVel  = new Vector3(0, 0, 0);
    public Vector3 rVel = new Vector3(0, 0, 0);
    public Vector3 vel = new Vector3(0, 0, 0);
    public double minV0 = 0.0;
    public double minV0Superthresh = 6.0; // TODO: need to tune this, controls how much over minV0 we make the v0 strive for

    public Shooter(Robot robot) {
        this.robot = robot;

        this.sensors = robot.sensors;

        this.ms1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        this.ms2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheel = new PriorityMotor(new DcMotorEx[]{ms1, ms2},"flywheel",3, 5, new double[] {1, -1}, robot.sensors);

        hood = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood1")},
            "hood", nPriorityServo.ServoType.AXON_MINI,
            0.0, 0.4, 0.02,
            new boolean[] {false},
            2, 5
        );

        turret = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "turret1"), robot.hardwareMap.get(Servo.class,"turret2")},
            "turret", nPriorityServo.ServoType.AXON_MINI,
            0.1, 0.78, 0.5,
            new boolean[] {false, false},
            2, 5
        );
        //turret.maxPower = 0.8;

        nanoTimes = new ArrayList<>();
        turretHistory = new ArrayList<>();

        flywheelBlocker = new nPriorityServo(
                new Servo[]{robot.hardwareMap.get(Servo.class, "flywheelBlocker")},
                "flywheelBlocker", nPriorityServo.ServoType.AXON_MICRO,
                0, 0.7, 0.1,
                new boolean[] {false},
                2, 5
        );

        net = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "net")},
                "net", nPriorityServo.ServoType.AXON_MINI,
                0.42, 0.95, 0.5,
                new boolean [] {false},
                2, 5
        );

        robot.hardwareQueue.addDevices(flywheel, hood, turret, flywheelBlocker, net);

        flywheel.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        switch (state) {
            case IDLE:
                setShooterBlocker(true);
                aimLauncherV8();
                setTargetVelocity(0.0);

                if (aimRequest) {
                    state = State.AIMING;
                }
                break;
            case AIMING:
                setShooterBlocker(true);
                setTargetVelocity(minV0);

                if (aimLauncherV8() && atVel()) {
                    state = State.READY;
                }

                if (stopRequest) {
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                }
                break;
            case READY:
                setShooterBlocker(false);
                aimLauncherV8();
                setTargetVelocity(minV0);

                if (flywheelBlocker.inPosition() && shootRequest) {
                    state = State.SHOOT;
                    robot.intake.reqShoot(true);
                }

                if (stopRequest) {
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                }
                break;
            case SHOOT:
                setShooterBlocker(false);
                aimLauncherV8();
                setTargetVelocity(minV0);

                if (stopRequest) {
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                    robot.intake.reqOff(true);
                }
                break;
            case TEST:
                break;
        }

        // Flywheel Velocity PID
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
        double pow = pidpow + ffpow;
        if (error > velocityHighPowerThresh) pow = 1;
        if (filteredVelocity < velocityNoSkipThresh) {
            pow = Math.min(pow, prevPow + velocityNoSkipAccel * robot.sensors.loopTime);
        }
        flywheel.setTargetPower(pow);
        prevPow = pow;

        // Aim Correction
        nanoTimes.add(0, System.nanoTime());
        turretHistory.add(0, turret.getCurrentAngle());

        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel PID Power", pidpow * 100);
        TelemetryUtil.packet.put("Shooter : Flywheel FF Power", ffpow * 100);
        TelemetryUtil.packet.put("Shooter : Flywheel Applied Power", pow * 100);
        TelemetryUtil.packet.put("Shooter : Turret Target Angle", turret.getTargetAngle());
        TelemetryUtil.packet.put("Shooter : state", this.state);
        LogUtil.shooterState.set(this.state.toString());
    }

    public void reqShoot (boolean req) { shootRequest = req; }

    public void reqAim (boolean req) { aimRequest = req; }

    public void reqStop (boolean req) { stopRequest = req; }

    public void setTurretAngle (double targetAngle) {
        turret.setTargetAngle(targetAngle);

        TelemetryUtil.packet.put("Shooter : turretAngle", targetAngle);
        LogUtil.turretAngle.set(targetAngle);
    }

    public boolean aimLauncherV8() {
        Vector3 P = new Vector3(-58.3414785, 55.6424675, 39.25 + 1); // TODO: change this hardcoded target to account for team color & distance
        P.subtract(new Vector3(ROBOT_POSITION.x, ROBOT_POSITION.y, launcherHeight));
        Vector3 V = new Vector3(-ROBOT_VELOCITY.x, -ROBOT_VELOCITY.y, 0); // TODO: need to subtract robot angular vel component thing to this

        double a = g * g / 4;
        // double b = 0;
        double c = V.x * V.x + V.y + V.y + g * P.z;
        double d = 2 * Vector3.dot(P, V);
        double e = P.x * P.x + P.y * P.y + P.z * P.z;
        List<Double> tRoots = Polynomial.findRealRoots(new double[]{1, 0.0, 0.0, -d/(2 * a), -e/a}, 1e-4);
        for (int i = 0; i < tRoots.size(); i++) {
            if(tRoots.get(i) < 0) {
                tRoots.remove(i);
                i--;
            }
        }
        if (tRoots.isEmpty()) {
            return false;
        }

        double minPhiT0 = -1.0;
        double[] thetas = new double[tRoots.size() + 1];
        double[] phis = new double[tRoots.size() + 1];
        for (int i = 0; i < tRoots.size(); i++) {
            double t0 = tRoots.get(i);
            Vector3 pf = new Vector3(P.x + V.x * t0, P.y + V.y * t0, P.z + g * t0 * t0 / 2);
            thetas[i] = pf.theta();
            phis[i] = pf.phi();
            if (i == 0) {
                thetas[tRoots.size()] = thetas[0];
                phis[tRoots.size()] = phis[0];
                minPhiT0 = t0;
            } else {
                if (phis[i] < phis[tRoots.size()]) {
                    phis[tRoots.size()] = phis[i];
                    thetas[tRoots.size()] = thetas[i];
                    minPhiT0 = t0;
                }
            }

        }

        minV0 = Math.sqrt(2 * a * minPhiT0 * minPhiT0 + c + d / 2 / minPhiT0) + minV0Superthresh;
        setTurretAngle((thetas[tRoots.size()] - ROBOT_POSITION.heading)); // converts from global to difference with heading
        hood.setTargetAngle(phis[tRoots.size()]);
        return true;
        // currently doesn't solve the thing twice, where the v0 is set to minV0 + superThresh
        // may cause some deviations from target
    }

    /**
     * The contents are just a placeholder for now
     * @return
     */
    public double getBallExitSpd() {
        return 639.899748567 / 14 * sensors.getVoltage();
        // the ~640 is just from the random 640 in/sec that PJ said, I have no clue what the model will be like but I suspect linear or quadratic
    }

    public void setHoodAngle(double target_angle) {
        hood.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : hoodAngle", target_angle);
        LogUtil.hoodAngle.set(target_angle);
    }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }

    public double getTargetVelocity() { return targetVelocity; }

    public double getFilteredVelocity() { return filteredVelocity; }

    public void setShooterBlocker(boolean active) { flywheelBlocker.setTargetAngle(active ? latchBlockAngle : -0.2);}

    public boolean atVel () {return Math.abs(targetVelocity - filteredVelocity) < 1.0;}
}
