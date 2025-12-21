package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.blueTag;
import static org.firstinspires.ftc.teamcode.utils.Globals.redTag;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.MergeLocalizer;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Polynomial;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;
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
    public Vector3 ballTarget;
    public Vector3 distance;
    public Vector3 ballExit2DSpd;
    public Vector3 tVel;
    public Vector3 rVel;
    public Vector3 vel;
    public double minV0 = 0.0;
    public double minV0Superthresh = 0.0; // TODO: need to tune this, controls how much over minV0 we make the v0 strive for pre mult
    public double minV0factor = 1.26; // TODO: tune this so that triple fire works; without this, 2nd & 3rd balls don't go in
    public double flywheelEfficiency = 0.6367;
    public double targetTurretAngle = 0.0;
    public double targetHoodAngle = 0.0;
    public double phiLim = Math.atan(0.875);

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

        ballTarget = new Vector3(-70.5, 60 * (Globals.isRed ? 1 : -1), 38.75 + 3);
    }

    public void update() {
        switch (state) {
            case IDLE:
                setShooterBlocker(true);
                aimLauncherV8();
                setTargetVelocity(0.0);
                setTurretAngle(targetTurretAngle);
                setHoodAngle(targetHoodAngle);

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

                setTurretAngle(targetTurretAngle);
                setHoodAngle(targetHoodAngle);

                if (stopRequest) {
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                }
                break;
            case READY:
                setShooterBlocker(false);
                aimLauncherV8();
                setTargetVelocity(minV0);
                setTurretAngle(targetTurretAngle);
                setHoodAngle(targetHoodAngle);

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
                setTurretAngle(targetTurretAngle);
                setHoodAngle(targetHoodAngle);

                if (stopRequest) {
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                    robot.intake.reqOff(true);
                }
                break;
            case TEST: // LEAVE THIS EMPTY AT ALL TIMES
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

        TelemetryUtil.packet.put("Shooter : turretTargetAngle", targetAngle);
        LogUtil.turretAngle.set(targetAngle);
    }

    public boolean aimLauncherV8() {
        Vector3 P = new Vector3(ballTarget); // TODO: change this target to account for distance
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
        if (tRoots.isEmpty()) return false;

        minV0 = Math.sqrt(2 * a * tRoots.get(0) * tRoots.get(0) + c + d / 2 / tRoots.get(0)) + minV0Superthresh;
        minV0 *= minV0factor * 2 / flywheelEfficiency; // converts minV0 to min flywheel vel for triple

        double v0 = getBallExitSpd();
        if (V.getMag() < 0.1) {
            targetTurretAngle = AngleUtil.clipAngle(Math.atan2(P.getY(), P.getX()) - ROBOT_POSITION.heading);
            targetHoodAngle = Math.PI / 2 - Math.asin(2 * g * P.getMag() / (v0 * v0)) / 2 - phiLim;
        }
        c -= v0 * v0;

        tRoots = Polynomial.findRealRoots(new double[]{1, 0, c/a, d/a, e/a}, 1e-4);
        if (tRoots.isEmpty()) return false;

        double[] thetas = new double[tRoots.size() + 1];
        double[] phis = new double[tRoots.size() + 1];
        for (int i = 0; i < tRoots.size(); i++) {
            double t0 = tRoots.get(i);
            Vector3 pf = new Vector3(P.x + V.x * t0, P.y + V.y * t0, P.z + g * t0 * t0 / 2);
            thetas[i] = pf.theta();
            phis[i] = pf.phi();
            double c1 = (58.3414785 - 72) / (55.6424675 - 48);
            double c2 = c1 * 48 - 72;
            double slope = c1 * (ROBOT_VELOCITY.y + v0 * Math.sin(thetas[i]) * Math.sin(phis[i])) - (ROBOT_VELOCITY.x + v0 * Math.cos(thetas[i]) * Math.sin(phis[i]));
            double t = -c2 / slope;

            if (t < 0) phis[i] = 100;
            else if (launcherHeight + v0 * Math.cos(phis[i]) * t - g * t * t / 2 < 38.75 + 3.5) phis[i] = 100;
            if (phis[i] - phiLim < 0) phis[i] = 100;
            if (i == 0) {
                thetas[tRoots.size()] = thetas[0];
                phis[tRoots.size()] = phis[0];
            } else {
                if (phis[i] < phis[tRoots.size()]) {
                    phis[tRoots.size()] = phis[i];
                    thetas[tRoots.size()] = thetas[i];
                }
            }

        }
        if (phis[tRoots.size()] == 100) return false;
        targetTurretAngle = AngleUtil.clipAngle(thetas[tRoots.size()] - ROBOT_POSITION.heading); // converts from global to difference with heading
        targetHoodAngle = Math.PI / 2 - phis[tRoots.size()] - phiLim;
        return true;
    }

    /**
     * @return v0 in in/s
     */
    public double getBallExitSpd() {
        return filteredVelocity * flywheelEfficiency * 0.5;
        // the ~640 is just from the random 640 in/sec that PJ said, I have no clue what the model will be like but I suspect linear or quadratic
    }

    public void setHoodAngle(double target_angle) {
        hood.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : hoodTargetAngle", target_angle);
        LogUtil.hoodAngle.set(target_angle);
    }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }

    public double getTargetVelocity() { return targetVelocity; }

    public double getFilteredVelocity() { return filteredVelocity; }

    public void setShooterBlocker(boolean active) { flywheelBlocker.setTargetAngle(active ? latchBlockAngle : -0.2);}

    public boolean atVel () {return Math.abs(targetVelocity - filteredVelocity) < 1.0;}


    // bootleg LM1 strat being used in LM2 code
    public static double closeAngle = 0.7, closeVel = 65, midAngle = 1.0, midVel  = 77, farAngle = 1.34, farVel = 0.0;
    public enum Dist {
        CLOSE(closeAngle, closeVel),
        MID(midAngle, midVel),
        FAR(farAngle, farVel),
        OFF(0.0, 0.0);

        private double hoodAngle, flywheelVel;

        Dist(double hoodAngle, double flywheelVel){
            this.hoodAngle = hoodAngle;
            this.flywheelVel = flywheelVel;
        }

        public static void setHoodAngle(Dist dist, double angle){
            dist.hoodAngle = angle;
        }

        public static void setFlywheelVel (Dist dist, double vel){
            dist.flywheelVel = vel;
        }
    } Dist dist = Dist.CLOSE;

    public double error;
    public void updateBootleg() {
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

        Vector2 d = Globals.isRed ? redTag.toVec2() : blueTag.toVec2();
        d.subtract(new Vector2(ROBOT_POSITION.x, ROBOT_POSITION.y));
        targetTurretAngle = Utils.minMaxClip(AngleUtil.clipAngle(Math.atan2(d.y, d.x) - ROBOT_POSITION.heading), -Math.PI / 2, Math.PI);
        setTurretAngle(targetTurretAngle);
        setHoodAngle(targetHoodAngle);

        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel PID Power", pow * 100);
        TelemetryUtil.packet.put("Shooter : Turret Target Angle", turret.getTargetAngle());
    }

    public void setShooter(Dist mode) {
        targetVelocity = mode.flywheelVel;
        targetHoodAngle = mode.hoodAngle;
    }

    public boolean atVelBootleg() { return Math.abs(error) < 1.0; }
}
