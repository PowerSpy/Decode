package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Polynomial;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

import java.util.List;

@Config
public class Shooter {
    public enum State {
        IDLE,
        AIMING,
        READY,
        SHOOT,
        INDEX,
        INDEXING,
        TEST
    } public State state = State.IDLE;

    private Robot robot;
    private DcMotorEx ms1, ms2;
    public PriorityMotor flywheel;
    public nPriorityServo hood, flywheelBlocker, net, kicker;
    public PriorityCRServo turret;

    private boolean aimRequest = false, shootRequest = false, stopRequest = false, indexRequest = false;

    public static PID turretPID = new PID (0.2, 0.0, 0.02);
    public static double turretMinPow = 0.1;
    public static double turretIntegralThresh = Math.toRadians(40);

    // velocity is in inches / second
    public static PID velocityPID = new PID (0.0, 0.0002, 0.0001);
    public static double velocityFFm = 0.000838827;
    public static double velocityFFb = 0.0530646;
    public static double velocityFilterLow = 0.05;
    public static double velocityFilterHigh = 0.5;
    public static double velocityFilterThresh = 100;
    public static double velocityHighPowerThresh = 25;
    public static double velocityNoSkipThresh = 400;
    public static double velocityNoSkipAccel = 0.8;
    public static double atVelThresh = 15;
    public static double latchBlockAngle = 2.5;
    private double targetVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double prevPow = 0;

    // auto-aim
    public final double dLauncher = Math.sqrt(66.632 * 66.632 + 229.61 * 229.61) / 25.4;
    public final double g = 9.805 * 100 / 2.54;
    public final double launcherHeight = 330.14203 / 25.4;
    public Vector3 ballTarget, P, V;
    public double a = g * g / 4, c, d, e;
    public double v0, cv0;
    public double minV0 = 0.0, minFlywheelVelocity = 0.0;
    public double minV0Superthresh = 0.0; // TODO: need to tune this, controls how much over minV0 we make the v0 strive for pre mult
    public double minV0factor = 1.07;
    public static double minV0factorClose = 1; // TODO: tune this so that triple fire works; without this, 2nd & 3rd balls don't go in
    public static double minV0factorFar = 1; // TODO: tune this so that triple fire works; without this, 2nd & 3rd balls don't go in
    public static double flywheelEfficiency = 0.63;
    public static double flywheelEfficiencyConstantFarAddition = -0.05;
    public double targetTurretAngle = 0.0;
    public double targetHoodAngle = 0.0;
    public static double hoodSweep = Math.toRadians(34.0);
    public static double hoodGearRatio = 48.0 / 30.0;
    private final double c1 = (58.3414785 + 72) / (55.6424675 - 48);
    private final double c2 = c1 * 48 - 72;
    private int moves;
    private boolean index;

    public Shooter(Robot robot) {
        this.robot = robot;

        this.ms1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        this.ms2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheel = new PriorityMotor(new DcMotorEx[]{ms1, ms2},"flywheel",3, 5, new double[] {1, -1}, robot.sensors);

        hood = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood1")},
            "hood", nPriorityServo.ServoType.AXON_MINI,
            0.027, 0.4, 0.03,
            new boolean[] {false},
            2, 5
        );

        kicker = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "kicker")},
            "kicker", nPriorityServo.ServoType.AXON_MICRO,
            0.027, 0.4, 0.03,
            new boolean[] {false},
            2, 5
        );

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

        turret = new PriorityCRServo(
                new CRServo[] {robot.hardwareMap.get(CRServo.class, "turret1"), robot.hardwareMap.get(CRServo.class, "turret2")},
                "turret", PriorityCRServo.ServoType.AXON_MINI,
                new boolean[] {false, false},
                3, 5
        );

        robot.hardwareQueue.addDevices(flywheel, hood, turret, flywheelBlocker, net);

        updateBallTarget();
    }

    // FSM needs to be upgraded to allow for indexing, we won't need it till lm4+, but we be working on it
    // idle -> aim req -> accel -> shoot
    // idle -> index req -> accel -> index
    // should be able to cancel into stop -> idle anytime
    // should be able to cancel into aim req or index req within accel
    public void update() {
        switch (state) {
            case IDLE:
                stopRequest = false;

                //aimLauncherV8();
                setTurretAngle(0.0);
                setTargetVelocity(0.0);
                setHoodAngle(0.0);
                setShooterBlocker(true);

                if (aimRequest) {
                    aimRequest = false;
                    state = State.AIMING;
                }
                if (indexRequest) {
                    indexRequest = false;
                    state = State.INDEX;
                }
                break;
            case INDEX:
                calcIndexPosition(0,0); // get values from LL
                state = State.INDEXING;
                break;
            case INDEXING:
                indexRequest = false;

                setShooterBlocker(true);
                setKicker(0);
                // have the feeder and whatever contraption push balls forward
                if(moves > 0){
                    moves--;
                    index = true;
                    state = State.SHOOT;
                    break;
                }
                reqIndex(false);
                state = State.IDLE;
                break;
            case AIMING:
                aimRequest = false;

                setShooterBlocker(true);
                if (aimLauncherV8()) {
                    state = State.READY;
                }
                setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(0.0);

                if (stopRequest) {
                    stopRequest = false;
                    aimRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                }
                break;
            case READY:
                setShooterBlocker(true);

                aimLauncherV8();
                setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                if (shootRequest) {
                    setShooterBlocker(false);
                    if (flywheelBlocker.inPosition()) {
                        state = State.SHOOT;
                        robot.intake.reqShoot(true);
                    }
                }

                if (stopRequest) {
                    stopRequest = false;
                    aimRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                }
                break;
            case SHOOT:
                shootRequest = false;

                setShooterBlocker(false);
                if (index) {
                    setTargetVelocity(60); //test optimal
                    setHoodAngle(0.8); //test optimal
                    setKicker(4.0); //test optimal
                    index = false;
                    if(kicker.inPosition()){
                        state = State.INDEXING;
                    }
                } else {
                    aimLauncherV8();
                    setTargetVelocity(minFlywheelVelocity);
                    setHoodAngle(targetHoodAngle);
                }

                if (stopRequest) {
                    stopRequest = false;
                    aimRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                }
                break;
            case TEST: // LEAVE THIS EMPTY AT ALL TIMES
                break;
        }

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
        double pow = Math.max(0, pidpow + ffpow);
        if (error > velocityHighPowerThresh) pow = 1;
        if (filteredVelocity < velocityNoSkipThresh) {
            pow = Math.min(pow, prevPow + velocityNoSkipAccel * robot.sensors.loopTime);
        }
        flywheel.setTargetPower(pow);
        prevPow = pow;

        // Turret PIDF
        targetTurretAngle = Sensors.turretAngleClip(targetTurretAngle);
        double turretError = targetTurretAngle - Sensors.turretAngleClip(robot.sensors.getTurretAngle());
        if (Math.abs(turretError) > turretIntegralThresh) turretPID.resetIntegral();
        else turretPID.clipIntegral(-1, 1);
        double turretPow = turretPID.update(turretError, -1, 1) + turretMinPow * Math.signum(turretError);
        if (Math.abs(turretError) < 0.05) turretPow = 0;
        turret.setTargetPower(turretPow);

        TelemetryUtil.packet.put("Shooter : state", this.state);
        //TelemetryUtil.packet.put("Shooter : Flywheel Power PID", pidpow * 100);
        //TelemetryUtil.packet.put("Shooter : Flywheel Power FF", ffpow * 100);
        TelemetryUtil.packet.put("Shooter : Flywheel Power Applied", pow * 100);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Turret Target", targetTurretAngle);
        TelemetryUtil.packet.put("Shooter : Hood Target", hood.getTargetAngle());
        TelemetryUtil.packet.put("Shooter : Turret Power PID", turretPow * 100);
        LogUtil.flywheelTarget.set(targetVelocity);
        LogUtil.shooterState.set(this.state.toString());
        LogUtil.turretTarget.set(targetTurretAngle);
        LogUtil.hoodAngle.set(hood.getTargetAngle());
    }

    public void reqShoot (boolean req) { shootRequest = req; }

    public void reqAim (boolean req) { aimRequest = req; }

    public void reqStop (boolean req) { stopRequest = req; }

    public void setManual(boolean on) {
        if (on) {
            state = State.TEST;
            robot.shooter.setTurretAngle(0);
            setShooter(Dist.OFF);
        } else {
            state = State.IDLE;
        }
    }

    public void reqIndex (boolean req) { indexRequest = req; }

    public void setTurretAngle (double targetAngle) {
        targetTurretAngle = targetAngle;
    }

    public void setHoodAngle(double target_angle) { hood.setTargetAngle(target_angle); }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }

    public double getTargetVelocity() { return targetVelocity; }

    public double getFilteredVelocity() { return filteredVelocity; }

    public void setShooterBlocker(boolean active) { flywheelBlocker.setTargetAngle(active ? latchBlockAngle : -0.2);}

    public void updateBallTarget() {
        ballTarget = new Vector3(-70.5, 60 * (Globals.isRed ? 1 : -1), 38.75);
    }

    public int calcIndexPosition(int greenPos, int motifPos){
        moves = (motifPos - greenPos)%3;
        return moves;
    }

    public boolean turretTrackTarget() {
        if (ROBOT_POSITION == null || ROBOT_VELOCITY == null) return false;
        // targetTurretAngle = AngleUtil.clipAngle(Math.atan2(P.getY(), P.getX()) - ROBOT_POSITION.heading);
        // above works
        // below needs testing
        targetTurretAngle = AngleUtil.clipAngle(Math.atan2(P.getY(), P.getX()) - ROBOT_POSITION.heading);
        return true;
    }

    public void updateAimingConstants() {
        if (ROBOT_POSITION == null || ROBOT_VELOCITY == null) return;
        P = new Vector3(ballTarget); // we'll figure out whether we need to change this target based on distance
        P.subtract(new Vector3(ROBOT_POSITION.x, ROBOT_POSITION.y, launcherHeight));
        V = new Vector3(-ROBOT_VELOCITY.x, -ROBOT_VELOCITY.y, 0); // TODO: need to subtract robot angular vel component thing to this

        v0 = getBallExitSpd();

        // double b = 0;
        c = V.x * V.x + V.y * V.y + g * P.z;
        cv0 = c - v0 * v0;
        d = 2 * Vector3.dot(P, V);
        e = P.x * P.x + P.y * P.y + P.z * P.z;
        if (P.getMag() >= 90) minV0factor = minV0factorFar;
        else minV0factor = minV0factorClose;

    }

    public boolean calcMinFlywheelVelocity() {
        if (ROBOT_POSITION == null || ROBOT_VELOCITY == null) return false;
        List<Double> tRoots = Polynomial.findRealRoots(new double[]{1, 0.0, 0.0, -d/(2 * a), -e/a}, 1e-4);
        for (int i = 0; i < tRoots.size(); i++) {
            if(tRoots.get(i) < 0) {
                tRoots.remove(i);
                i--;
            }
        }
        if (tRoots.isEmpty()) return false;
        minV0 = Math.sqrt(2 * a * tRoots.get(0) * tRoots.get(0) + c + d / 2 / tRoots.get(0)) + minV0Superthresh;
        minV0 *= minV0factor;
        minFlywheelVelocity = minV0 * 2;
        if (P.getMag() >= 90) minFlywheelVelocity /= flywheelEfficiency + flywheelEfficiencyConstantFarAddition;
        else minFlywheelVelocity /= flywheelEfficiency;
        return true;
    }

    public boolean calcStaticShotAngles() {
        if (!turretTrackTarget()) return false;
        if (v0 > (minV0 / minV0factor - minV0Superthresh) * 0.97) { // makes sure v0 is at least 97% of true minV0
            double A = 4 * v0 * v0 * v0 * v0 * e;
            double dist2 = e - P.z * P.z; // 2D dist squared
            double B = 4 * dist2 * v0 * v0 * (g * P.z - v0 * v0);
            double C = dist2 * dist2 * g * g;
            List<Double> tRoots = Polynomial.findRealRoots(new double[]{A, B, C}, 1e-4);
            double[] phis = new double[tRoots.size() + 1];
            double[] thetas = new double[phis.length];

            for (int i = 0; i < tRoots.size(); i++) {
                if (Math.abs(tRoots.get(i) - 0.5) <= 0.5) {
                    phis[i] = Math.asin(Math.sqrt(tRoots.get(i)));
                } else phis[i] = 100;
                thetas[i] = targetTurretAngle;

                double slope = c1 * (ROBOT_VELOCITY.y + v0 * Math.sin(thetas[i]) * Math.sin(phis[i])) - (ROBOT_VELOCITY.x + v0 * Math.cos(thetas[i]) * Math.sin(phis[i]));
                if ((int)slope == 0) {
                    phis[i] = 100;
                } else {
                    double t = (c1 * (48 - ROBOT_POSITION.y) + 72 + ROBOT_POSITION.x) / slope;
                    if (t <= 0) {
                        phis[i] = 100;
                    } else if (launcherHeight + v0 * Math.cos(phis[i]) * t - g * t * t / 2 < 38.75 + 3) {
                        phis[i] = 100;
                    }
                }
                if (phis[i] - hoodSweep < 0) phis[i] = 100;
                if (i == 0) {
                    thetas[tRoots.size()] = thetas[0];
                    phis[tRoots.size()] = phis[0];
                } else {
                    if (phis[i] != 100) {
                        if (phis[tRoots.size()] != 100) {
                            if (phis[i] > phis[tRoots.size()]) {
                                phis[tRoots.size()] = phis[i];
                                thetas[tRoots.size()] = thetas[i];
                            }
                        } else {
                            phis[tRoots.size()] = phis[i];
                            thetas[tRoots.size()] = thetas[i];
                        }
                    }
                }
            }

            if (phis[tRoots.size()] == 100) return false;
            targetTurretAngle = AngleUtil.clipAngle(thetas[tRoots.size()] - ROBOT_POSITION.heading); // converts from global to difference with heading
            targetHoodAngle = (phis[tRoots.size()] - hoodSweep) * hoodGearRatio; // first part converts angle from vertical to angle from horizontal && then subtracts the sweep of the hood
            return true;
        } else return false;
    }

    public boolean calcDynamicShotAngles() {
        if (!turretTrackTarget()) return false;
        if (v0 > (minV0 / minV0factor - minV0Superthresh) * 0.97) { // makes sure v0 is at least 97% of true minV0
            List<Double> tRoots = Polynomial.findRealRoots(new double[]{1, 0, cv0/a, d/a, e/a}, 1e-4);
            for(int i = 0; i < tRoots.size(); i++) {
                if(tRoots.get(i) < 0) {
                    tRoots.remove(i);
                    i--;
                }
            }
            if (tRoots.isEmpty()) {
                return false;
            }
            double[] phis = new double[tRoots.size() + 1];
            double[] thetas = new double[phis.length];
            for (int i = 0; i < tRoots.size(); i++) {
                double t0 = tRoots.get(i);
                Vector3 pf = new Vector3(P.x + V.x * t0, P.y + V.y * t0, P.z + g * t0 * t0 / 2);
                thetas[i] = pf.theta();
                phis[i] = pf.phi();


                double vel = Math.pow(ROBOT_VELOCITY.y + v0 * Math.sin(thetas[i]) * Math.sin(phis[i]), 2) + Math.pow(ROBOT_VELOCITY.x + v0 * Math.cos(thetas[i]) * Math.sin(phis[i]), 2);
                vel = Math.sqrt(vel);
                double dist = Math.pow(-60 - ROBOT_POSITION.x, 2) + Math.pow(ballTarget.y - ROBOT_POSITION.y, 2);
                dist = Math.sqrt(dist);
                double t = dist / vel;
                if (t <= 0) {
                    phis[i] = 100; // this makes sure the ball goes into the target through the restricted diagonal plane
                } else {
                    double heightAtWall = launcherHeight + v0 * Math.cos(phis[i]) * t - g * t * t / 2;
                    if (heightAtWall < 38.75 + 3) {
                        phis[i] = 100;
                    }
                }
                if (phis[i] - hoodSweep < 0) {
                    phis[i] = 100;
                }
                if (i == 0) {
                    thetas[tRoots.size()] = thetas[0];
                    phis[tRoots.size()] = phis[0];
                } else {
                    if (phis[i] != 100) {
                        if (phis[tRoots.size()] != 100) {
                            if (phis[i] > phis[tRoots.size()]) {
                                phis[tRoots.size()] = phis[i];
                                thetas[tRoots.size()] = thetas[i];
                            }
                        } else {
                            phis[tRoots.size()] = phis[i];
                            thetas[tRoots.size()] = thetas[i];
                        }
                    }
                }
            }

            if (phis[tRoots.size()] == 100) return false;
            targetTurretAngle = AngleUtil.clipAngle(thetas[tRoots.size()] - ROBOT_POSITION.heading); // converts from global to difference with heading
            targetHoodAngle = (phis[tRoots.size()] - hoodSweep) * hoodGearRatio; // first part converts angle from vertical to angle from horizontal && then subtracts the sweep of the hood
            return true;
        } else return false;
    }

    public boolean aimLauncherV8() {
        if (ROBOT_POSITION == null || ROBOT_VELOCITY == null) return false;
        Log.i("Points", "Starting aimLauncherV8");
        Vector3 P;
        if (Math.atan2(72 * (Globals.isRed ? 1 : -1) - ROBOT_POSITION.y, -72 - ROBOT_POSITION.x) <= Math.PI / 4) P = new Vector3(ballTarget);
        else P = new Vector3(-ballTarget.y * (Globals.isRed ? 1 : -1), -ballTarget.x * (Globals.isRed ? 1 : -1), ballTarget.z); // This only works for when base target is at
        P.subtract(new Vector3(ROBOT_POSITION.x, ROBOT_POSITION.y, launcherHeight));
        this.P = P;
        Vector3 V = new Vector3(-ROBOT_VELOCITY.x, -ROBOT_VELOCITY.y, 0); // TODO: need to subtract robot angular vel component thing to this

        targetTurretAngle = AngleUtil.clipAngle(Math.atan2(P.getY(), P.getX()) - ROBOT_POSITION.heading);
        Log.i("Points", "Set target turret angle & Starting MinV0");

        double a = g * g / 4;
        // double b = 0;
        double c = V.x * V.x + V.y * V.y + g * P.z;
        double d = 2 * Vector3.dot(P, V);
        double e = P.x * P.x + P.y * P.y + P.z * P.z;
        List<Double> tRoots = Polynomial.findRealRoots(new double[]{1, 0.0, 0.0, -d/(2 * a), -e/a}, 1e-4);
        for (int i = 0; i < tRoots.size(); i++) {
            if(tRoots.get(i) < 0) {
                tRoots.remove(i);
                i--;
            }
        }
        Log.i("MinV0", tRoots.size() + "");
        if (tRoots.isEmpty()) {
            Log.i("Points", "Shot dies in MinV0, tRoots is empty");
            return false;
        }
        Log.i("MinV0", "tRoots[0]: " + tRoots.get(0));

        minV0 = (Math.sqrt(2 * a * tRoots.get(0) * tRoots.get(0) + c + d / 2 / tRoots.get(0)) + minV0Superthresh);
        if (P.getMag() >= 90) minV0 *= minV0factorFar;
        else minV0 *= minV0factorClose;
        Log.i("MinV0", "value: " + minV0);
        minFlywheelVelocity = minV0 * 2;
        if (P.getMag() >= 90) minFlywheelVelocity /= flywheelEfficiency + flywheelEfficiencyConstantFarAddition;
        else minFlywheelVelocity /= flywheelEfficiency;
        Log.i("MinV0", "min flywheel:" + minFlywheelVelocity);

        double v0 = getBallExitSpd();
        Log.i("Points", "Got past MinV0, v0 = " + v0);

        double[] thetas;
        double[] phis;
        if (v0 > 36)  {
            Log.i("Points", "Entering the hood calculator");
            if (V.getMag() < 6) {
                Log.i("Points" , "Entering Static");
                double A = 4 * v0 * v0 * v0 * v0 * e;
                double dist2 = e - P.z * P.z; // 2D dist squared
                double B = 4 * dist2 * v0 * v0 * (g * P.z - v0 * v0);
                double C = dist2 * dist2 * g * g;
                tRoots = Polynomial.findRealRoots(new double[]{A, B, C}, 1e-4);
                Log.i("Static", "Root count: " + tRoots.size() + ", for polynomial [" + A + ", " + B + ", " + C + "]");
                if (tRoots.isEmpty()) {
                    Log.i("Points", "Shot dies in Static, tRoots is empty");
                    return false;
                }
                StringBuilder roots = new StringBuilder();
                for (int i = 0; i < tRoots.size(); i++) {
                    roots.append(tRoots.get(i));
                    if (i != tRoots.size() - 1) roots.append(", ");
                }
                Log.i("Static", "Roots: [" + roots.toString() + "]");
                phis = new double[tRoots.size() + 1];
                thetas = new double[phis.length];

                for (int i = 0; i < tRoots.size(); i++) {
                    if (Math.abs(tRoots.get(i) - 0.5) <= 0.5) {
                        phis[i] = Math.asin(Math.sqrt(tRoots.get(i)));
                    } else phis[i] = 100;
                    Log.i("Static", "Point 1: i = " + i + ", phis[i] = " + phis[i]);
                    thetas[i] = targetTurretAngle;

                    double vel = Math.pow(ROBOT_VELOCITY.y + v0 * Math.sin(thetas[i]) * Math.sin(phis[i]), 2) + Math.pow(ROBOT_VELOCITY.x + v0 * Math.cos(thetas[i]) * Math.sin(phis[i]), 2);
                    vel = Math.sqrt(vel);
                    double dist = Math.pow(-60 - ROBOT_POSITION.x, 2) + Math.pow(ballTarget.y - ROBOT_POSITION.y, 2);
                    dist = Math.sqrt(dist);
                    double t = dist / vel;
                    if (t <= 0) {
                        phis[i] = 100; // this makes sure the ball goes into the target through the restricted diagonal plane
                        Log.i("Static", "Point 2: i = " + i + ", t = " + t);
                    } else {
                        double heightAtWall = launcherHeight + v0 * Math.cos(phis[i]) * t - g * t * t / 2;
                        if (heightAtWall < 38.75 + 3) {
                            phis[i] = 100;
                            Log.i("Static", "Point 3: i = " + i + ", t = " + t + ", height at wall = " + heightAtWall);
                        }
                    }
                    // brlow needs to be fixed
//                    double slope = c1 * (ROBOT_VELOCITY.y + v0 * Math.sin(thetas[i]) * Math.sin(phis[i])) - ROBOT_VELOCITY.x + v0 * Math.cos(thetas[i]) * Math.sin(phis[i]);
//                    if ((int)slope == 0) {
//                        phis[i] = 100;
//                        Log.i("Static", "Point 1.5: i = " + i + ", slope = 0");
//                    } else {
//                        double t = (c1 * (48 - ROBOT_POSITION.y) + 72 + ROBOT_POSITION.x) / slope;
//                        if (t <= 0) {
//                            phis[i] = 100; // this makes sure the ball goes into the target through the restricted diagonal plane
//                            Log.i("Static", "Point 2: i = " + i + ", t = " + t);
//                        } else {
//                            double heightAtWall = launcherHeight + v0 * Math.cos(phis[i]) * t - g * t * t / 2;
//                            if (heightAtWall < 38.75 + 3) {
//                                phis[i] = 100;
//                                Log.i("Static", "Point 3: i = " + i + ", t = " + t + ", height at wall = " + heightAtWall);
//                            }
//                        }
//                    }

                    if (phis[i] - hoodSweep < 0) {
                        Log.i("Static", "Point 4: i = " + i + ", phis[i] = " + phis[i]);
                        phis[i] = 100;
                    }
                    Log.i("Static", "Point 5: i = " + i + ", phis[i] = " + phis[i]);
                    if (i == 0) {
                        thetas[tRoots.size()] = thetas[0];
                        phis[tRoots.size()] = phis[0];
                    } else {
                        if (phis[i] != 100) {
                            if (phis[tRoots.size()] != 100) {
                                if (phis[i] > phis[tRoots.size()]) {
                                    phis[tRoots.size()] = phis[i];
                                    thetas[tRoots.size()] = thetas[i];
                                }
                            } else {
                                phis[tRoots.size()] = phis[i];
                                thetas[tRoots.size()] = thetas[i];
                            }
                        }
                    }
                    Log.i("Static", "High Phi: " + phis[tRoots.size()]);
                    Log.i("Points", "Leaving Static");
                }
            } else {
                Log.i("Points", "Entering Dynamic");
                c -= v0 * v0;
                tRoots = Polynomial.findRealRoots(new double[]{1, 0, c/a, d/a, e/a}, 1e-4);
                Log.i("Dynamic", "tRoots.size() = " + tRoots.size());
                for(int i = 0; i < tRoots.size(); i++) {
                    if(tRoots.get(i) < 0) {
                        tRoots.remove(i);
                        i--;
                    }
                }
                if (tRoots.isEmpty()) {
                    Log.i("Points", "Shot dies in Dynamic, tRoots is empty");
                    return false;
                }
                StringBuilder roots = new StringBuilder("[");
                for (int i = 0; i < tRoots.size(); i++) {
                    roots.append(tRoots.get(i));
                    if (i != tRoots.size() - 1) roots.append(", ");
                }
                Log.i("Dynamic", "Roots: [" + roots.toString() + "]");
                phis = new double[tRoots.size() + 1];
                thetas = new double[phis.length];
                for (int i = 0; i < tRoots.size(); i++) {
                    double t0 = tRoots.get(i);
                    Vector3 pf = new Vector3(P.x + V.x * t0, P.y + V.y * t0, P.z + g * t0 * t0 / 2);
                    thetas[i] = pf.theta();
                    phis[i] = pf.phi();

                    double slope = c1 * (ROBOT_VELOCITY.y + v0 * Math.sin(thetas[i]) * Math.sin(phis[i])) - (ROBOT_VELOCITY.x + v0 * Math.cos(thetas[i]) * Math.sin(phis[i]));
                    if ((int)slope == 0) {
                        phis[i] = 100;
                        Log.i("Dynamic", "Point 1.5: i = " + i + ", slope = 0");
                    } else {
                        double t = (c1 * (48 - ROBOT_POSITION.y) + 72 + ROBOT_POSITION.x) / slope;
                        if (t <= 0) {
                            phis[i] = 100; // this makes sure the ball goes into the target through the restricted diagonal plane
                            Log.i("Dynamic", "Point 2: i = " + i + ", phis[i] = " + phis[i]);
                        } else if (launcherHeight + v0 * Math.cos(phis[i]) * t - g * t * t / 2 < 38.75 + 3) {
                            phis[i] = 100;
                            Log.i("Dynamic", "Point 3: i = " + i + ", phis[i] = " + phis[i]);
                        }
                    }
                    if (phis[i] - hoodSweep < 0) {
                        Log.i("Dynamic", "Point 4: i = " + i + ", phis[i] = " + phis[i]);
                        phis[i] = 100;
                    }
                    if (i == 0) {
                        thetas[tRoots.size()] = thetas[0];
                        phis[tRoots.size()] = phis[0];
                    } else {
                        if (phis[i] != 100) {
                            if (phis[tRoots.size()] != 100) {
                                if (phis[i] > phis[tRoots.size()]) {
                                    phis[tRoots.size()] = phis[i];
                                    thetas[tRoots.size()] = thetas[i];
                                }
                            } else {
                                phis[tRoots.size()] = phis[i];
                                thetas[tRoots.size()] = thetas[i];
                            }
                        }
                    }
                    targetTurretAngle = AngleUtil.clipAngle(thetas[tRoots.size()] - ROBOT_POSITION.heading); // converts from global to difference with heading
                    Log.i("Dynamic", "High Phi: " + phis[i]);
                    Log.i("Points", "Leaving Dynamic");
                }
            }
        } else {
            Log.i("Points", "Shot isn't possible, v0 is sub-36 in/s");
            return false;
        }


        if (phis[tRoots.size()] == 100) {
            Log.i("Points", "Phis are cooked, phis[" + tRoots.size() + "] = 100");
            return false;
        }
        targetHoodAngle = (phis[tRoots.size()] - hoodSweep) * hoodGearRatio; // first part converts angle from vertical to angle from horizontal && then subtracts the sweep of the hood
        Log.i("Points", "The shot is possible and we're returning true!!");
        return true;
    }

    public double getBallExitSpd() {
        if (P == null) return -1.0;
        double res = filteredVelocity * 0.5;
        if (P.getMag() >= 90) res *= flywheelEfficiency + flywheelEfficiencyConstantFarAddition;
        else res *= flywheelEfficiency;
        return res;
    }

    // further separation :)
    // bootleg LM1 strat being used in LM2 code
    public static double closeAngle = 0.1, closeVel = 630, midAngle = 0.65, midVel  = 750, farAngle = 0.5, farVel = 840;

    public enum Dist {
        CLOSE(closeAngle, closeVel),
        MID(midAngle, midVel),
        FAR(farAngle, farVel),
        OFF(0.0, 0.0);

        private double hoodAngle, flywheelVel;

        Dist(double hoodAngle, double flywheelVel) {
            this.hoodAngle = hoodAngle;
            this.flywheelVel = flywheelVel;
        }

        public static void setHoodAngle(Dist dist, double angle) {
            dist.hoodAngle = angle;
        }

        public static void setFlywheelVel(Dist dist, double vel) {
            dist.flywheelVel = vel;
        }
    }

    public void setShooter(Dist mode) {
        setTargetVelocity(mode.flywheelVel);
        setHoodAngle(targetHoodAngle = mode.hoodAngle);
    }

    public void setKicker(double angle){
        kicker.setTargetAngle(angle);
    }

    public boolean atVel() { return Math.abs(targetVelocity - filteredVelocity) <= atVelThresh; }
}
