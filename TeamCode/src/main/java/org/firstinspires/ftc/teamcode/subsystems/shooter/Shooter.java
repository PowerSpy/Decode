package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
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
import org.firstinspires.ftc.teamcode.utils.Pose2d;
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
        TEST
    } public State state = State.IDLE;

    private Robot robot;
    private DcMotorEx ms1, ms2;
    public final PriorityMotor flywheel;
    public final nPriorityServo hood, flywheelBlocker;
    public final PriorityCRServo turret;

    private boolean aimRequest = false, shootRequest = false, stopRequest = false;

    public static PID turretPID = new PID (0.25, 0.0, 0.05);
    public static double turretKStatic = 0.08;
    public static double turretKStaticLeft = 0.08;
    public static double turretKStaticRight = -0.08;
    public static double turretDeadzone = 2;
    public static double turretVelFactor = 0.2;
    private double lastTurretTarget = 0.0;
    public double targetTurretAngle = 0.0;
    private double targetTurretAngleVel = 0.0;
    public static double targetTurretAngleVelFilter = 0.5;
    public double targetHoodAngle = 0.0;
    public static double hoodSweep = Math.toRadians(34.0);
    public static double hoodGearRatio = 48.0 / 30.0;

    // velocity is in inches / second
    public static PID velocityPID = new PID (0.0, 0.0002, 0.0001);
    public static double velocityFFm = 0.00124492;
    public static double velocityFFb = 0.0866734;
    public static double velocityFilterLow = 0.05;
    public static double velocityFilterHigh = 0.5;
    public static double velocityFilterThresh = 60;
    public static double velocityHighPowerThresh = 15;
    public static double velocityNoSkipThresh = 200;
    public static double velocityNoSkipAccel = 0.9;
    public static double flywheelScaleVoltage = 12;
    public static double atVelThresh = 15;
    public static double latchBlockAngle = 2.5;
    private double targetVelocity = 0.0;
    private double filteredVelocity = 0.0;
    private double prevPow = 0;

    // auto-aim
    public final double dLauncher = 3.6 / 2.54;
    public final double g = 9.805 * 100 / 2.54; // gravitational accel in in/s/s
    public final double launcherHeight = 13.5;
    public Vector3 ballTarget, P, V;
    public double a = g * g / 4, c, d, e;
    public double v0, cv0;
    public double minV0 = 0.0, minFlywheelVelocity = 0.0;
    public static double minV0Superthresh = 0; // perhaps eliminate
    public static double ballInterpolateYCloseB = 68;
    public static double ballInterpolateYCloseS = 64;
    public static double ballInterpolateYFar = 63;
    public static double ballInterpolateZCloseB = 44;
    public static double ballInterpolateZCloseS = 40;
    public static double ballInterpolateZFar = 46;
    public static double minV0factorArc = 1.17; // TODO: tune for triple shot
    public static double minV0factorFlat = 1.24; // TODO: tune for triple shot
    public static double flywheelEfficiency = 0.955;
    public static double flywheelEfficiencyConstantFarAddition = -0.02;
    private Pose2d lastPos, currVel, lastVel;
    public static double posFilter = 0.9;
    public static double arcDistThresh = 3200;
    public static int arcFlip = 1;

    /*
    (-71, 48)
    (-48, 64)
    m = (y2 - y1) / (x2 - x1)
    y - y1 = m (x - x1)
    y = m x - m x1 + y1
    */
    private final double wallM = (48.0 - 64.0) / (-71.0 - -48.0);
    private final double wallB = 48 - wallM * -71;

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
            3, 7
        );

        flywheelBlocker = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "flywheelBlocker")},
            "flywheelBlocker", nPriorityServo.ServoType.AXON_MICRO,
            0, 0.7, 0.1,
            new boolean[] {false},
            2, 2
        );

        turret = new PriorityCRServo(
                new CRServo[] {robot.hardwareMap.get(CRServo.class, "turret1"), robot.hardwareMap.get(CRServo.class, "turret2")},
                "turret", PriorityCRServo.ServoType.AXON_MINI,
                new boolean[] {false, false},
                5, 6
        );

        robot.hardwareQueue.addDevices(flywheel, hood, turret, flywheelBlocker);

        updateBallTarget();
        lastVel = currVel = ROBOT_VELOCITY.clone();
        lastPos = ROBOT_POSITION.clone();
    }

    public void update() {
        switch (state) {
            case IDLE: {
                stopRequest = false;

                turretTrackTarget();
                setTargetVelocity(0.0);
                setHoodAngle(0.0);
                setShooterBlocker(true);

                if (aimRequest) {
                    aimRequest = false;
                    state = State.AIMING;
                }
                break;
            }
            case AIMING: {
                aimRequest = false;

                setShooterBlocker(true);
                TelemetryUtil.packet.put("Aim: aimLauncherV8", "before");
                boolean aimResult = aimLauncherV8();
                boolean turretResult = Math.abs(targetTurretAngle - robot.sensors.getTurretAngle()) <= Math.toRadians(ROBOT_POSITION.x >= 24 ? 4 : 1.5);
                TelemetryUtil.packet.put("Aim: aimResult", aimResult);
                TelemetryUtil.packet.put("Aim: turretResult", turretResult);
                if (aimResult && hood.inPosition() && turretResult) {
                    state = State.READY;
                }
                setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                if (stopRequest) {
                    stopRequest = false;
                    aimRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                }
                break;
            }
            case READY: {
                setShooterBlocker(true);

                TelemetryUtil.packet.put("Aim: aimLauncherV8", "before");
                boolean aimResult = aimLauncherV8();
                boolean turretResult = Math.abs(targetTurretAngle - robot.sensors.getTurretAngle()) <= Math.toRadians(ROBOT_POSITION.x >= 24 ? 4 : 1.5);
                TelemetryUtil.packet.put("Aim: aimResult", aimResult);
                TelemetryUtil.packet.put("Aim: turretResult", turretResult);
                if (aimResult) {
                    robot.sensors.light0G.setState(false);
                    robot.sensors.light0P.setState(false);
                }
                setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);
                if (shootRequest && turretResult) {
                    setShooterBlocker(false);
                    if (flywheelBlocker.inPosition()) {
                        state = State.SHOOT;
                        robot.intake.reqShoot(true);
                    }
                }

                if (stopRequest) {
                    stopRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                }
                break;
            }
            case SHOOT: {
                shootRequest = false;

                setShooterBlocker(false);
                aimLauncherV8();
                setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                if (stopRequest) {
                    stopRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                    setTargetVelocity(0.0);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                }
                break;
            }
            case TEST: { // LEAVE THIS EMPTY AT ALL TIMES
                break;
            }
        }

        // Filtering velocity
        lastVel = currVel.clone();
        currVel = ROBOT_VELOCITY.clone();
        currVel.mult(posFilter);
        lastVel.mult(1 - posFilter);
        currVel = Pose2d.add(currVel, lastVel);
        lastVel.mult(1 / (1 - posFilter));
        lastPos = ROBOT_POSITION.clone();
        if (currVel.mag() < 2) {
            currVel.x = 0;
            currVel.y = 0;
        }
        if (Math.abs(currVel.heading) < Math.toRadians(1)) {
            currVel.heading = 0;
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
        double pow = Math.max(0, pidpow + ffpow) * flywheelScaleVoltage / robot.sensors.getVoltage();
        if (error > velocityHighPowerThresh) pow = 1;
        else if (error < -velocityHighPowerThresh) pow = 0;
        if (filteredVelocity < velocityNoSkipThresh) {
            pow = Math.min(pow, prevPow + velocityNoSkipAccel * robot.sensors.loopTime);
        }
        flywheel.setTargetPower(pow);
        prevPow = pow;

        // Turret PID
        //double unclippedTargetTurretAngle = targetTurretAngle;
        targetTurretAngle = Sensors.turretAngleClip(targetTurretAngle);
        targetTurretAngleVel = targetTurretAngleVel * (1 - targetTurretAngleVelFilter) + (targetTurretAngle - lastTurretTarget) / robot.sensors.loopTime * targetTurretAngleVelFilter;
        targetTurretAngleVel = Utils.minMaxClip(targetTurretAngleVel, -150, 150);
        lastTurretTarget = targetTurretAngle;
        double turretAngle = robot.sensors.getTurretAngle();
        double turretError = targetTurretAngle - Sensors.turretAngleClip(turretAngle);
        double turretPow = turretPID.update(turretError, -1, 1);
        if (turretError > 0) turretPow += turretKStaticLeft;
        else if (turretError < 0) turretPow += turretKStaticRight;
        if (Math.abs(turretError) < Math.toRadians(turretDeadzone)) turretPow = 0;
        turretPow += targetTurretAngleVel / (turret.servoType.speed) * turretVelFactor; // meant to account for robot rotating
        if (Math.abs(turretError) > Math.toRadians(60)) turretPow = Math.signum(turretError);
        if (turretAngle >= Sensors.turretLimitLeft) turretPow = Math.min(turretPow, -turretKStatic);
        if (turretAngle <= Sensors.turretLimitRight) turretPow = Math.max(turretPow, turretKStatic);
        turretPow = Utils.minMaxClip(turretPow, -1, 1);
        turret.setTargetPower(turretPow);

        Log.i("Shooter","Robot Velocity" + (this.V != null ? this.V.getMag() : 0));
        TelemetryUtil.packet.put("Shooter : Robot Velocity", (this.V != null ? this.V.getMag() : 0));
        TelemetryUtil.packet.put("Shooter : currVel x", currVel.x);
        TelemetryUtil.packet.put("Shooter : currVel y", currVel.y);
        TelemetryUtil.packet.put("Shooter : currVel heading (deg)", Math.toDegrees(currVel.heading));
        TelemetryUtil.packet.put("Shooter : targetAngleVel (deg)", Math.toDegrees(targetTurretAngleVel));
        TelemetryUtil.packet.put("Shooter : state", this.state);
        TelemetryUtil.packet.put("Shooter : Flywheel Power Applied", pow * 100);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Turret Target (deg)", Math.toDegrees(targetTurretAngle));
        TelemetryUtil.packet.put("Shooter : Hood Target (deg)", Math.toDegrees(hood.getTargetAngle()));
        TelemetryUtil.packet.put("Shooter : Turret Power Applied", turretPow * 100);
        TelemetryUtil.packet.put("Shooter : Balltarget", ballTarget.toString());
        LogUtil.flywheelTarget.set(targetVelocity);
        LogUtil.shooterState.set(this.state.toString());
        LogUtil.turretTarget.set(targetTurretAngle);
        LogUtil.hoodAngle.set(hood.getTargetAngle());
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        canvas.setStroke("#808080");
        canvas.setStrokeWidth(1);
        canvas.strokeLine(ballTarget.x, ballTarget.y, ROBOT_POSITION.x, ROBOT_POSITION.y);
        canvas.setStroke(Globals.isRed ? "#ff0000" : "#0000ff");
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(ballTarget.x, ballTarget.y, 2.5);
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

    public void setTurretAngle (double targetAngle) {
        targetTurretAngle = targetAngle;
    }

    public void setHoodAngle(double target_angle) { hood.setTargetAngle(target_angle); }

    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }

    public double getTargetVelocity() { return targetVelocity; }

    public double getFilteredVelocity() { return filteredVelocity; }

    public void setShooterBlocker(boolean active) { flywheelBlocker.setTargetAngle(active ? latchBlockAngle : -0.2);}

    public void updateBallTarget() {
        ballTarget = new Vector3(-68, 67 * (Globals.isRed ? 1 : -1), 46);
    }

    public void updateBallTargetInterpolate() {
        if (ROBOT_POSITION.x >= 24) ballTarget = new Vector3(-68, ballInterpolateYFar * (Globals.isRed ? 1 : -1), ballInterpolateZFar);
        else {
            double k = Utils.minMaxClip(Math.hypot(-71 - ROBOT_POSITION.x, 71 * (Globals.isRed ? 1 : -1) - ROBOT_POSITION.y), 0, 126) / 126;
            ballTarget = new Vector3(-68, (ballInterpolateYCloseS * k + ballInterpolateYCloseB * (1 - k)) * (Globals.isRed ? 1 : -1), ballInterpolateZCloseS * k + ballInterpolateZCloseB * (1 - k));
        }
    }

    public void turretTrackTarget() {
        if (ROBOT_POSITION == null || ROBOT_VELOCITY == null) return;
        // for +-180 turret
        updateBallTargetInterpolate();
        Vector3 P;
        if (ROBOT_POSITION.x + 24 >= ROBOT_POSITION.y * (Globals.isRed ? -1 : 1)) P = new Vector3(ballTarget);
        else P = new Vector3(ballTarget.y * (Globals.isRed ? -1 : 1), ballTarget.x * (Globals.isRed ? -1 : 1), ballTarget.z); // invert target along y = x or y = -x
        P.subtract(new Vector3(ROBOT_POSITION.x, ROBOT_POSITION.y, launcherHeight));
        this.P = P;
        targetTurretAngle = AngleUtil.clipAngle(Math.atan2(P.getY(), P.getX()) - ROBOT_POSITION.heading);
    }

    public boolean aimLauncherV8() {
        if (ROBOT_POSITION == null || ROBOT_VELOCITY == null) {
            TelemetryUtil.packet.put("Aim: aimLauncherV8", "no position");
            return false;
        }
        Log.i("Points", "Starting aimLauncherV8");
        turretTrackTarget();
        Vector3 V = new Vector3(-currVel.x, -currVel.y, 0);
        V.subtract(Vector3.cross(new Vector3(0, 0, currVel.heading), new Vector3(dLauncher * Math.cos(ROBOT_POSITION.heading), dLauncher * Math.sin(ROBOT_POSITION.heading), 0)));
        if (V.getMag() < 6) V = new Vector3(0, 0, 0);
        this.V = V;
        Log.i("Points", "Set target turret angle & Starting MinV0");

        a = g * g / 4;
        // double b = 0;
        c = V.x * V.x + V.y * V.y + g * P.z;
        d = 2 * Vector3.dot(P, V);
        e = P.x * P.x + P.y * P.y + P.z * P.z;
        List<Double> tRoots = Polynomial.findRealRoots(new double[]{1, 0.0, 0.0, -d/(2 * a), -e/a}, 1e-4);
        for (int i = 0; i < tRoots.size(); i++) {
            if(tRoots.get(i) < 0) {
                tRoots.remove(i);
                i--;
            }
        }
        Log.i("MinV0", "Num Roots: " + tRoots.size());
        if (tRoots.isEmpty()) {
            Log.i("Points", "Shot dies in MinV0, tRoots is empty");
            TelemetryUtil.packet.put("Aim: aimLauncherV8", "Shot dies in MinV0, tRoots is empty");
            return false;
        }
        Log.i("MinV0", "tRoots[0]: " + tRoots.get(0));

        double dist2 = e - P.z * P.z; // 2D dist squared
        minV0 = (Math.sqrt(2 * a * tRoots.get(0) * tRoots.get(0) + c + d / 2 / tRoots.get(0)) + minV0Superthresh);
        if (arcFlip == 1) minV0 *= minV0factorFlat;
        else minV0 *= minV0factorArc;
        if (ROBOT_POSITION.x >= 24) minFlywheelVelocity = minV0 * 2 / (flywheelEfficiency + flywheelEfficiencyConstantFarAddition);
        else minFlywheelVelocity = minV0 * 2 / flywheelEfficiency;
        Log.i("MinV0", "minV0: " + minV0);
        Log.i("MinV0", "min flywheel:" + minFlywheelVelocity);

        double v0 = getBallExitSpd();
        Log.i("Points", "Got past MinV0, v0 = " + v0);

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();

        double[] thetas;
        double[] phis;
        double savedFlightTime = 0.0;
        if (v0 > 120)  {
            Log.i("Points", "Entering the hood calculator");
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
                TelemetryUtil.packet.put("Aim: aimLauncherV8", "Shot dies in Dynamic, tRoots is empty");
                return false;
            }
            StringBuilder roots = new StringBuilder("[");
            for (int i = 0; i < tRoots.size(); i++) {
                roots.append(tRoots.get(i));
                if (i != tRoots.size() - 1) roots.append(", ");
            }
            Log.i("Dynamic", "Roots: " + roots.toString() + "]");
            phis = new double[tRoots.size() + 1];
            thetas = new double[phis.length];
            arcFlip = (dist2 < arcDistThresh ? 1 : -1);
            for (int i = 0; i < tRoots.size(); i++) {
                double t0 = tRoots.get(i);
                Vector3 pf = new Vector3(P.x + V.x * t0, P.y + V.y * t0, P.z + g * t0 * t0 / 2);
                thetas[i] = pf.theta();
                phis[i] = pf.phi();

                Log.i("Dynamic", "Flip val = " + arcFlip + ", " + (arcFlip == 1 ? "arc shot" : "flat shot"));
                if (phis[i] - hoodSweep <= 0) {
                    Log.i("Dynamic", "Point 4: i = " + i + ", phis[i] = " + phis[i]);
                    phis[i] = 100 * arcFlip;
                }
                if (i == 0) {
                    thetas[tRoots.size()] = thetas[0];
                    phis[tRoots.size()] = phis[0];
                    savedFlightTime = t0;
                } else {
                    if (arcFlip == 1) {
                        if (phis[i] < phis[tRoots.size()]) {
                            phis[tRoots.size()] = phis[i];
                            thetas[tRoots.size()] = thetas[i];
                            savedFlightTime = t0;
                        }
                    } else {
                        if (phis[i] > phis[tRoots.size()]) {
                            phis[tRoots.size()] = phis[i];
                            thetas[tRoots.size()] = thetas[i];
                            savedFlightTime = t0;
                        }
                    }
                }
                targetTurretAngle = AngleUtil.clipAngle(thetas[tRoots.size()] - ROBOT_POSITION.heading);
                Log.i("Dynamic", "High Phi: " + phis[i]);
                Log.i("Points", "Leaving Dynamic");
            }
        } else {
            Log.i("Points", "Shot isn't possible, v0 is sub-120 in/s");
            TelemetryUtil.packet.put("Aim: aimLauncherV8", "Shot isn't possible, v0 is sub-120 in/s");
            return false;
        }

        if (phis[tRoots.size()] == -100) {
            Log.i("Points", "Shot not possible; Phis are cooked, phis[" + tRoots.size() + "] = -100");
            TelemetryUtil.packet.put("Aim: aimLauncherV8", "Phis are cooked, phis[" + tRoots.size() + "] = -100");
            return false;
        }
        targetHoodAngle = (phis[tRoots.size()] - hoodSweep) * hoodGearRatio; // subtracts the sweep of the hood
        Log.i("Points", "The shot is possible, returning true, time: " + savedFlightTime + ", flat?: " + arcFlip);
        TelemetryUtil.packet.put("Aim: aimLauncherV8", "The shot is possible and we're returning true!!");
        return true;
    }

    public boolean aimLauncherV8(Vector3 target) {
        Vector3 bubble = ballTarget.clone();
        ballTarget = target.clone();
        boolean res = aimLauncherV8();
        ballTarget = bubble;
        return res;
    }

    public double getBallExitSpd() {
        double res = filteredVelocity * 0.5;
        if (ROBOT_POSITION.x >= 24) res *= flywheelEfficiency + flywheelEfficiencyConstantFarAddition;
        else res *= flywheelEfficiency;
        return res;
    }

    // further separation :)
    // bootleg LM1 strat being used in LM2 & LM3 code
    public static double closeAngle = 0.2, closeVel = 470, midAngle = 0.5, midVel = 550, farAngle = 0.45, farVel = 625;

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

        public static void setHoodAngle(Dist dist, double angle) { dist.hoodAngle = angle; }

        public static void setFlywheelVel(Dist dist, double vel) { dist.flywheelVel = vel; }
    }

    public void setShooter(Dist mode) {
        setTargetVelocity(mode.flywheelVel);
        setHoodAngle(targetHoodAngle = mode.hoodAngle);
    }

    public boolean atVel() { return Math.abs(targetVelocity - filteredVelocity) <= atVelThresh; }
}
