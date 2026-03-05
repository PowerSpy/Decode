package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_GLOBAL_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.Polynomial;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;
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

    private final Robot robot;
    public final Flywheel flywheel;
    public final Turret turret;
    public final nPriorityServo hood, flywheelBlocker;
    private final ShotTable shooterTable;

    private boolean aimRequest = false, shootRequest = false, stopRequest = false;
    public boolean turretTrackInManual = false;

    public double targetHoodAngle = 0.0;
    public static double hoodSweep = Math.toRadians(26.43);
    public static double hoodGearRatio = 20.0 / 40.0 * 254.0 / 35.0;

    public static double latchBlockAngle = 1, latchOpenAngle = 0.2;

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
    public static double ballInterpolateYFar = 66;
    public static double ballInterpolateZCloseB = 44;
    public static double ballInterpolateZCloseS = 40;
    public static double ballInterpolateZFar = 45;
    public static double minV0factorArc = 1.2; // TODO: tune for triple shot
    public static double minV0factorFlat = 1.24; // TODO: tune for triple shot
    public static double flywheelEfficiency = 0.955;
    public static double flywheelEfficiencyConstantFarAddition = -0.02;
    private Pose2d lastPos, currVel, lastVel;
    public static double posFilter = 0.9;
    public static double arcDistThresh = 5000;

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

        this.flywheel = new Flywheel(robot);
        this.turret = new Turret(robot);

        this.shooterTable = new ShotTable();
        // Add: addSetpoint(distanceInches(from goal), new ShotSetpoint(flywheelVel, hoodPos, timeOfFlight(seconds)))
        //shooterTable.addSetpoint(39.9, new ShotSetpoint(400,Math.toRadians(27)));
        //shooterTable.addSetpoint(51.9, new ShotSetpoint(450,Math.toRadians(34)));
        //shooterTable.addSetpoint(61.3, new ShotSetpoint(480,Math.toRadians(42)));
        shooterTable.addSetpoint(42.8, new ShotSetpoint(410,Math.toRadians(28)));
        shooterTable.addSetpoint(57.5, new ShotSetpoint(450,Math.toRadians(32)));
        shooterTable.addSetpoint(69.3, new ShotSetpoint(472,Math.toRadians(31)));
        shooterTable.addSetpoint(75.9, new ShotSetpoint(485,Math.toRadians(32)));
        shooterTable.addSetpoint(98, new ShotSetpoint(528,Math.toRadians(36)));
        shooterTable.addSetpoint(114.5, new ShotSetpoint(560,Math.toRadians(37)));
        shooterTable.addSetpoint(132.6, new ShotSetpoint(615,Math.toRadians(47)));
        shooterTable.addSetpoint(142.6, new ShotSetpoint(635,Math.toRadians(50)));
        shooterTable.addSetpoint(150.7, new ShotSetpoint(645,Math.toRadians(48)));





        hood = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "hood1")},
            "hood", nPriorityServo.ServoType.AXON_MINI,
            0.03, 0.33, 0.03,
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

        robot.hardwareQueue.addDevices(hood, flywheelBlocker);

        updateBallTarget();
        lastVel = currVel = ROBOT_VELOCITY.clone();
        lastPos = ROBOT_POSITION.clone();
    }

    public void update() {
        switch (state) {
            case IDLE: {
                stopRequest = false;

                turretTrackTarget();
                flywheel.setTargetVelocity(Dist.CLOSE.flywheelVel);
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
                //TelemetryUtil.packet.put("Aim: aimLauncherV8", "before");
                //boolean aimResult = aimLauncherV8();
                predictGoal();
                boolean turretResult = turret.inPosition();
                //TelemetryUtil.packet.put("Aim: aimResult", aimResult);
                TelemetryUtil.packet.put("Aim: turretResult", turretResult);
                TelemetryUtil.packet.put("Aim: hood.inPosition", hood.inPosition());
                TelemetryUtil.packet.put("Aim: atVel", atVel());
                if (turretResult && this.atVel() && hood.inPosition()) {
                    state = State.READY;
                }
                flywheel.setTargetVelocity(minFlywheelVelocity);
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

                //TelemetryUtil.packet.put("Aim: aimLauncherV8", "before");
                //boolean aimResult = aimLauncherV8();
                //TelemetryUtil.packet.put("Aim: aimResult", aimResult);
                predictGoal();
                /*
                if (aimResult && Globals.RUNMODE != RunMode.AUTO) {
                    robot.sensors.light0G.set(true);
                    robot.sensors.light0P.set(true);
                }
                */
                flywheel.setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                if (shootRequest /* && isRobotInZone(0,0,-72,72,-72,-72) || isRobotInZone(48,0,72,24,72,-24)*/) {
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
                    flywheel.setTargetVelocity(0.0);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                }
                break;
            }
            case SHOOT: {
                shootRequest = false;
                predictGoal();
                setShooterBlocker(false);
                //aimLauncherV8();
                flywheel.setTargetVelocity(minFlywheelVelocity);
                setHoodAngle(targetHoodAngle);

                if (stopRequest) {
                    stopRequest = false;
                    shootRequest = false;
                    state = State.IDLE;
                    flywheel.setTargetVelocity(0.0);
                    robot.intake.reqShoot(false);
                    robot.intake.reqOff(true);
                }
                /*else if (!isRobotInZone(0,0,-72,72,-72,-72) && !isRobotInZone(48,0,72,24,72,-24)) {
                    state = State.AIMING;
                    robot.intake.reqShoot(false);
                    robot.intake.reqIntake(true);
                    robot.intake.setRollerDirection(false);
                }*/

                break;
            }
            case TEST: { // LEAVE THIS EMPTY AT ALL TIMES
                if (turretTrackInManual) turretTrackTarget();
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

        flywheel.update();

        turret.update();

        Log.i("Shooter","Robot Velocity" + (this.V != null ? this.V.getMag() : 0));
        TelemetryUtil.packet.put("Shooter : Robot Velocity", (this.V != null ? this.V.getMag() : 0));
        TelemetryUtil.packet.put("Shooter : currVel x", currVel.x);
        TelemetryUtil.packet.put("Shooter : currVel y", currVel.y);
        TelemetryUtil.packet.put("Shooter : currVel heading (deg)", Math.toDegrees(currVel.heading));
        TelemetryUtil.packet.put("Shooter : state", this.state);
        TelemetryUtil.packet.put("Shooter : Hood Target (deg)", Math.toDegrees(hood.getTargetAngle()));
        TelemetryUtil.packet.put("Shooter : Balltarget", ballTarget.toString());
        TelemetryUtil.packet.put("Shooter : goal distance", Math.hypot(ROBOT_POSITION.x - robot.shooter.ballTarget.x, ROBOT_POSITION.y - robot.shooter.ballTarget.y));
        TelemetryUtil.packet.put("Shooter : robot in Zone", isRobotInZone(0,0,-72,72,-72,-72) || isRobotInZone(48,0,72,24,72,-24));
        LogUtil.shooterState.set(this.state.toString());
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
            turret.setTargetAngle(0);
            setShooter(Dist.OFF);
        } else {
            state = State.IDLE;
        }
    }

    public void setHoodAngle(double target_angle) { hood.setTargetAngle(target_angle); }

    public void setShooterBlocker(boolean active) { flywheelBlocker.setTargetAngle(active ? latchBlockAngle : latchOpenAngle);}

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
        if (ROBOT_POSITION.x + 48 >= ROBOT_POSITION.y * (Globals.isRed ? -1 : 1)) P = new Vector3(ballTarget);
        else P = new Vector3(ballTarget.y * (Globals.isRed ? -1 : 1), ballTarget.x * (Globals.isRed ? -1 : 1), ballTarget.z); // invert target along y = x or y = -x
        P.subtract(new Vector3(ROBOT_POSITION.x, ROBOT_POSITION.y, launcherHeight));
        this.P = P;
        turret.setTargetAngle(Math.atan2(P.getY(), P.getX()) - ROBOT_POSITION.heading);
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
        double arcFlip = (dist2 < arcDistThresh ? 1 : -1);
        minV0 = (Math.sqrt(2 * a * tRoots.get(0) * tRoots.get(0) + c + d / 2 / tRoots.get(0)) + minV0Superthresh);
        if (arcFlip == 1 && ROBOT_POSITION.x >= 24) minV0 *= minV0factorFlat;
        //else if (arcFlip == 1) minV0*= minV0factorFlat; use if the close flat shot is jank to have 2 constants
        else minV0 *= minV0factorArc;
        if (ROBOT_POSITION.x >= 24) minFlywheelVelocity = minV0 * 2 / (flywheelEfficiency + flywheelEfficiencyConstantFarAddition);
        else minFlywheelVelocity = minV0 * 2 / flywheelEfficiency;
        Log.i("MinV0", "minV0: " + minV0);
        Log.i("MinV0", "min flywheel:" + minFlywheelVelocity);

        double v0 = getBallExitSpd();
        Log.i("Points", "Got past MinV0, v0 = " + v0);

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
                turret.setTargetAngle(thetas[tRoots.size()] - ROBOT_POSITION.heading);
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
        double res = flywheel.getFilteredVelocity() * 0.5;
        if (ROBOT_POSITION.x >= 24) res *= flywheelEfficiency + flywheelEfficiencyConstantFarAddition;
        else res *= flywheelEfficiency;
        return res;
    }

    public boolean atVel() { return flywheel.atVel(); }

    // further separation :)
    // bootleg LM1 strat being used in LM2 & LM3 code

    public enum Dist {
        CLOSE(0.5, 450),
        AUTO_POSITION(0.6, 460),
        MID(1.35, 520),
        FAR(1.3, 610),
        OFF(0.0, 0.0);

        private final double hoodAngle, flywheelVel;

        Dist(double hoodAngle, double flywheelVel) {
            this.hoodAngle = hoodAngle;
            this.flywheelVel = flywheelVel;
        }
    }

    public void setShooter(Dist mode) {
        flywheel.setTargetVelocity(mode.flywheelVel);
        setHoodAngle(targetHoodAngle = mode.hoodAngle);
    }

    public static boolean isRobotInZone(double x1, double y1, double x2, double y2, double x3, double y3) {

        //getting robot pose info
        double theta = Math.toRadians(ROBOT_POSITION.heading);
        double cosT = Math.cos(theta);
        double sinT = Math.sin(theta);
        double l = Globals.ROBOT_LENGTH;
        double w = Globals.ROBOT_WIDTH;
        double rx = ROBOT_POSITION.x;
        double ry = ROBOT_POSITION.y;

        //Four corners of robot - relative to robot
        double[][] corners = {
                {l/2, w/2}, {l/2, -w/2}, {-l/2, w/2}, {-l/2, -w/2}, {0, 0}
        };

        //Calculate triangle denom
        double denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);

        //Transform the corners of robot to global coordinates and check weights
        for (double[] c : corners) {
            // Rotation + Translation of each corner
            double px = rx + (c[0] * cosT - c[1] * sinT);
            double py = ry + (c[0] * sinT + c[1] * cosT);

            // Barycentric weight calculation
            double w1 = ((y2 - y3) * (px - x3) + (x3 - x2) * (py - y3)) / denom;
            double w2 = ((y3 - y1) * (px - x3) + (x1 - x3) * (py - y3)) / denom;
            double w3 = 1.0 - w1 - w2;

            //none of the triangle areas are negative meaning robot is inside the triangle
            if (w1 >= 0 && w2 >= 0 && w3 >= 0) {
                return true;
            }
        }

        return false;
    }

    public void predictGoal() {
        /*
        if (ROBOT_POSITION.x >= 24) ballTarget = new Vector3(-68, ballInterpolateYFar * (Globals.isRed ? 1 : -1), ballInterpolateZFar);
        else {
            double k = Utils.minMaxClip(Math.hypot(-71 - ROBOT_POSITION.x, 71 * (Globals.isRed ? 1 : -1) - ROBOT_POSITION.y), 0, 126) / 126;
            ballTarget = new Vector3(-68, (ballInterpolateYCloseS * k + ballInterpolateYCloseB * (1 - k)) * (Globals.isRed ? 1 : -1), ballInterpolateZCloseS * k + ballInterpolateZCloseB * (1 - k));
        }
        if (ROBOT_POSITION.x + 48 <= ROBOT_POSITION.y * (Globals.isRed ? -1 : 1)) ballTarget = new Vector3(ballTarget.y * (Globals.isRed ? -1 : 1), ballTarget.x * (Globals.isRed ? -1 : 1), ballTarget.z);
        // Original target
        */
        if (ROBOT_POSITION.x > 24) {
            ballTarget = new Vector3(-70,71 * (Globals.isRed ? 1 : -1),42);
        } else {
            ballTarget = new Vector3(-68,69 * (Globals.isRed ? 1 : -1),42);
        }
        // Initial values based on the target
        double initialDist = Math.hypot(ballTarget.x - ROBOT_POSITION.x, ballTarget.y - ROBOT_POSITION.y);
        ShotSetpoint values = shooterTable.getSetpoint(initialDist);

        // Setting initial goal for the virtual
        double virtualX = ballTarget.x;
        double virtualY = ballTarget.y;
        /*
        if(ROBOT_POSITION.x < - 32) {

            double temp = virtualY;
            virtualY = virtualX * (Globals.isRed ? -1 : 1);
            virtualX = temp * (Globals.isRed ? -1 : 1);
        }

         */

        // Looping through virtual goal
        //getting time of flight
        double time = initialDist / (values.flywheelVel / 2 * Math.sin(values.hoodAngle));
        double sotmCompensation = 0;

        TelemetryUtil.packet.put("Shooter : Robot Velocity", Math.hypot(ROBOT_GLOBAL_VELOCITY.x, ROBOT_GLOBAL_VELOCITY.y));

        // Offset the virtual goal by the robot's velocity during flight
        if (Math.hypot(ROBOT_GLOBAL_VELOCITY.x, ROBOT_GLOBAL_VELOCITY.y) > 10) {
            virtualX = ballTarget.x - (ROBOT_GLOBAL_VELOCITY.x * time);
            virtualY = ballTarget.y - (ROBOT_GLOBAL_VELOCITY.y * time);
            Canvas canvas = TelemetryUtil.packet.fieldOverlay();
            canvas.setStroke(Globals.isRed ? "#ff4000" : "#0040ff");
            canvas.setStrokeWidth(2);
            canvas.strokeCircle(virtualX, virtualY, 2.5);

            //Calculate distance to virtual goal
            double virtualDist = Math.hypot(virtualX - ROBOT_POSITION.x, virtualY - ROBOT_POSITION.y);
            // Get new shooter values from the regression based off of virtual distance
            values = shooterTable.getSetpoint(virtualDist);

            // Outputting final result
            Vector2 goalUnitVector = new Vector2((virtualX - ROBOT_POSITION.x) / virtualDist, (virtualY - ROBOT_POSITION.y) / virtualDist);
            double robotVelocityGoal = (ROBOT_GLOBAL_VELOCITY.x * goalUnitVector.x) + (ROBOT_GLOBAL_VELOCITY.y * goalUnitVector.y);
            double ballVelocity = flywheel.getFilteredVelocity() / 2 * Math.sin(values.hoodAngle) + robotVelocityGoal;
            TelemetryUtil.packet.put("Aim : robotVelocityGoal", robotVelocityGoal);
            TelemetryUtil.packet.put("Aim : ballVelocity", ballVelocity);
            double g = 386.088; // in/s
            minFlywheelVelocity = values.flywheelVel;
            double d = Math.pow(ballVelocity, 4) - g * (g * virtualDist * virtualDist + 2 * (ballTarget.z - launcherHeight) * ballVelocity * ballVelocity);
            TelemetryUtil.packet.put("Aim : d", d);
            if (d >= 0) {
                targetHoodAngle = Math.atan((ballVelocity * ballVelocity + Math.sqrt(d)) / (g * virtualDist));
                TelemetryUtil.packet.put("Aim : branch", "SOTM found hood angle");
            } else {
                targetHoodAngle = values.hoodAngle;
                TelemetryUtil.packet.put("Aim : branch", "SOTM failed");
            }
            if (ROBOT_POSITION.x < -60) {
                sotmCompensation = Math.toRadians(5) * (Globals.isRed ? -1 : 1);
            }
        } else if (ROBOT_POSITION.x > 24) {
            double ballVelocity = flywheel.getFilteredVelocity() / 2 * Math.sin(values.hoodAngle);
            sotmCompensation = 0;
            TelemetryUtil.packet.put("Aim : ballVelocity", ballVelocity);
            double g = 386.088; // in/s
            initialDist += 6;
            double d = Math.pow(ballVelocity, 4) - g * (g * initialDist * initialDist + 2 * (ballTarget.z - launcherHeight) * ballVelocity * ballVelocity);
            TelemetryUtil.packet.put("Aim : d", d);
            if (d >= 0) {
                targetHoodAngle = Math.atan((ballVelocity * ballVelocity + Math.sqrt(d)) / (g * initialDist));
                TelemetryUtil.packet.put("Aim : branch", "far shot hound hood angle");
            } else {
                targetHoodAngle = values.hoodAngle;
                TelemetryUtil.packet.put("Aim : branch", "far shot failed");
            }
        } else {
            targetHoodAngle = values.hoodAngle;
            minFlywheelVelocity = values.flywheelVel;
            sotmCompensation = 0;
            TelemetryUtil.packet.put("Aim : branch", "static");
        }
        TelemetryUtil.packet.put("Aim : target hood launch angle (deg)", Math.toDegrees(targetHoodAngle));
        targetHoodAngle = Utils.minMaxClip((targetHoodAngle - hoodSweep) * hoodGearRatio, 0, 1.6);
        double virtualTurretAngle = Math.atan2(virtualY - ROBOT_POSITION.y, virtualX - ROBOT_POSITION.x) + sotmCompensation;
        turret.setTargetAngle(virtualTurretAngle - ROBOT_POSITION.heading);
    }

    /*public void shootRegression() {
        double dist = Math.hypot(ballTarget.x - ROBOT_POSITION.x, ballTarget.y - ROBOT_POSITION.y);
        ShotSetpoint target = shooterTable.getSetpoint(dist);
        if (target != null) {
            minFlywheelVelocity = target.flywheelVel;
            targetHoodAngle = target.hoodAngle;
        }
    }*/
}
