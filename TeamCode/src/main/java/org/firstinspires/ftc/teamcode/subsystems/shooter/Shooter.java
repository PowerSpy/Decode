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
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.Vector2;
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

    private final Robot robot;
    public final Flywheel flywheel;
    public final Turret turret;
    public final nPriorityServo hood, flywheelBlocker;
    private ShotTable shooterTable;

    private boolean aimRequest = false, shootRequest = false, stopRequest = false;
    public boolean turretTrackInManual = false;

    public double targetHoodAngle = 0.0;
    public static double hoodSweep = Math.toRadians(26.43);
    public static double hoodGearRatio = 20.0 / 40.0 * 254.0 / 35.0;

    public static double latchBlockAngle = 1;

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
        shooterTable.addSetpoint(39.9, new ShotSetpoint(400,Math.toRadians(27)));
        shooterTable.addSetpoint(51.9, new ShotSetpoint(450,Math.toRadians(34)));
        shooterTable.addSetpoint(61.3, new ShotSetpoint(480,Math.toRadians(42)));
        shooterTable.addSetpoint(81.5, new ShotSetpoint(520,Math.toRadians(48)));
        shooterTable.addSetpoint(95.6, new ShotSetpoint(520,Math.toRadians(51)));
        shooterTable.addSetpoint(129, new ShotSetpoint(605,1.5));
        /*
        shooterTable.addSetpoint(40.3, new ShotSetpoint(430,0));
        shooterTable.addSetpoint(43.6, new ShotSetpoint(440,0.05));
        shooterTable.addSetpoint(48.5, new ShotSetpoint(465,0.2));
        shooterTable.addSetpoint(52.9, new ShotSetpoint(470,0.25));
        shooterTable.addSetpoint(65, new ShotSetpoint(545,0.5));
        shooterTable.addSetpoint(76.4, new ShotSetpoint(555,0.5 ));
        shooterTable.addSetpoint(85, new ShotSetpoint(585,0.55 ));
        shooterTable.addSetpoint(94.8, new ShotSetpoint(600,0.6));
        shooterTable.addSetpoint(136.9, new ShotSetpoint(580,1.4 ));
        shooterTable.addSetpoint(148, new ShotSetpoint(700,0.75));
        */

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
                flywheel.setTargetVelocity(0.0);
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
                if (turretResult && this.atVel()) {
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
                if (shootRequest) {
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
    public static double closeAngle = 0.2, closeVel = 470, midAngle = 0.5, midVel = 545, farAngle = 1.4, farVel = 580;

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
        flywheel.setTargetVelocity(mode.flywheelVel);
        setHoodAngle(targetHoodAngle = mode.hoodAngle);
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
        ballTarget = new Vector3(-68,68,42);
        // Initial values based on the target
        double initialDist = Math.hypot(ballTarget.x - ROBOT_POSITION.x, ballTarget.y - ROBOT_POSITION.y);
        ShotSetpoint values = shooterTable.getSetpoint(initialDist);

        // Setting initial goal for the virtual
        double virtualX = ballTarget.x;
        double virtualY = ballTarget.y;

        // Looping through virtual goal
        //getting time of flight
        double time = initialDist / (values.flywheelVel / 2 * Math.sin(values.hoodAngle));

        // Offset the virtual goal by the robot's velocity during flight
        if (Math.hypot(ROBOT_VELOCITY.x,ROBOT_VELOCITY.y) > 10) {
            virtualX = ballTarget.x - (ROBOT_VELOCITY.x * time);
            virtualY = ballTarget.y - (ROBOT_VELOCITY.y * time);
        }
        //Calculate distance to virtual goal
        double virtualDist = Math.hypot(virtualX - ROBOT_POSITION.x, virtualY - ROBOT_POSITION.y);
        // Get new shooter values from the regression based off of virtual distance
        values = shooterTable.getSetpoint(virtualDist);

        // Outputting final result
        if (Math.hypot(ROBOT_VELOCITY.x,ROBOT_VELOCITY.y) > 10) {
            Vector2 goalUnitVector = new Vector2((virtualX - ROBOT_POSITION.x) / virtualDist, (virtualY-ROBOT_POSITION.y) / virtualDist);
            double robotVelocityGoal = (ROBOT_VELOCITY.x * goalUnitVector.x) + (ROBOT_VELOCITY.y * goalUnitVector.y);
            double ballVelocity = flywheel.getFilteredVelocity() / 2 * Math.sin(values.hoodAngle) + robotVelocityGoal;
            double g = 386.088; // in/s
            minFlywheelVelocity = values.flywheelVel;
            targetHoodAngle = Math.atan((
                ballVelocity * ballVelocity + Math.sqrt(
                    Math.pow(ballVelocity, 4) - g * (
                        g * virtualDist * virtualDist
                        + 2 * (ballTarget.z - launcherHeight) * ballVelocity * ballVelocity
                    )
                )
            ) / (g * virtualDist));
        } else {
            targetHoodAngle = values.hoodAngle;
            minFlywheelVelocity = values.flywheelVel;
        }
        TelemetryUtil.packet.put("Shooter : target hood launch angle (deg)", Math.toDegrees(targetHoodAngle));
        targetHoodAngle = Utils.minMaxClip((targetHoodAngle - hoodSweep) * hoodGearRatio, 0, 1.6);
        double virtualTurretAngle = Math.atan2(virtualY - ROBOT_POSITION.y, virtualX - ROBOT_POSITION.x);
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
