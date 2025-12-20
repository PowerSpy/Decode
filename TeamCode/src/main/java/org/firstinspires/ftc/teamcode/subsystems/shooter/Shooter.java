package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
import static org.firstinspires.ftc.teamcode.utils.Globals.blueTag;
import static org.firstinspires.ftc.teamcode.utils.Globals.redTag;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Complex;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Polynomial;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

import java.util.ArrayList;
import java.util.List;

@Config
public class Shooter {
    public enum State {
        IDLE,
        ACCEL,
        SHOOT,
        INDEX
    } State state = State.IDLE;

    private final Robot robot;
    private final Sensors sensors;
    private final DcMotorEx ms1, ms2;
    public final PriorityMotor flywheel;
    public final nPriorityServo turret, hood, flywheelBlocker, net;

    public static ArrayList<Long> nanoTimes;
    public static ArrayList<Double> turretHistory;

    private boolean indexPrepareRequest = false, indexRequest = false;
    private boolean shootPrepareRequest = false, shootRequest = false;

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

    // autoaim stuff
    private final double dLauncher = Math.sqrt(66.632 * 66.632 + 229.61 * 229.61) / 25.4;
    private final double g = 9.805 * 100 / 2.54;
    private final double launcherHeight = 330.14203 / 25.4;
    public Vector3 ballTarget = new Vector3(-60, 60, 38.75 + 3); // +3 for safety of ball going in, clearance = 3 - 2.5 = 0.5
    public Vector3 distance = new Vector3(0,0,0);
    public Vector3 ballExit2DSpd = new Vector3(0, 0, 0);
    public Vector3 tVel  = new Vector3(0, 0, 0);
    public Vector3 rVel = new Vector3(0, 0, 0);
    public Vector3 vel = new Vector3(0, 0, 0);
    private final PID turretPID = new PID(0.5, 0.01, 0.01);
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
            0.0, 0.41, 0.412,
            new boolean[] {false},
            2, 5
        );

        turret = new nPriorityServo(
            new Servo[]{robot.hardwareMap.get(Servo.class, "turret1"), robot.hardwareMap.get(Servo.class,"turret2")},
            "turret", nPriorityServo.ServoType.AXON_MINI,
            0, 1.0, 0.5, // TODO: find out where in the servo is straight ahead
            new boolean[] {false, false},
            2, 5
        );

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
                0, 1.0, 0.5,
                new boolean [] {false},
                2, 5
        );

        robot.hardwareQueue.addDevices(flywheel, hood, turret, flywheelBlocker, net);

        flywheel.motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double error;

    public void update() {
        switch (state){
            case IDLE:
                if(shootPrepareRequest){
                    // Calculation for targetVelocity goes here
                    state = State.ACCEL;
                    aimLauncherV7(); // starts autoaim going
                }

                if(indexPrepareRequest){
                    // Calculation for targetVelocity goes here
                    state = State.ACCEL;
                }
                break;
            case ACCEL:
                if((atVel() && shootRequest) && aimLauncherV7()){ // aimLauncherV7 will attempt to aim the launcher if (atVel() && shootRequest), but will stop the fsm from entering Shoot if a shot isn't possible
                    state = State.SHOOT;
                }

                if(atVel() && indexRequest){
                    state = State.INDEX;
                }
                break;
            case SHOOT:
                aimLauncherV7(); // right up until the shot happens, the hood and turret should be recalculated as often as possible
                break;
            case INDEX:
                setShooterBlocker(false);

                // TODO: Probably need to reorganize this to better differentiate the two actions, especially because indexing and shooting will launch diff number of balls
                break;
        }


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

        setTurretAngle(Math.tan((ROBOT_POSITION.y - (Globals.isRed ? redTag.y : blueTag.y)) / (ROBOT_POSITION.x - (Globals.isRed ? redTag.x : blueTag.x))));

        nanoTimes.add(0, System.nanoTime());
        turretHistory.add(0, turret.getCurrentAngle());

        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel PID Power", pow * 100);
        TelemetryUtil.packet.put("Shooter : Turret Target Angle", turret.getTargetAngle());
    }

    public void indexPrepare() { indexPrepareRequest = true;}

    public void index() { indexRequest = true;}

    public void shootPrepare() { shootPrepareRequest = true;}

    public void shoot() {shootRequest = true;}

    /**
     * The input targetAngle should be in terms of global heading
     * The turret will operate in a system where facing the control hub is "0" to accommodate for (assuming intake side 0) -90 + 270 range of motion
     */
    public void setTurretAngle (double targetAngle) {
        turret.setTargetAngle(targetAngle + Math.PI / 2);

        TelemetryUtil.packet.put("Shooter : turretAngle", targetAngle + Math.PI / 2);
        LogUtil.turretAngle.set(targetAngle + Math.PI / 2);
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
        turret.setTargetAngle((thetas[tRoots.size()] - ROBOT_POSITION.heading)); // converts from global to difference with heading
        hood.setTargetAngle(phis[tRoots.size()]);
        return true;
        // currently doesn't solve the thing twice, where the v0 is set to minV0 + superThresh
        // may cause some deviations from target

    }

    /**
     * Treat turret angle as difference from heading
     * Phi is angle with respect to vertical
     * V7 uses the quartic function solver
     * @return whether or not a shot is possible as a boolean
     */
    public boolean aimLauncherV7() {
        Complex[] t = new Complex[4];
        int n = 0;
        double S = 1; // safety margin for clearing lip
        Vector3 P = new Vector3(-58.3414785, 55.6424675, 39.25 + S); // TODO: change this hardcoded target to account for team color
        P.subtract(new Vector3(ROBOT_POSITION.x, ROBOT_POSITION.y, launcherHeight));
        Vector3 V = new Vector3(-ROBOT_VELOCITY.x, -ROBOT_VELOCITY.y, 0); // TODO: need to subtract robot angular vel component thing to this
        // accel vector is irrelevant since the target is only accelerating constantly in upwards by g
        double v0 = getBallExitSpd(); // TODO: ts still needs to be fixed
        double A = g * g / 4;
        double B = 0; // B = 2 * (a dot v) = 0
        double C = V.x * V.x + V.y + V.y + g * P.z - v0 * v0; // g * P.z term is 2 * a.z * p.z, but 2 * a.z is g
        double D = 2 * Vector3.dot(P, V);
        double E = P.x * P.x + P.y * P.y + P.z * P.z;
        double a = C / A;
        double b = D / A;
        double c = E / A;
        if (Math.abs(b) < 1e-7) {
            double re1 = a * a - 4 * c;
            double im1 = re1;
            if (re1 < 0) {
                re1 = 0;
                im1 = Math.sqrt(-im1);
            } else {
                im1 = 0;
                re1 = Math.sqrt(re1);
            }
            for (int i = 0; i < t.length; i++) {
                int s1 = i % 2 == 0 ? 1 : -1;
                int s2 = i < 2 ? 1 : -1;
                t[i] = new Complex(re1, im1);
                t[i].multReal(s1);
                t[i].addReal(-a);
                t[i].multReal(0.5);
                t[i].nRoot(2);
                t[i].multReal(s2);
                if (Math.abs(t[i].imag()) > 1e-7 || t[i].real() <= 0) t[i] = null;
                else n++;
            }
        } else {
            double p = -a/12 - c;
            double q = -1 * a * a * a / 108 + a * c / 3 - b * b * 0.125;
            double re1 = q * q * 0.25 + p * p * p / 27;
            double im1 = re1;
            if (re1 < 0) {
                re1 = 0;
                im1 = Math.sqrt(-im1);
            } else {
                im1 = 0;
                re1 = Math.sqrt(re1);
            }
            Complex r = new Complex(-q * 0.5 + re1, im1);
            r.nRoot(3);
            Complex y1 = new Complex(-5 * a / 6, 0);
            if (r.mag() < 1e-7) y1.addReal(-1 * Math.pow(q, 1 / 3.0));
            else {
                y1.add(r);
                y1.add(Complex.divide(new Complex(p / -3, 0), r));
            }
            Complex w = new Complex(y1);
            w.multReal(2);
            w.addReal(a);
            w.nRoot(2);
            for (int i = 0; i < t.length; i++) {
                int s1 = i % 2 == 0 ? 1 : -1;
                int s2 = i < 2 ? 1 : -1;
                t[i] = new Complex(3 * a, 0);
                y1.multReal(2.0);
                t[i].add(y1);
                w.reciprocal();
                t[i].add(Complex.divide(new Complex(s1 * 2 * b, 0), w));
                t[i].multReal(-1.0);
                t[i].nRoot(2);
                t[i].multReal(s2);
                w.multReal(s1);
                t[i].add(w);
                t[i].multReal(0.5);
                if (Math.abs(t[i].imag()) > 1e-7 || t[i].real() <= 0) t[i] = null;
                else n++;
            }

        }

        if (n == 0) return false; // This means that a shot simply isn't possible
        double[] thetaList = new double[n + 1];
        double[] phiList = new double[n + 1];
        for (int i = 0, j = 0; i < n && j < t.length; j++) {
            if (t[j] == null) continue;
            double t0 = t[j].real();
            Vector3 pf = new Vector3(P.x + V.x * t0, P.y + V.y * t0, P.z + g * t0 * t0 / 2);
            thetaList[i] = pf.theta();
            phiList[i] = pf.phi();
            if (i == 0) {
                thetaList[n] = thetaList[0];
                phiList[n] = phiList[0];
            } else {
                if (phiList[i] < phiList[n]) {
                    phiList[n] = phiList[i];
                    thetaList[n] = thetaList[i];
                }
            }
            i++;

        }
        turret.setTargetAngle((thetaList[n] - ROBOT_POSITION.heading)); // converts from global to difference with heading
        hood.setTargetAngle(phiList[n]);
        return true;
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

    public void setShooterBlocker (boolean on) {flywheelBlocker.setTargetAngle (on ? 2.1 : -0.2);}

    public boolean atVel () {return Math.abs(error) < 1.0;}
}
