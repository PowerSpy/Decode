package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

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
    public final nPriorityServo flywheelBlocker, hood, net;
    public final PriorityCRServo turret;

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

    public PID turretPID = new PID (0.5, 0.0, 0.1);


    /*
    Hood / Velo
    Far: 1.34 / 100
    Middle: 1.0 / 70
    Close: 0.7 / 60
     */


    public Shooter(Robot robot) {
        this.robot = robot;

        this.sensors = robot.sensors;

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

        turret = new PriorityCRServo(
            new CRServo[]{robot.hardwareMap.get(CRServo.class, "turret1"), robot.hardwareMap.get(CRServo.class,"turret2")},
            "turret", PriorityCRServo.ServoType.AXON_MINI,
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
                    aimLauncherV4(); // starts autoaim going
                }

                if(indexPrepareRequest){
                    // Calculation for targetVelocity goes here
                    state = State.ACCEL;
                }
                break;
            case ACCEL:
                if(atVel() && shootRequest){
                    state = State.SHOOT;
                }

                if(atVel() && indexRequest){
                    state = State.INDEX;
                }
                break;
            case SHOOT:
                aimLauncherV4(); // right up until the shot happens, the hood and turret should be recalculated as often as possible
                break;
            case INDEX:
                setShooterBlocker(false);

                // TODO: Probably need to reorganize this to better differentiate the two actions, especially because indexing and shooting will lauch diff number of balls
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

        // turret PID
        error = turret.getAngle() - turret.getTargetAngle();
        turret.setTargetPower(turretPID.update(error, -0.5, 0.5)); // starting with low maximums because unless the turret starts lagging behind turns, we don't need it to move crazy fast



        TelemetryUtil.packet.put("Shooter : Flywheel Filtered Velocity", filteredVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel Target Velocity", targetVelocity);
        TelemetryUtil.packet.put("Shooter : Flywheel PID Power", pow * 100);
        TelemetryUtil.packet.put("Shooter : Turret Target Angle", turret.getTargetAngle());
    }

    public void indexPrepare() { indexPrepareRequest = true;}

    public void index() { indexRequest = true;}

    public void shootPrepare() { shootPrepareRequest = true;}

    public void shoot() {shootRequest = true;}

    public void setTurretAngle(double target_angle) {
        turret.setTargetAngle(target_angle);

        TelemetryUtil.packet.put("Shooter : turretAngle", target_angle);
        LogUtil.turretAngle.set(target_angle);
    }

    /**
     * treat turret angle as the offset from heading
     * phi is angle with respect to vertical
     * this is the v4 algorithm, v7 is in the works
     */
    public void aimLauncherV4() {
        distance = new Vector3(ballTarget.getX() - ROBOT_POSITION.x, ballTarget.getY() -  ROBOT_POSITION.y, 0);
        ballExit2DSpd = new Vector3(getBallExitSpd() * distance.x/distance.getMag(), getBallExitSpd() * distance.y/distance.getMag(), 0);
        tVel  = new Vector3(ROBOT_VELOCITY.x, ROBOT_VELOCITY.y, 0);
        // find a way to get robot angular velocity from drivetrain "turn"
        // rVel = Vector3.cross(new Vector3(0, 0, ___), new Vector3(dLauncher * Math.cos(heading), dLauncher * Math.sin(heading), 0));
        vel = Vector3.add(ballExit2DSpd, tVel); // .add(rVel);

        double phiLimit = Math.acos(Math.sqrt(2 * g * (ballTarget.z - launcherHeight))) - 1e-5;
        double theta = Math.PI - 1e-4;
        double phi = Math.PI / 2 - 1e-4;
        double[] r0 = new double[4];
        double F0 = 0.0;
        double L = 4 * Math.PI;
        int n = 0;
        double[] J = {0};
        Vector2 E = new Vector2(0, 0);
        double[] J_inv = {0};
        double detJReciprocal = 1;
        Vector2 s = new Vector2(0, 0);
        double thetaNext = theta;
        double phiNext = phi;
        Vector3 TE = new Vector3(100, 0, 0);

        while ((F0 < L || n < 2) && ((theta - thetaNext) * (theta - thetaNext) + (phi - phiNext) * (phi - phiNext) > 1e-14)) {
            J = errorFunctionYields(theta, phi);
            E = new Vector2(J[4], J[5]);
            J_inv = new double[]{J[3], -J[1], -J[2], J[0]};
            detJReciprocal = 1 / (J[0] * J[3] - J[1] * J[2]); // perhaps a bit of cause for fear
            for (int i = 0; i < 4; i++) {
                J_inv[i] = J_inv[i] * detJReciprocal;
            }
            Vector2 J1 = new Vector2(J_inv[0], J_inv[1]);
            Vector2 J2 = new Vector2(J_inv[2], J_inv[3]);
            s = new Vector2(Vector2.dot(J1, E), Vector2.dot(J2, E));
            thetaNext = clamp(theta - s.x, 0, 2 * Math.PI);
            phiNext = clamp(phi - s.y, 0.01, phiLimit);
            TE = new Vector3(J[6], J[7], J[8]);
            if (TE.getMag() < 0.2) {
                boolean diff = true;
                for (int i = 0; i < r0.length; i += 2) {
                    if (Math.abs(r0[i] - theta) <= 2e-5 && Math.abs(r0[i + 1] - phi) <= 2e-5) diff = false;
                }
                if (diff) {
                    r0[2 * n] = theta;
                    r0[2 * n + 1] = phi;
                    n++;
                    F0 += Math.abs(phi - 0.1);
                    phi = 0.1;
                } else {
                    F0 = L + 1;
                }
            } else {
                F0 += Vector2.subtract(new Vector2(theta, phi), new Vector2(thetaNext, phiNext)).mag();
                theta = thetaNext;
                phi = phiNext;
            }
        }

        if (r0[1] > r0[3]) {
            theta = r0[0];
            phi = r0[1];
        } else {
            theta = r0[2];
            phi = r0[3];
        }

        setTurretAngle(theta);
        setHoodAngle(phi);

    }

    private double[] errorFunctionYields(double theta, double phi) {

        double v0 = getBallExitSpd();
        double d = 1e-9;
        Vector2 e = errorFunction(theta, phi, v0);
        Vector2 s1 = Vector2.subtract(errorFunction(theta + d, phi, v0), e);
        s1.mul(1/d);
        Vector2 s2 = Vector2.subtract(e, (errorFunction(theta + d, phi, v0)));
        s2.mul(1/d);
        Vector3 TE = simulateShot(theta, phi, new Vector2(v0 * Math.sin(phi) * Math.cos(theta) + tVel.x + rVel.x, v0 * Math.sin(phi) * Math.sin(theta) + tVel.y + rVel.y), v0);
        TE.subtract(ballTarget);
        TE.subtract(new Vector3(0, 0, ballTarget.z));
        return new double[]{s1.x, s2.x, s1.y, s2.y, e.x, e.y, TE.x, TE.y, TE.z};
    }

    private Vector2 errorFunction(double theta, double phi, double v0) {

        double velX = v0 * Math.sin(phi) * Math.cos(theta) + tVel.x + rVel.x;
        double velY = v0 * Math.sin(phi) * Math.sin(theta) + tVel.y + rVel.y;
        double vel2D = Math.sqrt(velX * velX + velY * velY);
        double e1 = (velX * distance.x + velY * distance.y) / (distance.getMag() * vel2D) - 1;
        double e2 = (v0 * Math.cos(phi) + Math.sqrt( v0 * v0 * Math.cos(phi) * Math.cos(phi) - 2 * g * (ballTarget.z - launcherHeight))) / g - distance.getMag() / vel2D;
        return new Vector2(e1, e2);
    }

    private double clamp(double x, double l1, double l2) { return (l1 <= x && x <= l2 ? x : (x < l1 ? l1 : l2)); }

    public Vector3 simulateShot(double theta, double phi, Vector2 vel, double v0) {
        double t = distance.getMag() / vel.mag();
        return Vector3.add(new Vector3(vel.x * t, vel.y * t, launcherHeight + v0 * Math.cos(phi) * t - g * t * t / 2), new Vector3(-ROBOT_POSITION.x, -ROBOT_POSITION.y, 0));
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
