package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.TRACK_WIDTH;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.nMergeLocalizer;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

@Config
public class Drivetrain {
    public enum State {
        FOLLOW_SPLINE,
        PID_TO_POINT,
        BRAKE,
        WAIT,
        DRIVE,
        IDLE
    }
    public State state = State.IDLE;

    public PriorityMotor leftFront, leftRear, rightRear, rightFront;
    private final List<PriorityMotor> motors;

    public Vision vision;
    public Localizer localizer;
    public nMergeLocalizer nMergeLocalizer;
    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;

    public Drivetrain(Robot robot, Vision vision) { this(robot.hardwareMap, robot.sensors, robot.hardwareQueue, vision); }

    public Drivetrain(HardwareMap hardwareMap, Sensors sensors, HardwareQueue hardwareQueue, Vision vision) {
        this.vision = vision;
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;

        leftFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftFront"),
            "leftFront", 4, 5,
            1.0, sensors
        );
        leftRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "leftRear"),
            "leftRear", 4, 5,
            1.0, sensors
        );
        rightRear = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightRear"),
            "rightRear", 4, 5,
            1.0, sensors
        );
        rightFront = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "rightFront"),
            "rightFront", 4, 5,
            1.0, sensors
        );

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        configureMotors();
        setMinPowersToOvercomeFriction(1.0);

        localizer = new Localizer (sensors, this, "#ff0000", "#ffffff");
        nMergeLocalizer = new nMergeLocalizer (hardwareMap, sensors, this, "#0000ff", "#ff00ff");
        //if (vision != null) vision.start();
    }

    public void configureMotors() {
        for (PriorityMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.motor[0].setMotorType(motorConfigurationType);

            hardwareQueue.addDevice(motor);
        }

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
        leftRear.motor[0].setDirection(DcMotor.Direction.REVERSE);
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    // leftFront, leftRear, rightRear, rightFront
    double[] minPowersToOvercomeFriction = {
        0.3, 0.3, 0.3, 0.3
    };

    public void setMinPowersToOvercomeFriction(double scalar) {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0] * scalar);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1] * scalar);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2] * scalar);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3] * scalar);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.145);
        }
    }

    public void resetMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(0.0);
        leftRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightRear.setMinimumPowerToOvercomeStaticFriction(0.0);
        rightFront.setMinimumPowerToOvercomeStaticFriction(0.0);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0);
        }
    }

    public void setPoseEstimate(Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
        nMergeLocalizer.setPoseEstimate(pose2d);
        LogUtil.drivePositionReset = true;
    }

    public Pose2d getPoseEstimate() {
        return ROBOT_POSITION;
    }

    private Path path = null;
    long segmentStartTime;
    int lastSegmentIndex;
    public PathData data;
    private Vector2 moveVector = new Vector2(0, 0);
    private double turnPow = 0, lastGVFTime = 0.0;

    // TODO: Tune these values
    public static double correctScalar = 5.5, rotScalar = 1.15, decelThresh = 36.0;

    private Pose2d targetPoint = new Pose2d (0, 0, 0);
    public static PID xPID = new PID (0.4, 0.0, 0.007);
    public static PID yPID = new PID (0.4, 0.0, 0.007);
    public static PID turnPID = new PID (0.4, 0.0, 0.002);
    public static PID hPID = new PID (0.53, 0.0, 0.0);
    public static double turnKStatic = 0.15;
    public static double xThresh = 1.5, yThresh = 1.5, hThresh = Math.toRadians(2.5), waypointThresh = 3.0;
    public static double xError = 0.0, yError = 0.0, hError = 0.0;

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        if (path != null) {
            state = State.FOLLOW_SPLINE;
        }

        switch (state) {
            case FOLLOW_SPLINE:
                data = path.update(ROBOT_POSITION);

                // Null data indicates the end of path has been reached
                if (data == null) {
                    targetPoint = path.getLastPose();
                    maxPower = 0.8;
                    lastGVFTime = System.currentTimeMillis();
                    path = null;
                    state = State.PID_TO_POINT;
                    break;
                }

                // Timeout / stuck protection (looks good? will check more in depth. consider switching to PID to point rather than moving the path ahead since the splines will be starting at weird points now
                if (data.index != lastSegmentIndex) {
                    lastSegmentIndex = data.index;
                    segmentStartTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - segmentStartTime > 5000) {
                    Log.i("Drivetrain", "Segment " + data.index + " timed out. Skipping to next index");
                    path.setIndex(data.index + 1);
                    segmentStartTime = System.currentTimeMillis();
                    lastSegmentIndex = data.index + 1;
                }

                Vector2 traverse = new Vector2(data.velocity.x, data.velocity.y);
                Vector2 correct = new Vector2(0, traverse.mag() * traverse.mag() / data.radius * correctScalar);
                correct.rotate(Math.atan2(traverse.y, traverse.x));

                // Only scale the traverse part of moveVector bc correction should remain true to curr velocity
                if (data.decel && ROBOT_POSITION.getDistanceFromPoint(path.getSegLast(data.index)) < decelThresh) {
                    traverse.mul(Math.pow(Math.E, (ROBOT_POSITION.getDistanceFromPoint(path.getSegLast(data.index)) * 0.25)));
                }

                moveVector = Vector2.add(traverse, correct);
                moveVector.rotate(-ROBOT_POSITION.heading);
                double mag = moveVector.mag();
                moveVector.norm();

                double pathRot = 0;
                if (Math.abs(data.radius) < Spline.MAX_RADIUS) {
                    pathRot = traverse.mag() / mag * (TRACK_WIDTH) / (2.0 * data.radius) * (data.reversed ? -1 : 1);
                }

                double targetHeading = Math.atan2(traverse.y, traverse.x) + (data.reversed ? Math.PI : 0);
                turnPow = pathRot * rotScalar + hPID.update(targetHeading - ROBOT_POSITION.heading, -1.0, 1.0);

                moveVector.mul(data.power);
                setMoveVector(moveVector, turnPow);
                break;
            case PID_TO_POINT:
                calculateErrors();
                PIDF();
                if (atPoint()) {
                    state = isWaypoint ? State.WAIT : State.BRAKE;
                    xPID.resetIntegral();
                    yPID.resetIntegral();
                    turnPID.resetIntegral();
                }
                break;
            case BRAKE:
                stopAllMotors();
                state = State.WAIT;
                break;
            case WAIT:
                calculateErrors();
                if (!atPoint()) {
                    state = State.PID_TO_POINT;
                }
                break;
            case DRIVE:
                break;
            case IDLE:
                break;
        }

        updateTelemetry();
    }

    public void setPath (Path p) {
        this.path = p;
        segmentStartTime = System.currentTimeMillis();
        lastSegmentIndex = 0;
    }

    public Pose2d getCurrentPathTarget() { return path.getSegLast(lastSegmentIndex); }

    private void calculateErrors() {
        double deltaX = (targetPoint.x - ROBOT_POSITION.x);
        double deltaY = (targetPoint.y - ROBOT_POSITION.y);

        // convert error into direction robot is facing
        xError = Math.cos(ROBOT_POSITION.heading)*deltaX + Math.sin(ROBOT_POSITION.heading)*deltaY;
        yError = -Math.sin(ROBOT_POSITION.heading)*deltaX + Math.cos(ROBOT_POSITION.heading)*deltaY;
        hError = AngleUtil.clipAngle(targetPoint.heading - ROBOT_POSITION.heading);
    }
    
    double fwd, strafe, h;

    private void PIDF() {
        fwd = xPID.update(xError, -maxPower, maxPower);
        strafe = yPID.update(yError, -maxPower, maxPower);
        h = turnPID.update(hError, -maxPower, maxPower);
        if (hError > hThresh) h += turnKStatic;
        if (hError < -hThresh) h -= turnKStatic;

        setMinPowersToOvercomeFriction(1.0);

        double k = Math.exp(-(System.currentTimeMillis() - lastGVFTime));
        Vector2 move = new Vector2(fwd + moveVector.x * k, strafe + moveVector.y * k);
        setMoveVector(move, h + turnPow * k);
    }

    private boolean atPoint() {
        if (isWaypoint) return Math.abs(xError) < waypointThresh && Math.abs(yError) < waypointThresh;
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(hError) < hThresh;
    }

    private double maxPower = 1.0;
    private boolean isWaypoint = false;
    public void goToPoint(Pose2d targetPoint, double maxPower) { goToPoint(targetPoint, maxPower, false); };
    public void goToPoint(Pose2d targetPoint, double maxPower, boolean isWaypoint) {
        Pose2d lastTargetPoint = this.targetPoint;
        this.targetPoint = targetPoint;
        this.maxPower = maxPower;
        this.isWaypoint = isWaypoint;

        if (lastTargetPoint.x != targetPoint.x || lastTargetPoint.y != targetPoint.y || lastTargetPoint.heading != targetPoint.heading || state == State.DRIVE) {
            xPID.resetIntegral();
            yPID.resetIntegral();
            turnPID.resetIntegral();
            state = State.PID_TO_POINT;
        }
    }

    //private double lastMoveVectorX = 0;
    //public static double noWheelieAccelForward = 5, noWheelieDecelForward = 3, noWheelieAccelReverse = 4, noWheelieDecelReverse = 4, wheelieThresh = 1;
    public void setMoveVector(Vector2 moveVector, double turn) {
        /*
        double moveVectorXLimited = moveVector.x;
        if (Math.abs(moveVector.x) > wheelieThresh || Math.abs(lastMoveVectorX) > wheelieThresh) {
            if ((moveVector.x - lastMoveVectorX) * Math.signum(lastMoveVectorX) > 0)
                moveVectorXLimited = Utils.minMaxClip(moveVector.x, lastMoveVectorX - noWheelieAccelReverse * sensors.loopTime, lastMoveVectorX + noWheelieAccelForward * sensors.loopTime);
            else
                moveVectorXLimited = Utils.minMaxClip(moveVector.x, lastMoveVectorX - noWheelieDecelForward * sensors.loopTime, lastMoveVectorX + noWheelieDecelReverse * sensors.loopTime);
        }
        lastMoveVectorX = moveVectorXLimited;
        */
        double[] powers = {
                moveVector.x - turn - moveVector.y,
                moveVector.x - turn + moveVector.y,
                moveVector.x + turn - moveVector.y,
                moveVector.x + turn + moveVector.y
        };
        normalizeArray(powers);

        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

        TelemetryUtil.packet.put("Drivetrain : moveVector x", moveVector.x);
        //TelemetryUtil.packet.put("Drivetrain : moveVector x limited", moveVectorXLimited);
        TelemetryUtil.packet.put("Drivetrain : moveVector y", moveVector.y);
        TelemetryUtil.packet.put("Drivetrain : moveVector turn", turn);
    }

    public static double smoothPowerK = 0.5;
    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        Log.i("Drivetrain powers : ", lf + " " + lr + " " + rr + " " + rf);
        leftFront.setTargetPowerSmooth(lf, smoothPowerK);
        leftRear.setTargetPowerSmooth(lr, smoothPowerK);
        rightRear.setTargetPowerSmooth(rr, smoothPowerK);
        rightFront.setTargetPowerSmooth(rf, smoothPowerK);
    }

    public void stopAllMotors() {
        for (PriorityMotor motor : motors) {
            motor.setTargetPower(0);
        }
    }

    public void normalizeArray(double[] arr) {
        double largest = 1;
        for (int i = 0; i < arr.length; i++) {
            largest = Math.max(largest, Math.abs(arr[i]));
        }
        for (int i = 0; i < arr.length; i++) {
            arr[i] /= largest;
        }
    }

    public void drive(Gamepad gamepad) {
        resetMinPowersToOvercomeFriction();
        state = State.DRIVE;

        double forward = smoothControls(-1 * gamepad.left_stick_y);
        double strafe = smoothControls(-1 * gamepad.left_stick_x);
        double turn = smoothControls(-gamepad.right_stick_x);

        Vector2 drive = new Vector2(forward,strafe);
        if (drive.mag() <= 0.05) {
            drive.mul(0);
        }
        setMoveVector(drive,turn);
    }

    public double smoothControls(double value) {
        return 0.7 * Math.tan(0.96 * value);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("Drivetrain : state", state);



//        TelemetryUtil.packet.put("Drivetrain : TargetPoint", "(" + targetPoint.x + ", " + targetPoint.y + ", " + targetPoint.heading + ")");
        TelemetryUtil.packet.put("Drivetrain : PID xError", xError);
        TelemetryUtil.packet.put("Drivetrain : PID yError", yError);
        TelemetryUtil.packet.put("Drivetrain : PID hError", hError);

        TelemetryUtil.packet.put("Drivetrain : PID xPow", fwd);
        TelemetryUtil.packet.put("Drivetrain : PID yPow", strafe);
        TelemetryUtil.packet.put("Drivetrain : PID hPow", h);

        LogUtil.driveState.set(state.toString());

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        if (path != null) {
            DashboardUtil.drawRobot(canvas, new Pose2d(ROBOT_POSITION.x + sensors.loopTime * data.velocity.x, ROBOT_POSITION.y + sensors.loopTime * data.velocity.y, Math.atan2(data.velocity.y, data.velocity.x)), "#8000ff"); // purple
            Spline s = path.segments.get(data.index).spline;
            LogUtil.drivePath.set(s.toString());

            double n = 100;
            double step = 1/n;
            for (double t = 0; t < 1; t = t + step) {
                canvas.strokeLine(s.getPos(t).x, s.getPos(t).y, s.getPos(t + step).x, s.getPos(t + step).y);
            }

            for (RepulsionPoint repel : path.repulsion) {
                canvas.fillCircle(repel.x, repel.y, 0.5);
            }
        } else {
            DashboardUtil.drawRobot(canvas, targetPoint, isWaypoint ? "#c040ff" : "#8000ff"); // purple and bright violet
            LogUtil.drivePath.set(String.format(Locale.US, "%.3f %.3f %.3f", targetPoint.x, targetPoint.y, targetPoint.heading));
        }
    }
}