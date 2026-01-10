package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;
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
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.MergeLocalizer;
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

import java.util.ArrayList;
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
    public MergeLocalizer mergeLocalizer;
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
        mergeLocalizer = new MergeLocalizer (hardwareMap, sensors, this, "#0000ff", "#ff00ff");
        mergeLocalizer = new MergeLocalizer (hardwareMap, sensors, this, "#0000ff", "#ffffff");
        if (vision != null) vision.start();
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
            0.2431906580541209,
            0.14544137800655768,
            0.183475227610153,
            0.18010218334461908
    };

    public void setMinPowersToOvercomeFriction(double scalar) {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0] * scalar);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1] * scalar);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2] * scalar);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3] * scalar);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.195);
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
        mergeLocalizer.setPoseEstimate(pose2d);
        LogUtil.drivePositionReset = true;
    }

    public Pose2d getPoseEstimate() {
        return ROBOT_POSITION;
    }

    private Path path = null;
    ArrayList<RepulsionPoint> repel;

    private Vector2 moveVector = new Vector2(0, 0);
    private double turn = 0;
    public double[] powers = {0, 0, 0, 0};

    private PID turnPID = new PID (0.55, 0, 0.002);

    public PathData pd;
    public static double centripetalScalar = 0.2;

    private Pose2d targetPoint = new Pose2d (0, 0, 0);
    public static PID xPID = new PID (0.1, 0.0, 0.003);
    public static PID yPID = new PID (0.1, 0.0, 0.003);
    public static PID hPID = new PID (0.15, 0.0, 0.003);

    public static double xThresh = 1.0, yThresh = 1.0, hThresh = 1.0;
    private double xError = 0.0, yError = 0.0, hError = 0.0;

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        if(path != null) {
            state = State.FOLLOW_SPLINE;
        }

        switch(state) {
            case FOLLOW_SPLINE:
                pd = path.update(ROBOT_POSITION);

                if(path != null && path.completed) {
                    targetPoint = path.lastPose.clone();
                    path = null;
                    state = State.PID_TO_POINT;
                    break;
                }

                Vector2 pathForward, pathCentripetal;
                pathForward = pd.vel;

                pathCentripetal = new Vector2(0, pathForward.mag() * pathForward.mag() / pd.r * centripetalScalar);
                pathCentripetal.rotate(Math.atan2(pathForward.y, pathForward.x));

                /*
                Log.i("Path vel", pathForward.x + " " + pathForward.y);
                Log.i("Path accel", pd.accel + "");
                Log.i("Path cent", pathCentripetal.x + " " + pathCentripetal.y);
                Log.i("Path r", pd.r + "");
                 */

                moveVector = Vector2.add(pathForward, pathCentripetal);
                double mag = moveVector.mag();
                moveVector.rotate(-ROBOT_POSITION.heading);
                moveVector.norm();

                double pathRot = 0;
                if(Math.abs(pd.r) < Spline.MAX_RADIUS) {
                    pathRot = pathForward.mag() / mag * (TRACK_WIDTH / (2.0 * pd.r)) * (pd.reversed ? -1 : 1);
                }

                double targetHeading = Math.atan2(pathForward.y, pathForward.x) + (pd.reversed ? Math.PI : 0);
                turn = pathRot + turnPID.update(AngleUtil.clipAngle(targetHeading - ROBOT_POSITION.heading), -0.6, 0.6);

                double distRemaining = ROBOT_POSITION.getDistanceFromPoint(path.lastPose);
                if (path.pathSegments.get(pd.index).decel && distRemaining <= 12) {
                    moveVector.mul(0.8 * Math.sqrt(distRemaining / 12) + 0.2);
                }

                setMoveVector(moveVector, turn);
                break;
            case PID_TO_POINT:
                calculateErrors();
                PIDF();

                if(atPoint()){
                    state = State.BRAKE;
                }
                break;
            case BRAKE:
                stopAllMotors();
                state = State.WAIT;
                break;
            case WAIT:
                if(!atPoint()){
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
    }

    public void addPoint(Pose2d point, boolean reversed) {
        path.addPoint(point);
        path.setReversed(reversed);
    }

    private void calculateErrors(){
        double deltaX = (targetPoint.x - ROBOT_POSITION.x);
        double deltaY = (targetPoint.y - ROBOT_POSITION.y);

        // convert error into direction robot is facing
        xError = Math.cos(ROBOT_POSITION.heading)*deltaX + Math.sin(ROBOT_POSITION.heading)*deltaY;
        yError = -Math.sin(ROBOT_POSITION.heading)*deltaX + Math.cos(ROBOT_POSITION.heading)*deltaY;
        hError = AngleUtil.clipAngle(targetPoint.heading - ROBOT_POSITION.heading);
    }
    
    double fwd, strafe, h;

    private void PIDF(){
        fwd = xPID.update(xError, -maxPower, maxPower);
        strafe = yPID.update(yError, -maxPower, maxPower);
        h = turnPID.update(hError, -maxPower, maxPower);

        setMinPowersToOvercomeFriction(1.0);

        Vector2 move = new Vector2(fwd, strafe);
        setMoveVector(move, h);
    }

    private boolean atPoint(){
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(hError) < Math.toRadians(hThresh);
    }

    private Pose2d lastTargetPoint = new Pose2d(0, 0, 0);
    double maxPower = 1.0;
    public void goToPoint(Pose2d targetPoint, double maxPower){
        lastTargetPoint = this.targetPoint;
        this.targetPoint = targetPoint;
        this.maxPower = maxPower;

        if(lastTargetPoint.x != targetPoint.x || lastTargetPoint.y != targetPoint.y || lastTargetPoint.heading != targetPoint.heading){
            xPID.resetIntegral();
            yPID.resetIntegral();
            hPID.resetIntegral();

            state = State.PID_TO_POINT;
        }
    }

    public void setMoveVector(Vector2 moveVector, double turn) {
        double[] powers = {
                moveVector.x - turn - moveVector.y,
                moveVector.x - turn + moveVector.y,
                moveVector.x + turn - moveVector.y,
                moveVector.x + turn + moveVector.y
        };
        normalizeArray(powers);

        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);

        TelemetryUtil.packet.put("Drivetrain : x power", moveVector.x);
        TelemetryUtil.packet.put("Drivetrain : y power", moveVector.y);
        TelemetryUtil.packet.put("Drivetrain : turn power", turn);
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setTargetPowerSmooth(lf);
        leftRear.setTargetPowerSmooth(lr);
        rightRear.setTargetPowerSmooth(rr);
        rightFront.setTargetPowerSmooth(rf);
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
//        TelemetryUtil.packet.put("Drivetrain : PID xError", xError);
//        TelemetryUtil.packet.put("Drivetrain : PID yError", yError);
//        TelemetryUtil.packet.put("Drivetrain : PID hError", hError);

        LogUtil.driveState.set(state.toString());
        LogUtil.drivePath.set(String.format(Locale.US, "%.2f %.2f %.2f", targetPoint.x, targetPoint.y, targetPoint.heading)); // TODO Log spline

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        if (path != null) {
            DashboardUtil.drawRobot(canvas, new Pose2d(ROBOT_POSITION.x + sensors.loopTime * pd.vel.x, ROBOT_POSITION.y + sensors.loopTime * pd.vel.y, Math.atan2(pd.vel.y, pd.vel.x)), "#8000ff");
            Spline s = path.pathSegments.get(pd.index).spline;

            double n = 100;
            double step = 1/n;
            for (double t = 0; t < 1; t = t + step) {
                canvas.strokeLine(s.getPos(t).x, s.getPos(t).y, s.getPos(t + step).x, s.getPos(t + step).y);
            }

            for (RepulsionPoint repel : path.repel) {
                canvas.fillCircle(repel.x, repel.y, 0.5);
            }
        }
    }
}