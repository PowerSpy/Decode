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
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Drivetrain {
    public enum State {
        FOLLOW_SPLINE,
        DRIVE,
        IDLE
    }
    public State state = State.IDLE;

    public PriorityMotor leftFront, leftRear, rightRear, rightFront;
    private final List<PriorityMotor> motors;

    public Robot robot;
    public Localizer[] localizers;
    public Vision vision;
    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;

    public Drivetrain(Robot robot) {
        this.robot = robot;
        this.hardwareQueue = robot.hardwareQueue;
        this.sensors = robot.sensors;

        leftFront = new PriorityMotor(
            robot.hardwareMap.get(DcMotorEx.class, "leftFront"),
            "leftFront", 4, 5,
            -1.0, sensors
        );
        leftRear = new PriorityMotor(
            robot.hardwareMap.get(DcMotorEx.class, "leftRear"),
            "leftRear", 4, 5,
            -1.0, sensors
        );
        rightRear = new PriorityMotor(
            robot. hardwareMap.get(DcMotorEx.class, "rightRear"),
            "rightRear", 4, 5,
            -1.0, sensors
        );
        rightFront = new PriorityMotor(
            robot.hardwareMap.get(DcMotorEx.class, "rightFront"),
            "rightFront", 4, 5,
            -1.0, sensors
        );

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        configureMotors();

        localizers = new Localizer[]{
            new Localizer(robot.hardwareMap, sensors, this, "#00c000", "#00c00060"),
        };

        setMinPowersToOvercomeFriction(1.0);
    }

    public void configureMotors() {
        for (PriorityMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.motor[0].setMotorType(motorConfigurationType);

            hardwareQueue.addDevice(motor);
        }

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            0.2413194940290,
            0.2337791473,
            0.246122952040818,
            0.2737912666227906
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


    public void updateLocalizers() {
        for (Localizer l : localizers) {
            l.updateEncoders(sensors.odoWheelPositions);
            l.update();
        }
    }

    public void setPoseEstimate(Pose2d pose2d) {
        for (Localizer l : localizers) {
            l.setPoseEstimate(pose2d);
        }
    }

    public Pose2d getPoseEstimate() {
        return ROBOT_POSITION;
    }

    private Path path = null;
    ArrayList<RepulsionPoint> repel;
    private Pose2d pos;

    private Vector2 moveVector = new Vector2(0, 0);
    private double turn = 0;
    public double[] powers = {0, 0, 0, 0};

    private PID turnPID = new PID (1, 0, 0);

    public PathData pd;

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        ROBOT_POSITION = sensors.getOdometryPosition();
        Vector2 vel = sensors.getVelocity();
        ROBOT_VELOCITY = new Pose2d(vel.x, vel.y, Math.atan2(vel.x, vel.y));

        if(path != null) {
            state = State.FOLLOW_SPLINE;
        }

        switch(state) {
            case FOLLOW_SPLINE:
                pd = path.update(pos);
                Vector2 pathForward, pathCentripetal;
                pathForward = pd.vel;
                pathCentripetal = new Vector2(0, pathForward.mag() * pathForward.mag() / pd.r);
                pathCentripetal.rotate(Math.atan2(pathForward.y, pathForward.x));

                moveVector = Vector2.add(pathForward, pathCentripetal);
                double mag = moveVector.mag();
                moveVector.rotate(-pos.heading);
                moveVector.norm();

                double pathRot = 0;
                if(Math.abs(pd.r) < Spline.MAX_RADIUS) {
                    pathRot = pathForward.mag() / mag * TRACK_WIDTH / (2.0 * pd.r) * (pd.reversed ? -1 : 1);
                }

                double targetHeading = Math.atan2(pathForward.y, pathForward.x) + (pd.reversed ? Math.PI : 0);
                turn = pathRot + turnPID.update(AngleUtil.clipAngle(targetHeading - pos.heading), -0.6, 0.6);

                setMoveVector(moveVector, turn);

                if(path.completed) {
                    path = null;
                    state = State.IDLE;
                }
                break;
            case DRIVE:
                break;
            case IDLE:
                break;
        }
    }

    public void setPath (Path p) {
        this.path = p;
    }

    public void addPoint(Pose2d point, boolean reversed) {
        path.addPoint(point);
        path.setReversed(reversed);
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
        /*
        TelemetryUtil.packet.put("Drivetrain : state", state);
        LogUtil.driveState.set(state.toString());

        TelemetryUtil.packet.put("Drivetrain : atPoint", atPoint());
        TelemetryUtil.packet.put("Drivetrain : xError", xError);
        TelemetryUtil.packet.put("Drivetrain : yError", yError);
        TelemetryUtil.packet.put("Drivetrain : turnError (deg)", Math.toDegrees(turnError));
        TelemetryUtil.packet.put("Drivetrain : xTarget", targetPoint.x);
        TelemetryUtil.packet.put("Drivetrain : yTarget", targetPoint.y);
        TelemetryUtil.packet.put("Drivetrain : turnTarget (deg)", Math.toDegrees(targetPoint.heading));
        LogUtil.driveTargetX.set(targetPoint.x);
        LogUtil.driveTargetY.set(targetPoint.y);
        LogUtil.driveTargetAngle.set(targetPoint.heading);

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        DashboardUtil.drawRobot(canvas, targetPoint, "#8000ff");

        if (path != null) {
            Pose2d last = path.poses.get(0);
            for (int i = 1; i < path.poses.size(); i++) {
                Pose2d next = path.poses.get(i);
                canvas.strokeLine(last.x, last.y, next.x, next.y);
                last = next;
            }
        }
        */
    }
}