package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Collections;

@Config
public class PathfollowerDrivetrain
{
    public enum State
    {
        IDLE,
        TO_TARGET,
        TEST
    }
    public static class MotorIndices
    {
        public static int LEFT_FRONT = 0;
        public static int RIGHT_FRONT = 1;
        public static int LEFT_REAR = 2;
        public static int RIGHT_REAR = 3;
    }
    public static PID rotationPID = new PID(0.1, 0.1, 0.1); // Placeholder
    public static PID strafePID = new PID(0.1, 0.1, 0.1); // Placeholder
    public static PID forwardPID = new PID(0.1, 0.1, 0.1); // Placeholder
    public static double kSF = 0.1; // Placeholder
    public static double posErrorThreshold = 0.1; //Placeholder
    public static double rotErrorThreshold = 0.1; //Placeholder

    public boolean requestToTarget = false;

    private Robot robot;
    private PriorityMotor[] motors;
    private State state = State.IDLE;

    private double targetHeading;
    private double targetPosX, targetPosY;
    private Pose2D currentPos;
    private double currentHeading;
    private double minPower = -1;
    private double maxPower = 1;

    public PathfollowerDrivetrain(Robot robot, double minPower, double maxPower)
    {
        this.robot = robot;
        this.motors = new PriorityMotor[4];
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.motors[MotorIndices.LEFT_FRONT] = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "leftFront"),
                "leftFront", 4, 5,
                1.0, this.robot.sensors
        );
        this.motors[MotorIndices.RIGHT_FRONT] = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "rightFront"),
                "rightFront", 4, 5,
                1.0, this.robot.sensors
        );
        this.motors[MotorIndices.LEFT_REAR] = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "leftRear"),
                "leftRear", 4, 5,
                1.0, this.robot.sensors
        );
        this.motors[MotorIndices.RIGHT_REAR] = new PriorityMotor(
                this.robot.hardwareMap.get(DcMotorEx.class, "rightRear"),
                "rightRear", 4, 5,
                1.0, this.robot.sensors
        );

        this.motors[MotorIndices.LEFT_FRONT].motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        this.motors[MotorIndices.LEFT_REAR].motor[0].setDirection(DcMotorSimple.Direction.REVERSE);

        for(PriorityMotor m : this.motors)
        {
            m.motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            this.robot.hardwareQueue.addDevice(m);
        }
    }

    private void setNormalizedMotorPowers(double frontLeftPow, double frontRightPow, double rearLeftPow, double rearRightPow)
    {
        double q = Collections.max(Arrays.asList(Math.abs(frontLeftPow), Math.abs(frontRightPow), Math.abs(rearLeftPow), Math.abs(rearRightPow), 1.0));
        this.motors[MotorIndices.LEFT_FRONT].setTargetPower(frontLeftPow/q);
        this.motors[MotorIndices.RIGHT_FRONT].setTargetPower(frontRightPow/q);
        this.motors[MotorIndices.LEFT_REAR].setTargetPower(rearLeftPow/q);
        this.motors[MotorIndices.RIGHT_REAR].setTargetPower(rearRightPow/q);
    }

    public void update()
    {
        this.currentPos = this.robot.sensors.getEstPosition();
        this.currentHeading = this.currentPos.getHeading(Sensors.odoAngleUnit);

        switch(this.state)
        {
            case IDLE:
                if(this.requestToTarget)
                {
                    this.state = State.TO_TARGET;
                }
                break;
            case TO_TARGET:
                double xError = targetPosX-this.currentPos.getX(Sensors.odoDistanceUnit);
                double yError = targetPosY-this.currentPos.getY(Sensors.odoDistanceUnit);
                double headingError = targetHeading-this.currentHeading;

                if(Math.abs(xError) <= PathfollowerDrivetrain.posErrorThreshold && Math.abs(yError) <= PathfollowerDrivetrain.posErrorThreshold
                    && Math.abs(headingError) <= PathfollowerDrivetrain.rotErrorThreshold)
                {
                    this.requestToTarget = false;
                    this.state = State.IDLE;
                }

                double rotatePow = rotationPID.update(headingError, this.minPower, this.maxPower);
                double forwardPow = forwardPID.update(yError, this.minPower, this.maxPower);
                double strafePow = strafePID.update(xError, this.minPower, this.maxPower);

                double motorPowX = strafePow*Math.cos(-this.currentHeading)-forwardPow*Math.sin(-this.currentHeading);
                double motorPowY = strafePow*Math.sin(-this.currentHeading)+forwardPow*Math.cos(-this.currentHeading);

                double frontLeftPow = motorPowY + motorPowX + rotatePow;
                double frontRightPow = motorPowY - motorPowX + rotatePow;
                double rearLeftPow = motorPowY - motorPowX - rotatePow;
                double rearRightPow = motorPowY + motorPowX - rotatePow;

                setNormalizedMotorPowers(frontLeftPow, frontRightPow, rearLeftPow, rearRightPow);
                break;
        }
    }
}
