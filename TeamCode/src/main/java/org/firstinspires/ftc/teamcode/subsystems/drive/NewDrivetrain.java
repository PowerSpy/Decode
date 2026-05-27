package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Arrays;
import java.util.List;

public class NewDrivetrain {

    private final List<PriorityMotor> motors;

    private final PriorityMotor leftRear, rightRear, leftFront, rightFront;

    private final Sensors sensors;
    private final Vision vision;
    private final HardwareQueue hardwareQueue;
    public NewDrivetrain(HardwareMap hardwareMap, Sensors sensors, HardwareQueue hardwareQueue, Vision vision) {
        this.vision = vision;
        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;

        leftRear = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftRear"), "leftRear", 4, 5, 1, sensors);
        rightRear = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightRear"), "rightRear", 4, 5, 1, sensors);
        leftFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "leftFront"), "leftFront", 4, 5, 1, sensors);
        rightFront = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "rightFront"), "rightFront", 4, 5, 1, sensors);

        motors = Arrays.asList(leftRear, rightRear, leftFront, rightFront);

        configureMotors(motors);
    }

    public void configureMotors(List<PriorityMotor> motors) {
        leftRear.motor[0].setDirection(DcMotor.Direction.REVERSE);
        leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);

        for (PriorityMotor motor: motors) {
            motor.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardwareQueue.addDevice(motor);
        }
    }

    private void normalizeArray(double[] array) {
        double max = 1.0;
        for (Double val: array) {
            max = Math.max(Math.abs(val), max);
        }

        for (int i = 0; i < array.length; i++) {
            array[i] /= max;
        }
    }

    public void setMotorPowers(double[] powers) {
        leftFront.setTargetPower(powers[0]);
        leftRear.setTargetPower(powers[1]);
        rightRear.setTargetPower(powers[2]);
        rightFront.setTargetPower(powers[3]);
    }

    public void stopAllMotors() {
        for (PriorityMotor motor: motors) {
            motor.setTargetPower(0);
        }
    }

    public void drive(Gamepad gamePad) {
        double forward = gamePad.left_stick_y * -1;
        double strafe = gamePad.left_stick_x;
        double rotation = gamePad.right_stick_x;

        // leftFront, leftRear, rightRear, rightFront
        double[] powers = {forward - strafe - rotation, forward + strafe - rotation, forward - strafe + rotation, forward + strafe + rotation};
        normalizeArray(powers);

        setMotorPowers(powers);

    }
}
