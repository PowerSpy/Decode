package org.firstinspires.ftc.teamcode.tests.utils_testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class RightFrontMotorTester extends LinearOpMode {

    public void runOpMode(){
        Robot robot = new Robot(hardwareMap);

        DcMotorEx m1 = robot.hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx m2 = robot.hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx m3 = robot.hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx m4 = robot.hardwareMap.get(DcMotorEx.class, "rightRear");

        DcMotorEx m5 = robot.hardwareMap.get(DcMotorEx.class, "roller");

        while(!isStopRequested()){
            m1.setPower(0.5);
            m2.setPower(0.5);
            m3.setPower(0.5);
            m4.setPower(0.5);
            m5.setPower(0.5);
        }

    }
}
