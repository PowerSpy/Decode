package org.firstinspires.ftc.teamcode.tests.utils_testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class RightFrontMotorTester extends LinearOpMode {
    public static double power = 0.5;

    // ticksToInches = 0.3860 * 2pi inch / 384 ticks
    double ticksToInches = 0.006316;
    public void runOpMode(){
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, "testMotor");

        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            m.setPower(power);

            telemetry.addData("Motor Power", m.getPower());
            telemetry.addData("Motor Pos (ticks)", m.getCurrentPosition());
            telemetry.addData("Motor Pos (inch)", m.getCurrentPosition() * ticksToInches);
            telemetry.addData("Motor Vel (ticks/sec)", m.getVelocity());
            telemetry.addData("Motor Vel (rads/sec)", m.getVelocity(AngleUnit.RADIANS));

            telemetry.update();
        }
    }
}
