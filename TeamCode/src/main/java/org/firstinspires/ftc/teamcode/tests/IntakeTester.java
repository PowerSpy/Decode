package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@TeleOp(group = "Test")
public class IntakeTester extends LinearOpMode {
    public static double feedPower = 0.0, intakePower = 0.0;
    public static boolean updateValues = false;
    public static double indexServoAngle = 0;

    public void runOpMode() {
        Globals.RUNMODE = RunMode.TESTER;
        Robot robot = new Robot(hardwareMap);
        Intake intake = robot.intake;
        intake.state = Intake.State.TEST;

        while (opModeInInit()) {
            robot.update();
        }

        // TODO: Add Color Sensor once wired
        while (!isStopRequested()) {
            if (gamepad1.x) intake.feed.setTargetPower(0);
            if (gamepad1.y) intake.feed.setTargetPower(0.5);
            if (gamepad1.a) intake.roller.setTargetPower(0);
            if (gamepad1.b) intake.roller.setTargetPower(0.75);

            if (updateValues) {
                intake.feed.setTargetPower(feedPower);
                intake.roller.setTargetPower(intakePower);
                intake.lindex.setTargetPos(indexServoAngle);
                updateValues = false;
            }

            robot.update();
        }
    }
}
