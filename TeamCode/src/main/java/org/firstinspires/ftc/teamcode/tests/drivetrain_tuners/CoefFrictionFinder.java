package org.firstinspires.ftc.teamcode.tests.drivetrain_tuners;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;

@Disabled
@TeleOp(group = "tests")
public class CoefFrictionFinder extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        boolean recording = false;
        double accelerationAvgX = 0;
        double accelerationAvgY = 0;
        ArrayList<Vector2> velocities = new ArrayList<>();
        int count = 0;

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1);

            // We don't be driving no more
            if (gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_y == 0) {
                recording = true;
                // accelerationAvg = 0;
                count = 0;
            } else {
                recording = false;
            }

            Vector2 velocityVec = new Vector2(ROBOT_VELOCITY.x, ROBOT_VELOCITY.y);
            if (velocityVec.mag() == 0) {
                // We got it bois
                recording = false;
            }

            /*Pose2d acceleration =
            if (recording) {
                accelerationAvgX += count++;
            } todo */
        }
    }
}
