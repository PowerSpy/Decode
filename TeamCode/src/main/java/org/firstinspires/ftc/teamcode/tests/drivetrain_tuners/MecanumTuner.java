package org.firstinspires.ftc.teamcode.tests.drivetrain_tuners;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Disabled
@Config
@TeleOp
public class MecanumTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        waitForStart();
        drivetrain.setMotorPowers(1,1,1,1);
        double start = System.currentTimeMillis();
        if (System.currentTimeMillis()  >= start + 1500) {
            drivetrain.setMotorPowers(0,0,0,0);

            Pose2d curr = ROBOT_POSITION;
            TelemetryUtil.packet.put("heading", curr.heading);
            TelemetryUtil.packet.put("x", curr.x);
            TelemetryUtil.packet.put("y",curr.y);
            drivetrain.update();
        }
    }
}
