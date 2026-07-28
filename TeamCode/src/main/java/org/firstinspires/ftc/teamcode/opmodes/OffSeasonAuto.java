package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.Path;
import org.firstinspires.ftc.teamcode.subsystems.drive.PathfollowerDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.NewIntake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.RunMode;

@Config
@Autonomous(name = "Offseason 2026 Team 2 Auto", group = "Auto")
public class OffSeasonAuto extends LinearOpMode {
    private Robot robot;
    private Pose2D[] intakingPoses;
    private Pose2D[] depositingPoses;
    public boolean pointToPointMode = true;

    public void pointTopointAuto()
    {
        Globals.RUNMODE = RunMode.AUTO;
        this.robot = new Robot(this.hardwareMap);

        this.robot.deposit.state = Deposit.State.IDLE;

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 350);
        }
        robot.sensors.light0G.set(false);

        assert intakingPoses.length == depositingPoses.length;

        for(int i = 0;i < intakingPoses.length;i++)
        {
            robot.drivetrain.goTo(Pose2d.fromSensorsPose2D(intakingPoses[i]));
            robot.waitWhile(() -> robot.drivetrain.state != PathfollowerDrivetrain.State.IDLE);
            robot.intake.requestIntake(true);
            robot.waitWhile(() -> robot.intake.state != NewIntake.State.IDLE);
            robot.drivetrain.goTo(Pose2d.fromSensorsPose2D(depositingPoses[i]));
            robot.waitWhile(() -> robot.drivetrain.state != PathfollowerDrivetrain.State.IDLE);
            robot.deposit.requestDump = true;
            robot.waitWhile(() -> robot.deposit.state != Deposit.State.IDLE);
        }
    }

    public void pathFollowerAuto()
    {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        if(pointToPointMode) pointTopointAuto();
        else pathFollowerAuto();
    }
}
