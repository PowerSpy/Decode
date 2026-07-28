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
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@Autonomous(name = "Offseason 2026 Team 2 Auto", group = "Auto")
public class OffSeasonAuto extends LinearOpMode {
    private Robot robot;
    public static Pose2D[] intakingPoses;
    public static Pose2D[] depositingPoses;
    public static Path[] intakingPath;
    public static Path[] depositingPaths;

    public boolean pointToPointMode = true;

    void initializeAuto()
    {
        Globals.RUNMODE = RunMode.AUTO;
        this.robot = new Robot(this.hardwareMap);

        this.robot.deposit.state = Deposit.State.IDLE;

        while (opModeInInit()) {
            robot.update();
            robot.sensors.light0G.set(System.currentTimeMillis() % 500 < 350);
        }
        robot.sensors.light0G.set(false);
    }

    public void pointToPointAuto()
    {
        long startTime = System.currentTimeMillis();
        this.initializeAuto();

        assert intakingPoses.length == depositingPoses.length;

        for(int i = 0;i < intakingPoses.length;i++)
        {
            robot.drivetrain.goTo(Pose2d.fromSensorsPose2D(intakingPoses[i]));
            robot.waitWhile(() -> robot.drivetrain.state != PathfollowerDrivetrain.State.IDLE);
            TelemetryUtil.packet.put("autoAchievedPose : ", Pose2d.fromSensorsPose2D(intakingPoses[i]));
            robot.intake.requestIntake(true);
            robot.waitWhile(() -> robot.intake.state != NewIntake.State.IDLE);
            robot.drivetrain.goTo(Pose2d.fromSensorsPose2D(depositingPoses[i]));
            robot.waitWhile(() -> robot.drivetrain.state != PathfollowerDrivetrain.State.IDLE);
            TelemetryUtil.packet.put("autoAchievedPose : ", Pose2d.fromSensorsPose2D(depositingPoses[i]));
            robot.deposit.requestDump = true;
            robot.waitWhile(() -> robot.deposit.state != Deposit.State.IDLE);
        }

        TelemetryUtil.packet.put("Time : ", System.currentTimeMillis()-startTime);
    }

    public void pathFollowerAuto()
    {
        long startTime = System.currentTimeMillis();
        this.initializeAuto();

        assert intakingPath.length == depositingPaths.length;

        for(int i = 0;i < depositingPaths.length;i++)
        {
            robot.drivetrain.followPath(intakingPath[i]);
            robot.waitWhile(() -> robot.drivetrain.state != PathfollowerDrivetrain.State.IDLE);
            TelemetryUtil.packet.put("autoFollowedPath : ", intakingPath[i]);
            robot.intake.requestIntake(true);
            robot.waitWhile(() -> robot.intake.state != NewIntake.State.IDLE);
            robot.drivetrain.followPath(depositingPaths[i]);
            robot.waitWhile(() -> robot.drivetrain.state != PathfollowerDrivetrain.State.IDLE);
            TelemetryUtil.packet.put("autoFollowedPath : ", depositingPaths[i]);
            robot.deposit.requestDump = true;
            robot.waitWhile(() -> robot.deposit.state != Deposit.State.IDLE);
        }

        TelemetryUtil.packet.put("Time : ", System.currentTimeMillis()-startTime);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        if(pointToPointMode) pointToPointAuto();
        else pathFollowerAuto();
    }
}
