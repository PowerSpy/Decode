package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.drive.RepulsionPoint;

import java.util.ArrayList;

@Config
public class Globals {
    // general
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static RunMode RUNMODE = RunMode.TESTER;
    public static boolean TESTING_DISABLE_CONTROL = true;
    public static boolean isRed = true;
    public static long autoStartTime = -1;
    public static boolean autoHang = true;
    public static boolean gotBloodyAnnihilated = false; // STOP DELETEING THIS FOR GODS SAKE


    //vision
    //true if full field
    public static boolean fullField = false;
    public static double LLHeight = 10; //Todo - Change
    public static double LlAngle = 10; //Todo - Change - Angle the limelight is looking below from the horizontal - Degrees
    public static double ballRadius = 2.5;



    // drivetrain
    public static boolean DRIVETRAIN_ENABLED = true;
    public static double TRACK_WIDTH = 11.27;
    //robot width includes bumpers
    public static double ROBOT_WIDTH = 16.04;
    public static double ROBOT_LENGTH = 18.0;
    public static double ROBOT_BACK_LENGTH = 6.2;
    public static double ROBOT_FORWARD_LENGTH = 7.4;
    public static double BALL_PATTERN;
    public static Pose2d ROBOT_POSITION = new Pose2d(0,0,0);
    public static Pose2d ROBOT_VELOCITY = new Pose2d(0,0,0);
    public static Pose2d ROBOT_GLOBAL_VELOCITY = new Pose2d(0,0,0);
    public static Pose2d AUTO_ENDING_POSE = new Pose2d(0,0,0);


    public static ArrayList<RepulsionPoint> getMidline() {
        ArrayList<RepulsionPoint> rp = new ArrayList<>();

        for (int i = 72; i >= -72; i -= 4) {
            rp.add(new RepulsionPoint(i, 0, 6));
        }

        // Prevent slamming into the gate too hard (58 on y axis)
        rp.add(new RepulsionPoint(0, (isRed ? 1 : -1) * 60, 2.5));

        return rp;
    }

    // loop time methods
    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    // DECODE
    public static Pose2d redTag = new Pose2d(-58.3414795, 55.6424675);
    public static Pose2d blueTag = new Pose2d(-58.3414795, -55.6424675);
    public static double tagHeight = 29.5;

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }
}
