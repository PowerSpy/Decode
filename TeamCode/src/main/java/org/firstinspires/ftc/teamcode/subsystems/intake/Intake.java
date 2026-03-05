package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Intake {
    private final Robot robot;
    public final PriorityMotor roller, feed;
    //public final nPriorityServo index1, index2;

    private boolean requestIntake = false, requestShoot = false, requestOff = false, reversed = false;

    public static double intakeRollerPower = 1.0, intakeFeedPower = 0.4, shootRollerPower = 1.0, shootFeedPower = 0.8;
    public static long intakeReverseDuration = 100;

    public enum State {
        IDLE,
        INTAKE,
        SHOOT_FEED,
        TEST
    }

    public double intakeCurrent;

    public State state = State.IDLE;

    public Intake(Robot robot) {
        this.robot = robot;

        roller = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "roller")},
            "roller", 2, 4,
            new double[] {1}, robot.sensors
        );

        feed = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "feed")},
            "feed", 2, 4,
            new double[] {1}, robot.sensors
        );
        /*
        index1 = new nPriorityServo(
                new Servo[]{robot.hardwareMap.get(Servo.class, "index1")},
                "index1", nPriorityServo.ServoType.AXON_MINI,
                0.027, 0.4, 0.03,
                new boolean[] {false},
                3, 7, true
        );

        index2 = new nPriorityServo(
                new Servo[]{robot.hardwareMap.get(Servo.class, "index2")},
                "index2", nPriorityServo.ServoType.AXON_MINI,
                0.027, 0.4, 0.03,
                new boolean[] {false},
                3, 7, true
        );
        */
        robot.hardwareQueue.addDevices(roller, feed);

        roller.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feed.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    long launchTime = System.currentTimeMillis();
    long turnedOffTime = 0;

    public void update() {
        switch (state) {
            case IDLE: {
                requestOff = false;

                roller.setTargetPower(0.0);
                feed.setTargetPower(System.currentTimeMillis() < turnedOffTime + intakeReverseDuration ? -intakeFeedPower : 0.0);

                if (requestIntake) {
                    requestIntake = false;
                    state = State.INTAKE;
                }

                if (requestShoot) {
                    requestShoot = false;
                    state = State.SHOOT_FEED;
                }

                break;
            }
            case INTAKE: {
                requestIntake = false;

                roller.setTargetPowerSmooth(intakeRollerPower * (reversed ? -1 : 1), 0.3);
                feed.setTargetPower(intakeFeedPower * (reversed ? -1 : 1));

                if (requestOff) {
                    requestOff = false;
                    requestIntake = false;
                    requestShoot = false;
                    state = State.IDLE;
                }

                if (requestShoot) {
                    requestShoot = false;
                    state = State.SHOOT_FEED;
                }

                break;
            }
            case SHOOT_FEED: {
                requestShoot = false;

                roller.setTargetPowerSmooth(shootRollerPower, 0.3);
                feed.setTargetPower(shootFeedPower);

                if (requestOff) {
                    requestOff = false;
                    requestIntake = false;
                    requestShoot = false;
                    state = State.IDLE;
                }

                if (requestIntake) {
                    requestIntake = false;
                    state = State.INTAKE;
                }

                break;
            }
            case TEST: {
                break;
            }
        }

        //intakeCurrent = roller.getCurrent();

        this.updateTelemetry();
    }

    public void reqIntake (boolean req) {
        requestIntake = req;
    }

    public void reqShoot (boolean req) {
        requestShoot = req;
    }

    public void reqOff (boolean req) { requestOff = req; turnedOffTime = System.currentTimeMillis(); }

    public void setRollerDirection(boolean reversed) { this.reversed = reversed; }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", this.state);
        LogUtil.intakeState.set(this.state.toString());
        TelemetryUtil.packet.put("Intake : reversed", reversed);
        LogUtil.intakeReversed.set(reversed);
        //TelemetryUtil.packet.put("Intake : current (AMPS)", intakeCurrent);
        //LogUtil.intakeReversed.set(reversed);

    }
}
