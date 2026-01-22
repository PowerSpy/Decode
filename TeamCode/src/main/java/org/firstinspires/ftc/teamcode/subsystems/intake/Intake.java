package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Intake {
    private final Robot robot;
    public final PriorityMotor roller, feed;

    private boolean requestIntake = false, requestShoot = false, requestOff = false, reversed = false;

    public static double intakeRollerPower = 0.75, intakeFeedPower = 0.35, shootRollerPower = 0.8, shootFeedPower = 0.7;

    public enum State {
        IDLE,
        INTAKE,
        SHOOT_FEED,
        TEST
    }

    public State state = State.IDLE;

    public Intake(Robot robot) {
        this.robot = robot;

        roller = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "roller")},
            "roller", 2, 5,
            new double[] {1}, robot.sensors
        );

        feed = new PriorityMotor(
            new DcMotorEx[] {robot.hardwareMap.get(DcMotorEx.class, "feed")},
            "feed", 2, 5,
            new double[] {1}, robot.sensors
        );

        robot.hardwareQueue.addDevices(roller, feed);

        roller.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feed.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    long launchTime = System.currentTimeMillis();

    public void update() {
        switch (state) {
            case IDLE: {
                requestOff = false;

                roller.setTargetPower(0.0);
                feed.setTargetPower(0.0);

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

                roller.setTargetPower(intakeRollerPower * (reversed ? -1 : 1));
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

                roller.setTargetPower(shootRollerPower);
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

        this.updateTelemetry();
    }

    public void reqIntake (boolean req) {
        requestIntake = req;
    }

    public void reqShoot (boolean req) {
        requestShoot = req;
    }

    public void reqOff (boolean req) { requestOff = req; }

    public void setRollerDirection(boolean reversed) { this.reversed = reversed; }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("Intake : state", this.state);
        LogUtil.intakeState.set(this.state.toString());
        TelemetryUtil.packet.put("Intake : reversed", reversed);
        LogUtil.intakeReversed.set(reversed);
    }
}
