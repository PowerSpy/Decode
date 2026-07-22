package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class NewIntake {
    private final Robot robot;
    private final PriorityMotor roller;
    private PriorityCRServo flipper;
    private boolean requestIntake = false;
    private boolean requestOff = false;
    private boolean requestTransfer = false;
    private boolean reversed = false;

    private long transferStart = -1;
    private long transferWaitStart = -1;
    private long transferRetractStart = -1;

    public static double rollerPower = 1.0, rollerTransferPower = 1.0, rollerRetractPower = 1.0; // Placeholder
    public static double flipperTransferPower = 1.0, flipperRetractPower = 1.0; // Placeholder
    public static long transferTimeMillis = 300; // Placeholder
    public static long transferWaitMillis = 300; // Placeholder
    public static long transferRetractMillis = 300; // Placeholder


    public enum State {
        IDLE,
        INTAKE,
        TRANSFER_WAIT,
        TRANSFER,
        TRANSFER_RETRACT,
        TEST
    }

    public State state = State.IDLE;

    public NewIntake(Robot robot) {
        this.robot = robot;
        roller = new PriorityMotor(
                new DcMotorEx[] { robot.hardwareMap.get(DcMotorEx.class, "roller") },
                "roller", 2, 4,
                new double[] { 1 }, robot.sensors);

        flipper = new PriorityCRServo(
            new CRServo[] {robot.hardwareMap.get(CRServo.class, "flipper1"), robot.hardwareMap.get(CRServo.class, "flipper2")},
            "flipper", PriorityCRServo.ServoType.AXON_MINI,
            new boolean[] {false, true},
            2, 2
        );

        this.robot.hardwareQueue.addDevices(flipper);
        this.robot.hardwareQueue.addDevices(roller);
    }

    long turnedOffTime = 0;

    public void update() {
        switch (state) {
            case IDLE: {
                roller.setTargetPower(0.0);
                flipper.setTargetPower(0.0);
                if (requestIntake) {
                    requestIntake = false;
                    state = State.INTAKE;
                }

                if (requestTransfer)
                {
                    this.state = State.TRANSFER_WAIT;
                }

                break;
            }
            case INTAKE: {
                roller.setTargetPower(rollerPower * (reversed ? -1 : 1));

                if (requestOff)
                {
                    requestOff = false;
                    state = State.IDLE;
                }
                break;
            }
            case TRANSFER_WAIT: {
                if(this.transferWaitStart == -1)
                {
                    this.transferWaitStart = System.currentTimeMillis();
                }
                roller.setTargetPower(NewIntake.rollerTransferPower);
                if(System.currentTimeMillis()-this.transferWaitStart >= NewIntake.transferWaitMillis)
                {
                    this.state = State.TRANSFER;
                    this.transferWaitStart = -1;
                }
                break;
            }
            case TRANSFER: {
                if(this.transferStart == -1)
                {
                    this.transferStart = System.currentTimeMillis();
                }
                flipper.setTargetPower(NewIntake.flipperTransferPower);
                if(System.currentTimeMillis()-this.transferStart >= NewIntake.transferTimeMillis)
                {
                    this.state = State.TRANSFER_RETRACT;
                    this.transferStart = -1;
                }
            }
            case TRANSFER_RETRACT: {
                if(this.transferRetractStart == -1)
                {
                    this.transferRetractStart = System.currentTimeMillis();
                }
                flipper.setTargetPower(NewIntake.flipperRetractPower);
                roller.setTargetPower(NewIntake.rollerRetractPower);
                if(System.currentTimeMillis()-this.transferRetractStart >= NewIntake.transferRetractMillis)
                {
                    this.state = State.IDLE;
                    this.requestTransfer = false;
                    this.transferRetractStart = -1;
                }
            }
        }
        updateTelemetry();
    }

    public void requestIntake()
    {
        requestIntake = true;
    }

    public void reqOff()
    {
        requestOff = true;
        turnedOffTime = System.currentTimeMillis();
    }

    public void requestTransfer()
    {
        this.requestTransfer = true;
    }

    private void updateTelemetry() {
        TelemetryUtil.packet.put("NewIntake: state", this.state);
        TelemetryUtil.packet.put("NewIntake: reversed", reversed);
    }

    public void setRollerDirection(boolean reverse)
    {
        reversed = reverse;
    }
}
