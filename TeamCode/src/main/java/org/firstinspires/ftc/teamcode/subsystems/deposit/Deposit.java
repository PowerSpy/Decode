package org.firstinspires.ftc.teamcode.subsystems.deposit;

import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

public class Deposit {
    public enum State {
        IDLE, // Linear slides down and Bucket in default position
        RAISE, // Raises up
        DUMP_WAIT, // Dumps ball
        DUMP, // Dumps ball
        DUMP_RETRACT, // Dumps ball
        LOWER, // Lowers back to IDLE position,
        TEST
    }

    public static double slidesLoweredLength = 0.0; // Placeholder
    public static double slidesRaisedLength = 1.0; // Placeholder
    public static double completionThresholdSlides = 0.1, completionThresholdAngle = 0.1; // Placeholder
    public static double holdBucketArm = 0.1, holdBucket = 0.1, holdSlides = 0.1; // Placeholder
    public static double prepareDumpBucketArm = 0.1; // Placeholder
    public static double prepareDumpBucket = 0.1; // Placeholder
    public static double dumpBucket = 0.1; // Placeholder
    public static double prepareDumpSlidesLength = 0.1; // Placeholder
    public static long dumpBucketTimeMillis = 300; // Placeholder


    public State state = State.IDLE;
    private final Robot robot;
    //public final PriorityMotor slides;
    public final Slides slides;
    public final nPriorityServo bucketArmServos; // Controls arm connected to bucket
    public final nPriorityServo bucketServo; // This one controls the bucket
    public boolean requestRaise = false, requestDump = false, requestDown = false;

    private long dumpStartTime = -1;
    public Deposit(Robot robot) {
        this.robot = robot;

        /*
        slides = new PriorityMotor(
                new DcMotorEx[]{
                        robot.hardwareMap.get(DcMotorEx.class, "slide1"),
                        robot.hardwareMap.get(DcMotorEx.class, "slide2")
                },
                "slides", 3, 5, new double[]{1, -1},
                robot.sensors
        );
        */

        bucketArmServos = new nPriorityServo( // TODO: Numbers need to be calibrated (these are guesses)
                new Servo[]{
                        robot.hardwareMap.get(Servo.class, "bucketArmServo1"),
                        robot.hardwareMap.get(Servo.class, "bucketArmServo2")
                },
                "bucketArm", nPriorityServo.ServoType.AXON_MINI,
                0, 0.5, 0, new boolean[]{false}, 2, 3
        );

        bucketServo = new nPriorityServo( // TODO: Numbers need to be calibrated
                new Servo[]{
                        robot.hardwareMap.get(Servo.class, "bucketServo1")
                },
                "bucket", nPriorityServo.ServoType.AXON_MINI,
                0, 0.5, 0, new boolean[]{false}, 2, 3
        );

        slides = new Slides(this.robot);

        robot.hardwareQueue.addDevices(bucketArmServos);
        robot.hardwareQueue.addDevices(bucketServo);

    }

    public void holdPositions()
    {
        this.bucketServo.setTargetAngle(Deposit.holdBucket);
        this.bucketArmServos.setTargetAngle(Deposit.holdBucketArm);
        this.slides.setTargetLength(Deposit.holdSlides);
    }

    public boolean inHoldPositions()
    {
        return (Math.abs(this.bucketServo.getCurrentAngle() - Deposit.holdBucket) < Deposit.completionThresholdAngle) &&
                (Math.abs(this.bucketArmServos.getCurrentAngle() - Deposit.holdBucketArm) < Deposit.completionThresholdAngle) &&
                (Math.abs(this.slides.getLength() - Deposit.holdSlides) < Deposit.completionThresholdSlides);
    }

    public void update() {
        switch (this.state) {
            case IDLE: {
                if (this.requestRaise) {
                    this.state = State.RAISE;
                }

                if(this.requestDown)
                {
                    this.state = State.LOWER;
                }

                if(this.requestDump)
                {
                    this.state = State.DUMP_WAIT;
                }

                break;
            }
            case RAISE: {
                this.slides.setTargetLength(Deposit.slidesRaisedLength);

                if(Math.abs(this.slides.getLength()-Deposit.slidesRaisedLength) < Deposit.completionThresholdSlides)
                {
                    this.state = State.IDLE;
                }
                break;
            }
            case DUMP_WAIT: {
                this.bucketServo.setTargetAngle(Deposit.prepareDumpBucket);
                this.bucketArmServos.setTargetAngle(Deposit.prepareDumpBucketArm);
                this.slides.setTargetLength(Deposit.prepareDumpSlidesLength);

                if(Math.abs(this.bucketServo.getCurrentAngle()-Deposit.prepareDumpBucket) < Deposit.completionThresholdAngle &&
                        Math.abs(this.bucketArmServos.getCurrentAngle()-Deposit.prepareDumpBucketArm) < Deposit.completionThresholdAngle &&
                        Math.abs(this.slides.getLength()-Deposit.prepareDumpSlidesLength) < Deposit.completionThresholdSlides)
                {
                    this.state = State.DUMP;
                }
                break;
            }
            case DUMP: {
                if(this.dumpStartTime == -1)
                {
                    this.dumpStartTime = System.currentTimeMillis();
                }

                this.bucketServo.setTargetAngle(Deposit.dumpBucket);

                if (System.currentTimeMillis()-this.dumpStartTime >= Deposit.dumpBucketTimeMillis)
                {
                    this.state = State.DUMP_RETRACT;
                    this.dumpStartTime = -1;
                }
                break;
            }
            case DUMP_RETRACT: {
                holdPositions();

                if(this.inHoldPositions())
                {
                    this.state = State.IDLE;
                }
                break;
            }
            case LOWER: {
                this.slides.setTargetLength(Deposit.slidesLoweredLength);

                if(Math.abs(this.slides.getLength()-Deposit.slidesLoweredLength) < Deposit.completionThresholdSlides)
                {
                    this.state = State.IDLE;
                }
                break;
            }
            case TEST: {
                break;
            }


        }
    }
}



