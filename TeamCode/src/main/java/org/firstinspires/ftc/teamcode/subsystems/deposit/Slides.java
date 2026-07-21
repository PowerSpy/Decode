package org.firstinspires.ftc.teamcode.subsystems.deposit;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
public class Slides
{
    public static double kSF = 0.0001; // Static friction (placeholder)
    public static double kG = 0.0001; // Gravity value (placeholder)
    public static PID pid = new PID(0.1, 0.1, 0.1); // Placeholders
    public static double minPower, maxPower;

    private Robot robot;
    private PriorityMotor slidesMotor;

    private double length, vel, targetLength;

    public Slides(Robot robot)
    {
        this.robot = robot;
        this.slidesMotor = new PriorityMotor(
                new DcMotorEx[]{
                        robot.hardwareMap.get(DcMotorEx.class, "slide1"),
                        robot.hardwareMap.get(DcMotorEx.class, "slide2")
                },
                "slides", 3, 5, new double[]{1, -1},
                robot.sensors
        );
        robot.hardwareQueue.addDevice(this.slidesMotor);
    }

    public void update()
    {
        this.length = this.robot.sensors.getSlidesPos();
        this.vel = this.robot.sensors.getSlidesVelocity();

        double ffTerms = Slides.kSF * Math.signum(this.vel) + Slides.kG;
        double power = pid.update(this.targetLength-this.length, Slides.minPower, Slides.maxPower) + ffTerms;
        this.slidesMotor.setTargetPower(power);
    }

    public void setTargetLength(double tLength)
    {
        this.targetLength = tLength;
    }

    public Robot getRobot()
    {
        return this.robot;
    }

    public double getLength()
    {
        return this.length;
    }

    public double getVel()
    {
        return this.vel;
    }
}
