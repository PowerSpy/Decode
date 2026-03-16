package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.priority.nPriorityServo;

@Config
public class Turret {
    private final Robot robot;
    public final nPriorityServo turret;
    private double lastTurretTarget = 0.0;
    private double targetTurretAngle = 0.0;
    private double targetTurretAngleVel = 0.0;
    public static double targetTurretAngleVelFilter = 0.9;
    public static double targetPredictNumLoops = 1;

    public Turret(Robot robot) {
        this.robot = robot;

        turret = new nPriorityServo(
                new Servo[] {robot.hardwareMap.get(Servo.class, "turret1"), robot.hardwareMap.get(Servo.class, "turret2")},
                "turret", nPriorityServo.ServoType.AXON_MINI_EXTENDED,
                0, 1, 0.08,
                new boolean[] {true, true},
                5, 6
        );

        robot.hardwareQueue.addDevice(turret);
    }

    public void update() {
        // Turret PIDF
        targetTurretAngleVel = targetTurretAngleVel * (1 - targetTurretAngleVelFilter) + (targetTurretAngle - lastTurretTarget) / robot.sensors.loopTime * targetTurretAngleVelFilter;
        targetTurretAngleVel = Utils.minMaxClip(targetTurretAngleVel, -150, 150);
        lastTurretTarget = targetTurretAngle;
        turret.setTargetAngle(targetTurretAngle + targetTurretAngleVel * robot.sensors.loopTime * targetPredictNumLoops);

        TelemetryUtil.packet.put("Turret : Target Angle (deg)", Math.toDegrees(targetTurretAngle));
        TelemetryUtil.packet.put("Turret : servo predicted current angle", Math.toDegrees(turret.getCurrentAngle()));
        TelemetryUtil.packet.put("Turret : servo target", Math.toDegrees(turret.getTargetAngle()));

        LogUtil.turretTarget.set(targetTurretAngle);
    }

    public void setTargetAngle(double targetAngle) { targetTurretAngle = Sensors.turretAngleClip(targetAngle); }

    public double getTargetAngle() { return targetTurretAngle; }

    public boolean inPosition() { return turret.inPosition(); }
}
