package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.drive.PathfollowerDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.NewIntake;
import org.firstinspires.ftc.teamcode.subsystems.park.Park;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.RunMode;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class Robot {
    public HardwareMap hardwareMap;
    public HardwareQueue hardwareQueue;

    public Sensors sensors;
    public PathfollowerDrivetrain drivetrain;
    public NewIntake intake;
    public Shooter shooter;
    public Deposit deposit;
    public Park park;

    private BooleanSupplier stopChecker = null;
    public ArrayList<Consumer<Canvas>> canvasDrawTasks = new ArrayList<>();

    public Robot(HardwareMap hardwareMap) { this(hardwareMap, false); }

    public Robot(HardwareMap hardwareMap, boolean useVision) {
        this.hardwareMap = hardwareMap;
        hardwareQueue = new HardwareQueue();

        TelemetryUtil.setup();
        LogUtil.reset();

        sensors = new Sensors(this);
        drivetrain = new PathfollowerDrivetrain(this);
        intake = new NewIntake(this);
//        shooter = new Shooter(this);
        park = new Park(this);
        deposit = new Deposit(this);
//        sensors.resetTurretAngleEncoder();
    }

    public void update() {
        START_LOOP();

        if (this.stopChecker != null && this.stopChecker.getAsBoolean()) return;

        sensors.update();

        drivetrain.update();
        intake.update();
//        shooter.update();
        deposit.update();
        park.update();

        if (this.stopChecker != null && this.stopChecker.getAsBoolean()) return;

        hardwareQueue.update();

        this.updateTelemetry();
    }

    /**
     * Sets the condition that should stop waiting (waitWhile)
     * @param func the function to check (return true to stop)
     */
    public void setStopChecker(BooleanSupplier func) { this.stopChecker = func; }

    /**
     * Waits while a condition is true
     * @param func the function to check
     */
    public void waitWhile(BooleanSupplier func) {
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && func.getAsBoolean());
    }

    /**
     * Waits while a condition is true
     * @param func the function to check
     */
    public void waitWhileWithTimeout(BooleanSupplier func, long duration) {
        long start = System.currentTimeMillis();
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && System.currentTimeMillis() - start < duration && func.getAsBoolean());
    }

    /**
     * Waits for a duration
     * @param duration the duration in milliseconds
     */
    public void waitFor(long duration) {
        long start = System.currentTimeMillis();
        do {
            update();
        } while (!this.stopChecker.getAsBoolean() && System.currentTimeMillis() - start < duration);
    }

    public void updateTelemetry() {
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        for (Consumer<Canvas> task : canvasDrawTasks) task.accept(canvas);

        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());

        TelemetryUtil.sendTelemetry();
        LogUtil.send();
    }
}
