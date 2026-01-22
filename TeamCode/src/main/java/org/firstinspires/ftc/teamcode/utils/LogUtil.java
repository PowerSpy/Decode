package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@Config
public class LogUtil {
    private static Datalogger datalogger = null;

    public static class StateField extends Datalogger.LoggableField {
        private String value = "";

        public StateField(String name) { super(name); }

        @Override
        public void writeToBuffer(StringBuilder out) { out.append(value); }

        public void set(String newValue) {
            if (!newValue.equals(value)) {
                TelemetryUtil.packet.put("LogUtil : stateTransition", name + "," + value);
                LogUtil.stateTransition = true;
            }
            value = newValue;
        }

        @NonNull
        @Override
        public String toString() { return value; }
    }

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    //public static Datalogger.GenericField loopTime = new Datalogger.GenericField("loopTime");
    public static Datalogger.GenericField intakeState = new Datalogger.GenericField("intakeState");
    public static StateField shooterState = new StateField("shooterState");
    public static Datalogger.GenericField turretAngle = new Datalogger.GenericField("turretAngle");
    public static Datalogger.GenericField turretTarget = new Datalogger.GenericField("turretTarget");
    public static Datalogger.GenericField flywheelVelocity = new Datalogger.GenericField("flywheelVelocity");
    public static Datalogger.GenericField flywheelTarget = new Datalogger.GenericField("flywheelTarget");
    public static Datalogger.GenericField hoodAngle = new Datalogger.GenericField("hoodAngle");
    public static Datalogger.GenericField intakeReversed = new Datalogger.GenericField("intakeReversed");
    public static Datalogger.GenericField driveState = new Datalogger.GenericField("driveState");
    public static Datalogger.GenericField driveCurrentX = new Datalogger.GenericField("driveCurrentX");
    public static Datalogger.GenericField driveCurrentY = new Datalogger.GenericField("driveCurrentY");
    public static Datalogger.GenericField driveCurrentAngle = new Datalogger.GenericField("driveCurrentAngle");
    //public static Datalogger.GenericField driveTargetX = new Datalogger.GenericField("driveTargetX");
    //public static Datalogger.GenericField driveTargetY = new Datalogger.GenericField("driveTargetY");
    //public static Datalogger.GenericField driveTargetAngle = new Datalogger.GenericField("driveTargetAngle");
    public static Datalogger.GenericField drivePath = new Datalogger.GenericField("drivePath");
    public static Datalogger.GenericField parkState = new Datalogger.GenericField("parkState");
    public static Datalogger.GenericField parkAngle = new Datalogger.GenericField("parkAngle");

    private static int loopCountBeforeWrite;

    public static boolean DISABLED = true;
    public static boolean stateTransition = false;
    public static boolean drivePositionReset = false;

    public static void reset() {
        if (datalogger != null) {
            datalogger = null;
        }
    }

    public static void init() {
        loopCountBeforeWrite = 0;

        long timeNow = System.currentTimeMillis();
        String fileName = "Log_" + timeNow + "_"
            + new SimpleDateFormat("yyyy-MM-dd_HH.mm.ss", Locale.US).format(new Date(timeNow))
            + "_CAT4_Decode_" + Globals.RUNMODE.toString();
        TelemetryUtil.packet.put("LogUtil : filename", fileName);

        if (datalogger != null) throw new IllegalStateException("LogUtil was already initialized");
        if (DISABLED) return;

        datalogger = new Datalogger.Builder()
            // Pass through the filename
            .setFilename(fileName)
            // Request an automatic timestamp field
            .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
            // Tell it about the fields we care to log.
            // Note that order *IS* important here! The order in which we list
            // the fields is the order in which they will appear in the log.
            .setFields(
                intakeState,
                shooterState,
                turretAngle,
                turretTarget,
                flywheelVelocity,
                flywheelTarget,
                hoodAngle,
                intakeReversed,
                driveCurrentX,
                driveCurrentY,
                driveCurrentAngle,
                drivePath,
                driveState
            )
            .build();
    }

    public static void send() {
        if (datalogger == null) return;
        --loopCountBeforeWrite;
        if (loopCountBeforeWrite <= 0 || stateTransition || drivePositionReset) {
            if (drivePositionReset) driveState.set("[reset]");
            datalogger.writeLine();
            loopCountBeforeWrite = 5;
            stateTransition = false;
            drivePositionReset = false;
            TelemetryUtil.packet.put("LogUtil : stateTransition", "[ none ]");
        }
    }
}
