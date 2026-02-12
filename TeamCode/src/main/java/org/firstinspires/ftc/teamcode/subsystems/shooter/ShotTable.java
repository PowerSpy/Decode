package org.firstinspires.ftc.teamcode.subsystems.shooter;
import java.util.TreeMap;

class ShotSetpoint {
    public final double flywheelVel;
    public final double hoodAngle;
    public final double timeOfFlight;

    public ShotSetpoint(double shootVel, double hood, double time) {
        this.flywheelVel = shootVel;
        this.hoodAngle = hood;
        this.timeOfFlight = time;
    }

    public ShotSetpoint lerp(ShotSetpoint other, double fraction) {
        return new ShotSetpoint(
                this.flywheelVel + (other.flywheelVel - this.flywheelVel) * fraction,
                this.hoodAngle + (other.hoodAngle - this.hoodAngle) * fraction,
                this.timeOfFlight + (other.timeOfFlight - this.timeOfFlight) * fraction
        );
    }
}

public class ShotTable {
    private final TreeMap<Double, ShotSetpoint> table = new TreeMap<>();
    public double flywheelMultiplier = 1.0;
    public double hoodMultiplier = 1.0;

    public void addSetpoint(double distanceInches, ShotSetpoint setpoint) {
        table.put(distanceInches, setpoint);
    }

    public ShotSetpoint getSetpoint(double distanceInches) {
        Double lowIndex = table.floorKey(distanceInches); //this is to get the point above what we want for extrapolation
        Double higIndex = table.ceilingKey(distanceInches); //this is to get the point below what we want for extrapolation
        ShotSetpoint baseSetpoint;

        if (lowIndex == null && higIndex == null) return null; //case where both are null
        if (lowIndex == null) baseSetpoint = table.get(higIndex); //case where we only have highIndex
        else if (higIndex == null) baseSetpoint = table.get(lowIndex); //case where we only have lowIndex
        else if (lowIndex.equals(higIndex)) baseSetpoint = table.get(lowIndex); //case where both are the same
        else {
            double fraction = (distanceInches - lowIndex) / (higIndex - lowIndex); //interpolation based on the high and lower from input on the distance value
            baseSetpoint = table.get(lowIndex).lerp(table.get(higIndex), fraction); //smooths using linear interpolation between the previous factor to find the flywheelVelocity, hoodAngle, and time of flight.
        }

        return new ShotSetpoint(
                baseSetpoint.flywheelVel * flywheelMultiplier, //offset created just in case we are consistently off, we can tune this
                baseSetpoint.hoodAngle * hoodMultiplier, //offset created just in case we are consistently off, we can tune this
                baseSetpoint.timeOfFlight //this one shouldn't change too much we can make a multiplier in the future if the need comes for SOTM.
        );
    }
}