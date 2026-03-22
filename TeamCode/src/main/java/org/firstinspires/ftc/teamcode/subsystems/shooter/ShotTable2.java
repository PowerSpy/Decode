package org.firstinspires.ftc.teamcode.subsystems.shooter;

import org.firstinspires.ftc.teamcode.utils.Lerp;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.Function;

public class ShotTable2 {
    private final TreeMap<Double, ShotPoint> table = new TreeMap<>(); // distance -> flywheel

    public ShotTable2() {
        // TODO Find new values. These are placeholders.
/*
        table.put( 44.0, new ShotPoint(400)
            .addValue(400, Math.toRadians(26.4))
        );
        table.put( 50.0, new ShotPoint(445)
            .addValue(445, Math.toRadians(26.4))
        );
        table.put( 60.0, new ShotPoint(465)
            .addValue(465, Math.toRadians(39.0))
            .addValue(450, Math.toRadians(32))
        );
        table.put( 70.0, new ShotPoint(523)
            .addValue(523, Math.toRadians(48.2))
            .addValue(520, Math.toRadians(47))
        );
        table.put( 80.0, new ShotPoint(525)
            .addValue(525, Math.toRadians(48.5))
            .addValue(527, Math.toRadians(49))
        );
        table.put( 83.0, new ShotPoint(525)
            .addValue(525, Math.toRadians(49.3))
        );
        table.put( 90.0, new ShotPoint(538)
            .addValue(538, Math.toRadians(49.0))
            .addValue(542, Math.toRadians(49))
        );
        table.put( 98.0, new ShotPoint(546)
            .addValue(546, Math.toRadians(49.3))
            .addValue(548, Math.toRadians(49))
        );
        table.put(110.0, new ShotPoint(580)
            .addValue(580, Math.toRadians(49.3))
            .addValue(560, Math.toRadians(37))
        );
        table.put(133.5, new ShotPoint(630)
            .addValue(630, Math.toRadians(51))
            .addValue(610, Math.toRadians(51.65))
            .addValue(580, Math.toRadians(45.38))
            .addValue(520, Math.toRadians(41.43))
        );
        table.put(140.0, new ShotPoint(660)
            .addValue(660, Math.toRadians(48.5))
            .addValue(635, Math.toRadians(50))
            .addValue(620, Math.toRadians(48.5))
            .addValue(600, Math.toRadians(38))
            .addValue(580, Math.toRadians(37.4))
            .addValue(560, Math.toRadians(39))
        );
        table.put(154.0, new ShotPoint(660)
            .addValue(660, Math.toRadians(48.5))
            .addValue(645, Math.toRadians(48))
            .addValue(620, Math.toRadians(50.11))
            .addValue(600, Math.toRadians(45))
            .addValue(570, Math.toRadians(45.4))
        );
*/
        // 2026-03-12
        table.put( 45.0, new ShotPoint(430)
            .addValue(430, Math.toRadians(26))
            .addValue(420, Math.toRadians(27))
            .addValue(400, Math.toRadians(27))
            .addValue(380, Math.toRadians(28))
        );
        table.put( 61.7, new ShotPoint(450)
            .addValue(500, Math.toRadians(27))
            .addValue(480, Math.toRadians(30))
            .addValue(450, Math.toRadians(36))
            .addValue(430, Math.toRadians(39))
        );
        table.put( 77.5, new ShotPoint(530)
            .addValue(530, Math.toRadians(49))
            .addValue(520, Math.toRadians(48))
            .addValue(490, Math.toRadians(45))
            .addValue(470, Math.toRadians(44))
            .addValue(450, Math.toRadians(41))
        );
        table.put( 95.0, new ShotPoint(550)
            .addValue(550, Math.toRadians(50))
            .addValue(500, Math.toRadians(49))
            .addValue(470, Math.toRadians(43))
        );
        table.put(136.8, new ShotPoint(640)
            .addValue(640, Math.toRadians(49))
            .addValue(630, Math.toRadians(48))
            .addValue(600, Math.toRadians(47))
            .addValue(570, Math.toRadians(43))
            .addValue(550, Math.toRadians(43))
        );
        table.put(153.4, new ShotPoint(640)
            .addValue(630, Math.toRadians(49))
            .addValue(620, Math.toRadians(48))
            .addValue(600, Math.toRadians(51))
            .addValue(580, Math.toRadians(43))
        );
    }

    public double getFlywheelForDistance(double distance) {
        return tableLerp(table, distance, (ShotPoint shotPoint) -> {
            if (shotPoint == null) return 0.0;
            return shotPoint.flywheelTargetVel;
        });
    }

    public double getLaunchAngleForDistanceAndFlywheel(double distance, double flywheelVel) {
        return tableLerp(table, distance, (ShotPoint shotPoint) -> {
            if (shotPoint == null) return 0.0;
            return tableLerp(shotPoint.launchTable, flywheelVel, (Double launchAngle) -> {
                if (launchAngle == null) return 0.0;
                return launchAngle;
            });
        });
    }

    private static <E> double tableLerp(TreeMap<Double, E> table, Double key, Function<E, Double> mapFunction) {
        Map.Entry<Double, E> lowEntry = table.floorEntry(key);
        Map.Entry<Double, E> highEntry = table.ceilingEntry(key);
        if (lowEntry == null && highEntry == null) return mapFunction.apply(null);
        if (lowEntry == null) return mapFunction.apply(highEntry.getValue());
        if (highEntry == null) return mapFunction.apply(lowEntry.getValue());
        if (lowEntry.getKey().equals(highEntry.getKey())) return mapFunction.apply(lowEntry.getValue());
        return Lerp.lerp(
            mapFunction.apply(lowEntry.getValue()),
            mapFunction.apply(highEntry.getValue()),
            (key - lowEntry.getKey()) / (highEntry.getKey() - lowEntry.getKey())
        );
    }

    private static class ShotPoint {
        private final double flywheelTargetVel;
        private final TreeMap<Double, Double> launchTable = new TreeMap<>(); // flywheel -> launch angle

        private ShotPoint(double flywheelTargetVel) {
            this.flywheelTargetVel = flywheelTargetVel;
        }

        private ShotPoint addValue(double flywheelVel, double launchAngle) {
            launchTable.put(flywheelVel, launchAngle);
            return this;
        }
    }
}

/* Visualizer: Run snippet then dump into Desmos 3D
        ShotTable2 table = new ShotTable2();
        for (Map.Entry<Double, ShotTable2.ShotPoint> entry : table.table.entrySet()) {
            ShotTable2.ShotPoint shotPoint = entry.getValue();
            for (Map.Entry<Double, Double> entry2 : shotPoint.launchTable.entrySet()) {
                System.out.println(String.format(Locale.US, "%3.0f\t%3.0f\t%3.0f", entry.getKey(), entry2.getKey(), Math.toDegrees(entry2.getValue())));
            }
        }
        System.out.println("\n\n");
        for (double d = 35; d <= 160; d += 5) {
            double tv = table.getFlywheelForDistance(d);
            System.out.println(String.format(Locale.US, "%3.0f\t%3.0f\t%3d", d, tv, 0));
            for (double v = 390; v <= 670; v += 10) {
                double h = table.getLaunchAngleForDistanceAndFlywheel(d, v);
                System.out.println(String.format(Locale.US, "%3.0f\t%3.0f\t%3.0f", d, v, Double.isNaN(h) ? 100.0 : Math.toDegrees(h)));
            }
        }
*/

