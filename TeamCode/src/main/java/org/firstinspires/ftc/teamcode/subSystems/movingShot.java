
package org.firstinspires.ftc.teamcode.subSystems;

import org.firstinspires.ftc.teamcode.dataStorage.LocalizationParm;
import org.firstinspires.ftc.teamcode.dataStorage.FlywheelParm;
import java.util.List;

public class movingShot{

    private final List<FlywheelParm> shotTable;

    public movingShot(
            List<FlywheelParm> shotTable) {

        if (shotTable == null || shotTable.size() < 2) {
            throw new IllegalArgumentException(
                    "Shot table needs at least two entries");
        }

        this.shotTable = shotTable;
    }

    public static double wrapAngle(double angle) {

        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    private FlywheelParm getShot(double distance) {

        if (distance <= shotTable.get(0).distance) {
            return shotTable.get(0);
        }

        int last = shotTable.size() - 1;

        if (distance >= shotTable.get(last).distance) {
            return shotTable.get(last);
        }

        for (int i = 0; i < last; i++) {

            FlywheelParm a = shotTable.get(i);
            FlywheelParm b = shotTable.get(i + 1);

            if (distance >= a.distance &&
                    distance <= b.distance) {

                double t =
                        (distance - a.distance) /
                                (b.distance - a.distance);

                return new FlywheelParm(
                        distance,

                        interpolate(
                                a.flywheelVelocity,
                                b.flywheelVelocity,
                                t),

                        interpolate(
                                a.hoodPosition,
                                b.hoodPosition,
                                t),

                        interpolate(
                                a.flightTime,
                                b.flightTime,
                                t)
                );
            }
        }

        return shotTable.get(last);
    }

    private double interpolate(
            double a,
            double b,
            double t) {

        return a + (b - a) * t;
    }

    public ShotSolution calculate(

            double robotX,
            double robotY,
            double robotHeading,

            double velocityX,
            double velocityY,

            double targetX,
            double targetY) {

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double distance = Math.hypot(dx, dy);

        FlywheelParm shot = getShot(distance);

        double virtualX = targetX;
        double virtualY = targetY;

        // Iteratively calculate the virtual target
        for (int i = 0; i < 5; i++) {

            virtualX =
                    targetX - velocityX * shot.flightTime;

            virtualY =
                    targetY - velocityY * shot.flightTime;

            dx = virtualX - robotX;
            dy = virtualY - robotY;

            distance = Math.hypot(dx, dy);

            shot = getShot(distance);
        }

        double fieldAngle = Math.atan2(dy, dx);

        double turretAngle =
                wrapAngle(fieldAngle - robotHeading);

        return new ShotSolution(
                turretAngle,
                distance,
                shot.flywheelVelocity,
                shot.hoodPosition,
                shot.flightTime,
                virtualX,
                virtualY
        );
    }

    public static class ShotSolution {

        public final double turretAngle;
        public final double distance;
        public final double flywheelVelocity;
        public final double hoodPosition;
        public final double flightTime;

        public final double virtualX;
        public final double virtualY;

        public ShotSolution(
                double turretAngle,
                double distance,
                double flywheelVelocity,
                double hoodPosition,
                double flightTime,
                double virtualX,
                double virtualY) {

            this.turretAngle = turretAngle;
            this.distance = distance;
            this.flywheelVelocity = flywheelVelocity;
            this.hoodPosition = hoodPosition;
            this.flightTime = flightTime;

            this.virtualX = virtualX;
            this.virtualY = virtualY;
        }
    }
}
