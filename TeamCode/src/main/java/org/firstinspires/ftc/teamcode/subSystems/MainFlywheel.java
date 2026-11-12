//package org.firstinspires.ftc.teamcode.subSystems;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.dataStorage.FlywheelParm;
//import org.firstinspires.ftc.teamcode.dataStorage.LocalizationParm;
//
//import java.util.Arrays;
//import java.util.List;
//
//import robotcore.Subsystem;
//
//public class MainFlywheel extends Subsystem {
//
//    private DcMotorEx flywheel;
//    private DcMotorEx turret;
//
//    private Servo hood;
//
//    private movingShot controller;
//
//    // Example turret calibration
//    private static final double TURRET_TICKS_PER_REV = 1440.0;
//
//    private static final double TURRET_KP = 0.8;
//
//    private static final double MAX_TURRET_POWER = 0.5;
//
//    private static final double MAX_TURRET_ANGLE =
//            Math.toRadians(150);
//
//    private static final double MIN_TURRET_ANGLE =
//            Math.toRadians(-150);
//
//    @Override
//    public void init(OpMode opMode) {
//
//        // Hardware mapping
//        flywheel = hardwareMap.get(
//                DcMotorEx.class, "flywheel");
//
//        turret = hardwareMap.get(
//                DcMotorEx.class, "turret");
//
//        hood = hardwareMap.get(
//                Servo.class, "hood");
//
//        flywheel.setMode(
//                DcMotor.RunMode.RUN_USING_ENCODER);
//
//        turret.setMode(
//                DcMotor.RunMode.RUN_USING_ENCODER);
//
//        flywheel.setZeroPowerBehavior(
//                DcMotor.ZeroPowerBehavior.FLOAT);
//
//        // Example calibration table.
//        // Replace these values with real test data!
//        List<FlywheelParm> shots = Arrays.asList(
//
//                new FlywheelParm(
//                        24, 1200, 0.30, 0.45),
//
//                new FlywheelParm(
//                        36, 1500, 0.38, 0.55),
//
//                new FlywheelParm(
//                        48, 1800, 0.45, 0.65),
//
//                new FlywheelParm(
//                        60, 2100, 0.52, 0.75)
//        );
//
//        controller = new movingShot(shots);
//
//        telemetry.addLine("Shooting system initialized");
//
//        telemetry.update();
//    }
//
//    public static void runFlywheel() {
//
//        // =================================
//        // LOCALIZATION
//        // Replace with your actual system
//        // =================================
//
//        double robotX = 0;
//        double robotY = 0;
//
//        double robotHeading = 0;
//
//        double velocityX = 0;
//        double velocityY = 0;
//
//        // =================================
//        // TARGET COORDINATES
//        // =================================
//
//        double targetX = 144;
//        double targetY = 144;
//
//        // =================================
//        // CALCULATE SHOOTING SOLUTION
//        // =================================
//
//        movingShot.ShotSolution solution =
//                controller.calculate(
//
//                        robotX,
//                        robotY,
//                        robotHeading,
//
//                        velocityX,
//                        velocityY,
//
//                        targetX,
//                        targetY
//                );
//
//        // =================================
//        // TURRET CONTROL
//        // =================================
//
//        double currentTurretAngle =
//                getTurretAngleRadians();
//
//        double turretError =
//                solution.turretAngle -
//                        currentTurretAngle;
//
//        // Do not wrap through a mechanical limit.
//        double turretTarget = Math.max(
//                MIN_TURRET_ANGLE,
//                Math.min(
//                        MAX_TURRET_ANGLE,
//                        solution.turretAngle
//                )
//        );
//
//        turretError =
//                turretTarget - currentTurretAngle;
//
//        double turretPower =
//                TURRET_KP * turretError;
//
//        turretPower = Math.max(
//                -MAX_TURRET_POWER,
//                Math.min(
//                        MAX_TURRET_POWER,
//                        turretPower
//                )
//        );
//
//        turret.setPower(turretPower);
//
//        // =================================
//        // FLYWHEEL CONTROL
//        // =================================
//
//        flywheel.setVelocity(
//                solution.flywheelVelocity
//        );
//
//        // =================================
//        // HOOD CONTROL
//        // =================================
//
//        hood.setPosition(
//                solution.hoodPosition
//        );
//
//        // =================================
//        // READY CHECK
//        // =================================
//
//        double flywheelError = Math.abs(
//                flywheel.getVelocity() -
//                        solution.flywheelVelocity
//        );
//
//        boolean turretReady =
//                Math.abs(turretError) <
//                        Math.toRadians(2);
//
//        boolean flywheelReady =
//                flywheelError < 50;
//
//        boolean readyToShoot =
//                turretReady && flywheelReady;
//
//        // =================================
//        // TELEMETRY
//        // =================================
//
//        telemetry.addData(
//                "Robot X", robotX);
//
//        telemetry.addData(
//                "Robot Y", robotY);
//
//        telemetry.addData(
//                "Heading",
//                Math.toDegrees(robotHeading));
//
//        telemetry.addData(
//                "Distance",
//                solution.distance);
//
//        telemetry.addData(
//                "Virtual X",
//                solution.virtualX);
//
//        telemetry.addData(
//                "Virtual Y",
//                solution.virtualY);
//
//        telemetry.addData(
//                "Turret Target",
//                Math.toDegrees(solution.turretAngle));
//
//        telemetry.addData(
//                "Turret Error",
//                Math.toDegrees(turretError));
//
//        telemetry.addData(
//                "Flywheel Target",
//                solution.flywheelVelocity);
//
//        telemetry.addData(
//                "Flywheel Actual",
//                flywheel.getVelocity());
//
//        telemetry.addData(
//                "Hood Position",
//                solution.hoodPosition);
//
//        telemetry.addData(
//                "Flight Time",
//                solution.flightTime);
//
//        telemetry.addData(
//                "Ready To Shoot",
//                readyToShoot);
//
//        telemetry.update();
//    }
//    private double getTurretAngleRadians() {
//
//        double ticks = turret.getCurrentPosition();
//
//        return (ticks / TURRET_TICKS_PER_REV) *
//                2.0 * Math.PI;
//    }
//}
