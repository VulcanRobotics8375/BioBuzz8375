//stolen from brogan m pratt vid

package org.firstinspires.ftc.teamcode.subSystems;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import robotcore.Subsystem;
import java.util.Locale;
import com.acmerobotics.dashboard.config.Config;
@Configurable //so we can tune using the FTC dashboard
public class fieldCentricMecanum extends Subsystem{
    //private Telemetry telemetry;
    public GoBildaPinpointDriver odo; // Declare   OpMode member for the Odometry Computer
    double oldTime = 0;
    //Motor and pinpoint declaration
    public DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Heading Lock Variables
    private double headingSetpoint = 0.0;
    private boolean isLocking = false;
    public static double STRAFE_GAIN = 1.2; // >1 boosts sideways to match forward (try 1.1-1.2 after Mecanum Wheel Test)
    // ---- power saving: soft start ----
    // Motors draw HUGE current when they jump from 0 to full power (that's what browns out the hub).
    // This only limits SPEEDING UP: power can rise at most ACCEL_PER_S per second (5 = 0 -> full in 0.2 s).
    // Slowing down, stopping and letting go of the stick are still INSTANT (no extra coasting).
    // 0 = off (old behavior). Higher = snappier, lower = gentler on the battery.
    public static double ACCEL_PER_S = 0;   // OFF (drivers found 5 sluggish). Try 10-15 if you want it back gently.
    private final double[] lastPow = new double[4];
    private long lastDriveNanos = 0;
    public static double kP = 0; //tune to adjust correction strength for anti-drifting.Tuned 8/16/26 by Zara

    @Override
    public void init(OpMode opMode) {
        //this.telemetry = telemetry;
        //connect actual motors to programmable objects
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        //odo.setOffsets(-142.93, 30.469, DistanceUnit.MM);
        odo.setOffsets(-3.278, -2.778, DistanceUnit.INCH);
        odo.setEncoderResolution(74.5027025034, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        //set directions (stole from DriveTrainTest hopefully works) -- needed to reverse them -- zld
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double axial, double lateral, double yaw) {
        //odo.update(); moved to fieldCentric() to prevent double updating

        lateral *= STRAFE_GAIN;
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        double maxPower = 1.0;
        double maxSpeed = 1.0; //can change for outreach events, etc. so kids can drive without breaking anything or hurting anyone

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        //planning to pid tune to get velocity >> power to prevent weird drift. also could double for auton. actuall ycan i js auton tune and use the same controlelrs here? that would be cool.
        long now = System.nanoTime();
        double dt = lastDriveNanos == 0 ? 0 : Math.min((now - lastDriveNanos) / 1e9, 0.1);
        lastDriveNanos = now;
        double step = ACCEL_PER_S * dt;
        double fl = softStart(0, maxSpeed * (frontLeftPower / maxPower), step);
        double bl = softStart(1, maxSpeed * (backLeftPower / maxPower), step);
        double fr = softStart(2, maxSpeed * (frontRightPower / maxPower), step);
        double br = softStart(3, maxSpeed * (backRightPower / maxPower), step);
        frontLeftDrive.setPower(fl);
        backLeftDrive.setPower(bl);
        frontRightDrive.setPower(fr);
        backRightDrive.setPower(br);
        if (showTelemetry) {
            telemetry.addData("drive power fl / fr / bl / br", "%.2f / %.2f / %.2f / %.2f", fl, fr, bl, br);
        }

    }

    /** Limit how fast power can RISE (soft start). Lowering power / stopping / reversing to 0 = instant. */
    private double softStart(int i, double target, double step) {
        double last = lastPow[i];
        double out;
        if (ACCEL_PER_S <= 0) {
            out = target;                                          // feature off
        } else {
            if (last != 0 && Math.signum(target) != Math.signum(last)) last = 0;   // reversing: drop to 0 right away
            if (Math.abs(target) <= Math.abs(last)) out = target;  // slowing down: instant
            else out = last + Math.signum(target) * Math.min(Math.abs(target) - Math.abs(last), step);
        }
        lastPow[i] = out;
        return out;
    }

    public void fieldCentric(double axial, double lateral, double yaw) {
        fieldCentric(axial, lateral, yaw, Double.NaN);
    }

    /**
     * driveHeading = the heading (rad) used to turn the sticks. 0 = the robot faces "stick forward".
     * NaN = use the raw Pinpoint heading (stick forward = whatever way the robot faced at init).
     */
    public void fieldCentric(double axial, double lateral, double yaw, double driveHeading) {
        odo.update(); //update odo positions and class sensors
        Pose2D pos = odo.getPosition();

        double currentHeading = pos.getHeading(AngleUnit.RADIANS);
        if (Double.isNaN(driveHeading)) driveHeading = currentHeading;

        // Heading Lock Controller
        if (Math.abs(yaw) > 0.05) {
            //if driver is actively rotating the robot: tracks the new setpoint
            headingSetpoint = currentHeading;
            isLocking = false;
        } else {
            //Driver not turning: sets current heading as the lock heading
            if (!isLocking) {
                headingSetpoint = currentHeading;
                isLocking = true;
            }
            //Calculate shortest path steering error (-PI to PI)
            double headingError = headingSetpoint - currentHeading;
            headingError = AngleUnit.normalizeRadians(headingError);

            //Apply proportional gain to override target yaw speed
            yaw = -headingError * kP;
        }

        //Odometry for field centric stuff (vector translatino)
        double theta = Math.atan2(axial, lateral);
        double r = Math.hypot(lateral, axial);

        if (showTelemetry) {
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
        }

        theta = AngleUnit.normalizeRadians(theta - driveHeading);

        double newAxial = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newAxial, newStrafe, yaw);
    }
}
