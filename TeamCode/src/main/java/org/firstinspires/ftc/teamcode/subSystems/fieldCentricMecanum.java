//stolen from brogan m pratt vid

package org.firstinspires.ftc.teamcode.subSystems;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import robotcore.Subsystem;
import java.util.Locale;
import com.acmerobotics.dashboard.config.Config;
@Config //so we can tune using the FTC dashboard
public class fieldCentricMecanum extends Subsystem{
    //private Telemetry telemetry;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;
    //Motor and pinpoint declaration
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Heading Lock Variables
    private double headingSetpoint = 0.0;
    private boolean isLocking = false;
    public static double kP = 0.9; //tune to adjust correction strength for anti-drifting.Tuned 8/16/26 by Zara

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        //this.telemetry = telemetry;
        //connect actual motors to programmable objects
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        //odo.setOffsets(-142.93, 30.469, DistanceUnit.MM);
        odo.setOffsets(-156.626489257812, 42.521887207031, DistanceUnit.MM);
        odo.setEncoderResolution(74.5027025034, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        //set directions (stole from DriveTrainTest hopefully works) -- needed to reverse them -- zld
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void drive(double axial, double lateral, double yaw) {
        //odo.update(); moved to fieldCentric() to prevent double updating

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
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));

    }

    public void fieldCentric(double axial, double lateral, double yaw) {
        odo.update(); //update odo positions and sensors
        Pose2D pos = odo.getPosition();

        double currentHeading = pos.getHeading(AngleUnit.RADIANS);

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

        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.update();

        theta = AngleUnit.normalizeRadians(theta - pos.getHeading(AngleUnit.RADIANS));

        double newAxial = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newAxial, newStrafe, yaw);
    }


    @Override
    public void init(OpMode opMode) {

    }
}
