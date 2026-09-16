package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import robotcore.Subsystem;
@Config
public class MotorOuttake extends Subsystem {
    boolean isOn;
    public DcMotorEx OuttakeMotor;

    double lastError = 0, integralSum = 0;
    public static double kP = 200, kI = 8, kD = 1;
    public static double rpm = 3000;
    FtcDashboard dashboard;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init(OpMode opMode) {
        //instantiateSubsystem(opMode);

        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        dashboard = FtcDashboard.getInstance();
    }

    public void runOuttake() {
        double targetTicksPerSec = (rpm / 60.0) * 28.0;
        double actualTicksPerSec = -1*OuttakeMotor.getVelocity();
        double actualRPM = (actualTicksPerSec / 28.0) * 60.0;

        if(gamepad1.rightBumperWasPressed()){
            rpm += 100;
        }
        else if(gamepad1.leftBumperWasPressed()){
            rpm -= 100;
        }


        if (gamepad1.xWasPressed()) {
            integralSum = 0;
            lastError = 0;
            timer.reset();
            if (isOn) {
                isOn = false;
            } else {
                isOn = true;
            }
        }
        if (isOn) {
            double error = targetTicksPerSec - actualTicksPerSec;
            double dt = timer.seconds();
            timer.reset();

            integralSum += error * dt;

            double derivative = (error - lastError) / dt;
            lastError = error;

            double output = (12 * targetTicksPerSec)
                    + (kP * error)
                    + (kI * integralSum)
                    + (kD * derivative);

            output = output / 32767.0;
            output = Math.max(-1, Math.min(1, output));

            OuttakeMotor.setPower(output);
        } else {
            OuttakeMotor.setPower(0);
        }

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("rpm:", rpm);
        packet.put("actualRPM", actualRPM);
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("rpm", rpm);
        telemetry.update();
    }

}




