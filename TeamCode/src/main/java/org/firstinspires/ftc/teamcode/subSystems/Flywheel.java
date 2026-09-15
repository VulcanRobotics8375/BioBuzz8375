package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import robotcore.Subsystem;

public class Flywheel extends Subsystem {
    boolean isOn = false, rightTuning = true;
    public DcMotorEx OuttakeMotor = null;
    public double lowVelocity = 900, highVelocity = 1500, curTargetVelocity = highVelocity;
    double FR = 0, PR = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    public void init() {
        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeMotor.setDirection(DcMotor.Direction.REVERSE);
        PIDFCoefficients pidfCoefficientsR = new PIDFCoefficients(PR,0,0,FR);
        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsR);

        telemetry.addLine("Init  complete");
    }

    public void runFlywheel() {
        //get all our gamepad commands
        // set target velocity
        // update telemetry

        if (gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            }else {curTargetVelocity = highVelocity; }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.xWasPressed()){
            rightTuning = !rightTuning;
        }

        if(gamepad1.dpadLeftWasPressed()){
            FR += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            FR -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            PR += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            PR -= stepSizes[stepIndex];
        }

        //set new PIDF coefficients
        PIDFCoefficients pidfCoefficientsR = new PIDFCoefficients(PR,0,0,FR);
        OuttakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsR);

        //set velocity
        OuttakeMotor.setVelocity(curTargetVelocity);

        double curVelocity = OuttakeMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity","%.2f", curVelocity);
        telemetry.addData("Error","%.2f",error);
        telemetry.addLine("---------------------------");
        telemetry.addData("Right Motor is being tuned: ", "%b", rightTuning);
        telemetry.addData("Tuning PR", "%.4f (D-Pad U/D)", PR);
        telemetry.addData("Tuning FR", "%.4f (D-Pad L/R)", FR);
        telemetry.addData("Step Size", "%.4f (B Button)",stepSizes[stepIndex]);
    }

    @Override
    public void init(OpMode opMode) {

    }
}