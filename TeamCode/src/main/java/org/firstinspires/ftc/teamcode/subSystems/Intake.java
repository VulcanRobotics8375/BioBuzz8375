package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import robotcore.Subsystem;

public class Intake extends Subsystem {
    private boolean transfer= false;
    private boolean stopped = false;
    private boolean intake= false;

    private String status;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private Servo transferServo;
    @Override
    public void init(OpMode opMode) {
        intakeMotor = hardwareMap.get(DcMotor.class,"intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo = hardwareMap.get(Servo.class,"transfer_servo");
    }
    public void runIntake() {
        if (gamepad2.xWasPressed()) {
            intake = !intake;
        }
        if (gamepad2.right_trigger_pressed && intake) {
            intakeMotor.setPower(0.4);
            status = "backward";
        } else if (intake) {
            intakeMotor.setPower(-0.8);
            status = "forward";
        } else {
            intakeMotor.setPower(0.0);
        }

        if (gamepad2.aWasPressed()) {
            transfer = !transfer;
            if (transfer) {
                transferMotor.setPower(-0.6);
            } else {
                transferMotor.setPower(0.0);
            }
        }
        if (gamepad2.leftBumperWasPressed()) {
            stopped = !stopped;
        }
        if (!stopped) {
            transferServo.setPosition(0.65);
        } else {
            transferServo.setPosition(0.4);
        }
    }
}
