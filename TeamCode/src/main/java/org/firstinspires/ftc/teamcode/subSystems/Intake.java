package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

import robotcore.Subsystem;

public class Intake extends Subsystem {
    private boolean pos = false, moving =false; //true = up; false = down
    private double angle = 0;
    private double i=0;
    private DcMotor intakeMotor = null;
    private DcMotor transferMotor = null;
    private Servo savox, axon;
    boolean isOn = false, revIsOn = false;
    @Override
    public void init(OpMode opMode) {
        intakeMotor = hardwareMap.get(DcMotor.class,"intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        savox = hardwareMap.get(Servo.class, "savox");
        axon = hardwareMap.get(Servo.class, "axon");
        axon.setDirection(Servo.Direction.REVERSE);
        axon.setPosition((10 / 255) + (99 / 255));
        savox.setPosition(0.1 + (1 / 80) + (99 / 160));


    }
    public void moveIntake() {
        if (gamepad2.xWasPressed()) {
            ((PwmControl) axon).setPwmEnable();
            ((PwmControl) savox).setPwmEnable();
            moving = true;
            i=0;
            if (pos) {
                angle = 0;
            } else {
                angle = 99;
            }
            pos = !pos;
        }

        if (i>=60) {
            moving=false;
            ((PwmControl) axon).setPwmDisable();
            ((PwmControl) savox).setPwmDisable();
        } else {
            i+=1;
        }

        if (moving) {
            axon.setPosition((10 / 255) + (angle / 255));
            savox.setPosition(0.1 + (1 / 80) + angle / 160);
        }
        telemetry.addLine(String.format(Locale.US, "c: %.4f", angle));
        telemetry.addLine(String.format(Locale.US, "c: %.0f", i));

    }
    public void runIntake(){
        if (!pos && i>=60) {
            if (gamepad2.yWasPressed()) {
                if (isOn) {
                    isOn = false;
                    intakeMotor.setPower(0.0);
                    transferMotor.setPower(0.0);
                    telemetry.addLine("off");
                } else {
                    isOn = true;
                    revIsOn = false;
                    intakeMotor.setPower(1.0);
                    transferMotor.setPower(1.0);
                    telemetry.addLine("on");
                }
            }
            if (gamepad2.aWasPressed()) {
                if (revIsOn) {
                    revIsOn = false;
                    intakeMotor.setPower(0.0);
                    transferMotor.setPower(0.0);
                } else {
                    revIsOn = true;
                    isOn = false;
                    intakeMotor.setPower(0.0);
                    intakeMotor.setPower(-1.0);
                    transferMotor.setPower(0.0);
                    transferMotor.setPower(-1.0);
                }
            }
        } else {
            intakeMotor.setPower(0.0);
            transferMotor.setPower(0.0);
            isOn = false;
            revIsOn = false;
        }
    }
}