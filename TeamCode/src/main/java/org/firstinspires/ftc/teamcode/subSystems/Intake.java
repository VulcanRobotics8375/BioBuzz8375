package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import robotcore.Subsystem;

public class Intake extends Subsystem {
    private DcMotor intakeMotor = null;
    boolean isOn = false, revIsOn = false;
    @Override
    public void init(OpMode opMode) {
        intakeMotor = hardwareMap.get(DcMotor.class,"intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake(){
        if (gamepad2.yWasPressed()) {
            if (isOn) {
                isOn = false;
                intakeMotor.setPower(0.0);
                telemetry.addLine("off");
            } else {
                isOn = true;
                revIsOn = false;
                intakeMotor.setPower(1.0);
                telemetry.addLine("on");
            }
        }
        if (gamepad2.aWasPressed()) {
            if (revIsOn){
                revIsOn = false;
                intakeMotor.setPower(0.0);
            }
            else{
                revIsOn = true;
                isOn = false;
                intakeMotor.setPower(0.0);
                intakeMotor.setPower(-1.0);
            }
        }

    }
}