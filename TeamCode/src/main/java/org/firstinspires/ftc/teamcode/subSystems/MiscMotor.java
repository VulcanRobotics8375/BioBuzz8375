package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import robotcore.Subsystem;

public class MiscMotor extends Subsystem {
    private DcMotor Motor = null;
    boolean isOn = false, revIsOn = false;
    @Override
    public void init(OpMode opMode) {
        Motor = hardwareMap.get(DcMotor.class,"motor");
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runIntake(){
        if (gamepad2.yWasPressed()) {
            if (isOn) {
                isOn = false;
                Motor.setPower(0.0);
                telemetry.addLine("off");
            } else {
                isOn = true;
                revIsOn = false;
                Motor.setPower(1.0);
                telemetry.addLine("on");
            }
        }

    }
}