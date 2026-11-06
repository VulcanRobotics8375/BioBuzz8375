package org.firstinspires.ftc.teamcode.subSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import robotcore.Subsystem;
@Config
public class outtake extends Subsystem {
    boolean onoff = false;
    public DcMotorEx OuttakeMotor;

    @Override
    public void init(OpMode opMode) {
        //instantiateSubsystem(opMode);

        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void runOuttake() {
        if (gamepad2.yWasPressed()){
            onoff = !onoff;
            if (onoff) {
                OuttakeMotor.setPower(-1.0);
            } else {
                OuttakeMotor.setPower(0.0);
            }
        }
    }
}




