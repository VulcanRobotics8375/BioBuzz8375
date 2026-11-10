package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

import robotcore.Subsystem;

public class flywheelPID extends Subsystem {
    public DcMotorEx OuttakeMotor = null;
    List<Integer> targetVelocity = List.of(1000,2000,3000,4000,5000,6000);
    int index = 0;
    double currentVelocity = 0;
    boolean flywheelOn = false;

    @Override
    public void init(OpMode opMode) {
        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        OuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }
}
