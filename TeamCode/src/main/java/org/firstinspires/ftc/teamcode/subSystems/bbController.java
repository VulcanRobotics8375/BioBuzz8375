package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

import robotcore.Subsystem;

public class bbController extends Subsystem {
    public DcMotorEx OuttakeMotor = null;
    List <Integer>  targetVelocity = List.of(1000,2000,3000,4000,5000,6000);
    int index = 0;
    double currentVelocity = 0;
    boolean flywheelOn = false;

    @Override
    public void init(OpMode opMode) {
        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        OuttakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void runOuttake(){

        if(gamepad2.yWasPressed()){
            if(flywheelOn){
                OuttakeMotor.setPower(0.0);
                flywheelOn = false;
            }
            else{
                OuttakeMotor.setPower(1.0);
                flywheelOn = true;
            }
        }
        if(flywheelOn){
            currentVelocity = OuttakeMotor.getVelocity();
            if(currentVelocity < targetVelocity.get(index) - 50){
                OuttakeMotor.setPower(1.0);
            }
            else{
                OuttakeMotor.setPower(0.0);
            }
        }
        if(gamepad2.left_trigger_pressed){
            if(index < 0) {
                index = targetVelocity.size() - 1;
            }
            else{
                index --;
            }
        } else if (gamepad2.right_trigger_pressed) {
            if(index >= targetVelocity.size()){
                index = 0;
            }
            else {
                index++;
            }
        }
    }
}
