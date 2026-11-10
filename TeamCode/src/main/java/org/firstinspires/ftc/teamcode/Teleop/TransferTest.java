package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subSystems.Transfer;

import robotcore.Subsystem;
@TeleOp(name= "TransferTest")
public class TransferTest  extends OpMode {
private Servo TransferServo = null; //actually its the adaptive hood servo {
    public double angle;


    @Override
    public void init() {
            TransferServo = hardwareMap.get(Servo.class, "transfer_servo");
            angle = TransferServo.getPosition();
        }

    @Override
    public void loop() {
            telemetry.addData("ServoAngleOne", angle);
            if (gamepad1.rightBumperWasPressed()) {
                angle += 0.01;
            }
            if (gamepad1.leftBumperWasPressed()) {
                angle -= 0.01;
            }
            if (gamepad1.bWasPressed()) {
                telemetry.addData("ServoAngleTwo", angle);
            }
            TransferServo.setPosition(angle);
            telemetry.update();

    }
}




