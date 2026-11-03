package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import robotcore.Subsystem;

public class Intake extends Subsystem {
    private boolean transfer= false;
    private String status;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    @Override
    public void init(OpMode opMode) {
        intakeMotor = hardwareMap.get(DcMotor.class,"intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void runIntake() {
        if (gamepad1.aWasPressed()) {
            intakeMotor.setPower(1.0);
            status = "forward";
        }
        if (gamepad1.bWasPressed()) {
            intakeMotor.setPower(0.0);
            status = "off";
        }
        if (gamepad1.yWasPressed()) {
            intakeMotor.setPower(-1.0);
            status = "backward";
        }
        if (gamepad1.rightTriggerWasPressed()){
            transfer = !transfer;
            if (transfer) { transferMotor.setPower(1.0); } else {
                transferMotor.setPower(0.0);
            }
        }
        telemetry.addData("intake status:", status).addData("transfer:", transfer);
    }
}