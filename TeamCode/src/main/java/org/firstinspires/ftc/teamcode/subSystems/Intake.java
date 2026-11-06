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
        if (gamepad2.right_trigger_pressed) {
            intakeMotor.setPower(1.0);
            status = "forward";
        }
        if (gamepad2.bWasPressed()) {
            intakeMotor.setPower(0.0);
            status = "off";
        }
        if (gamepad2.xWasPressed()) {
            intakeMotor.setPower(-1.0);
            status = "backward";
        }
        if (gamepad2.aWasPressed()){
            transfer = !transfer;
            if (transfer) { transferMotor.setPower(-1.0); } else {
                transferMotor.setPower(0.0);
            }
        }
        telemetry.addData("intake status:", status).addData("transfer:", transfer);
    }
}