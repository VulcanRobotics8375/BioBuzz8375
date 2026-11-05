package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Flywheel;
import org.firstinspires.ftc.teamcode.subSystems.MiscMotor;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;
import org.firstinspires.ftc.teamcode.subSystems.Intake;


@TeleOp(name="RunMotor")
public class RunMotor extends OpMode {
    MiscMotor Motor = new MiscMotor();
    private Transfer transfer = new Transfer();
    public void init() {
        Motor.instantiateSubsystem(this);
        Motor.init(this);
        transfer.init(this);
    }

    @Override
    public void loop() {
        Motor.runIntake();
        transfer.runTransfer();
    }
}
