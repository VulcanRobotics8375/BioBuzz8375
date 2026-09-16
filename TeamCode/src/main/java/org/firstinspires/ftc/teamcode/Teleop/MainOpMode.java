package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.Flywheel;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;
import org.firstinspires.ftc.teamcode.subSystems.Intake;


@TeleOp(name="MainOpMode")
    public class MainOpMode extends OpMode {
    private double axial, lateral, yaw;
    fieldCentricMecanum drive = new fieldCentricMecanum();
    Intake Intake = new Intake();
    Flywheel Flywheel = new Flywheel();


    public void init(){
        drive.instantiateSubsystem(this);
        Intake.instantiateSubsystem(this);
        Flywheel.instantiateSubsystem(this);
        drive.init(this);
        Intake.init(this);
        Flywheel.init(this);
    }

    @Override
    public void loop() {
        axial = gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;

        Intake.moveIntake();
        drive.fieldCentric(axial, lateral, yaw);
        Intake.runIntake();
        Flywheel.runFlywheel();
        telemetry.update();
    }
}
