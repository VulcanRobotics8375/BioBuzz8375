package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class autonIntake {
    private final DcMotor intakeMotor;
    public autonIntake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class,"intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public Command off(){
        return instant(() -> intakeMotor.setPower(0.0));
    }
    public Command on() {
        return instant (() -> intakeMotor.setPower(0.4));
    }
}
