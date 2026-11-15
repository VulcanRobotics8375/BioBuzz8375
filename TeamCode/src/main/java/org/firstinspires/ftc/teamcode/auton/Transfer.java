package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Transfer{
    private final DcMotor TransferMotor;

    public Transfer(HardwareMap hardwareMap){
        TransferMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        TransferMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public Command off(){
        return instant(() -> TransferMotor.setPower(0.0));
    }
    public Command on(){
        return instant(() -> TransferMotor.setPower(1.0));
    }
}
