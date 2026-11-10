package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subSystems.MotorOuttake;

@TeleOp (name = "bbController Test")
public class bbControllerTest extends OpMode {
    MotorOuttake Outtake = new MotorOuttake();
    public void init(){
        Outtake.init(this);
    }

    @Override
    public void loop(){

    }
}
