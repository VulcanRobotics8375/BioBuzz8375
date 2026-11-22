package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Wait + Preload + Park")
public class AutonWaitPark extends auton {
    @Override
    protected Command routine (){return waitShootPark();}
}
