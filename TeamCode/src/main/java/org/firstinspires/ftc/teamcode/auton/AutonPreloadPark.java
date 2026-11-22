package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Safe fallback auton: shoot the preloads from the start pose, then park. Same settings as the full auton. */
@Autonomous(name = "Preload + Park", preselectTeleOp = "MainOpMode")
public class AutonPreloadPark extends auton {
    @Override
    protected Command routine() { return shootPark(); }
}

