package org.firstinspires.ftc.teamcode.auton.pedroPathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

/** Pedro AutoTune entry points. Open http://192.168.43.1:10158 while connected to the robot. */
public class Tuning {
    @Tuner(name = "Pinpoint Procedure")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
}
