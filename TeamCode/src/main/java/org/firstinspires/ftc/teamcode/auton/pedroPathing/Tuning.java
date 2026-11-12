package org.firstinspires.ftc.teamcode.auton.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

/** Pedro AutoTune entry points. Open http://192.168.43.1:10158 while connected to the robot. */
public class Tuning {
    @Tuner(name = "Mecanum Procedure")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Procedure")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Procedure")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                h -> new Mecanum(h, Constants.drivetrainConfig));
    }

    @Tuner(name = "Tests Procedure")
    public static Procedure tests() {
        return new Tests(
                h -> new Mecanum(h, Constants.drivetrainConfig),
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
