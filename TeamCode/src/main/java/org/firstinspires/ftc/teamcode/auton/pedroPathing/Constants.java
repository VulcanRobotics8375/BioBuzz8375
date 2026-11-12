package org.firstinspires.ftc.teamcode.auton.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;

import java.util.OptionalDouble;

/**
 * Pedro Pathing 3 setup. Tune in this order at http://192.168.43.1:10158 (phone/laptop on the robot wifi):
 *   1. Mecanum Procedure   -> paste its Java output over drivetrainConfig
 *   2. Pinpoint Procedure  -> paste over localizerConfig
 *   3. Foresight Procedure -> paste over foresightConfig   (needs 1 + 2 correct first!)
 *   4. Tests Procedure     -> line / curve / hold tests
 * Re-deploy after each paste.
 */
@Configurable

public class Constants {
    public static double headingkP1= 1.8;
    public static double headingkP2= 5; // 0   (not used: single heading kP like the tuner gives)

    public static double forwardPrimarykP = 0.5006021564916031;
    public static double forwardsSecondarykP = 0.18495912048767038; //0.05
    public static double strafePrimarykP = 0.7897260354530434;
    public static double strafeSecondarykP = 0.29178266823159366;
    public static double natForwardDecelerationValue = 40.0;   // tuner measured 49.69; lower = starts braking earlier
    public static double brakeAgreesionValue = 0.85;           // Foresight default 1.0
    // ^ these two are read ONCE when the app starts (Pedro takes a number, not a lambda) -> change + redeploy
    // The controllers below read these through () -> lambdas, so Panels edits apply LIVE.
    // (Controller.proportional(number) copies the number once at app start - Panels changes did nothing.)


    // Starting values = what teleop (fieldCentricMecanum) uses right now. The Mecanum Procedure verifies them.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.backLeftName.set("leftBack");
        c.frontRightName.set("rightFront");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    // Starting values = teleop's odo setup (offsets -3.278 / -2.778 in, 74.5027 ticks/mm, X reversed).
    // The Pinpoint Procedure re-measures all of this.
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odo");
        c.xPodOffset.set(-3.278);
        c.yPodOffset.set(-2.778);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.encoderResolutionUnit.set(DistanceUnit.MM);
        c.ticksPerUnit.set(OptionalDouble.of(74.5027025034));
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    });

    // PLACEHOLDERS copied from the Pedro docs example robot - NOT tuned for ours.
    // Overwrite the whole block with the Foresight Procedure's Java output.
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.4642659070163277);
//                Controller secondaryTranslationalForward = Controller.proportional(0.17153384722902382);
//                Controller primaryTranslationalLateral = Controller.proportional(0.9217559098739423);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.34056417892689683);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.015085278600536718));
//                c.brake.set(Controller.proportionalFeedforward(0.01282248681045621));
//
//                c.headingFeedback.set(Controller.proportional(9.926111938444553));//9.926111938444553
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.057156723928636445, 0.006465756757639612));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.05907889613602843, 0.08249188551375836));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002036490968303793, 0.0015102767562263537));
//
//                c.maxAchievableForwardVelocity.set(69.65477496041756);
//                c.maxAchievableStrafeVelocity.set(50.085917864318645);
//                c.naturalForwardDeceleration.set(51.4208725517375);
//                c.naturalStrafeDeceleration.set(75.35327534714084);
//            }
//    );

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(() -> forwardPrimarykP);//0.5794286965307958
//                Controller secondaryTranslationalForward = Controller.proportional(() -> forwardsSecondarykP);//0.21408342074821024
//                Controller primaryTranslationalLateral = Controller.proportional(() -> strafePrimarykP);
//                Controller secondaryTranslationalLateral = Controller.proportional(() -> strafeSecondarykP);
//                Controller primaryHeadingkP = Controller.proportional(() -> headingkP1);
//                Controller secondaryHeadingkP = Controller.proportional(() -> headingkP2);
//
//
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.014263324769551267));
//                c.brake.set(Controller.proportionalFeedforward(0.012123826054118577));
//
////                c.headingFeedback.set(Controller.proportional(headingkP1)); //8.126663660629694
//                c.headingFeedback.set(Controller.piecewise(secondaryHeadingkP).put(Math.toRadians(5), primaryHeadingkP));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06236632309183649, 0.005800316859457542));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.15031971362258606, 0.08020085686334907));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(3.618088624072881E-4, 0.001577737594715235));
//
//                c.maxAchievableForwardVelocity.set(77.51015804984054);
//                c.maxAchievableStrafeVelocity.set(55.066101866255835);
//                c.naturalForwardDeceleration.set(54.69554455624963);
//                c.naturalStrafeDeceleration.set(90.0010882774667);
//            }
//    );

    // ---- run 3 (raw tuner output) ----
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.5552791875898476);
//                Controller secondaryTranslationalForward = Controller.proportional(0.20516082248129328);
//                Controller primaryTranslationalLateral = Controller.proportional(1.7593679073822934);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.650039430601455);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.0159971289557347));
//                c.brake.set(Controller.proportionalFeedforward(0.013597559612374494));
//
//                c.headingFeedback.set(Controller.proportional(11.62366083627443));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06074405670449834, 0.006661590941537631));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.09021499418432215, 0.05123832785908969));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001446450906830264, 0.002658870600879968));
//
//                c.maxAchievableForwardVelocity.set(67.26529549488745);
//                c.maxAchievableStrafeVelocity.set(50.63055039660014);
//                c.naturalForwardDeceleration.set(55.30383007404673);
//                c.naturalStrafeDeceleration.set(79.45093814692297);
//            }
//    );

    // ===== RUN 4 (latest Foresight tuner output) =====
    // The 5 gains read the static fields at the top -> tune live in Panels.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(() -> forwardPrimarykP);
                Controller secondaryTranslationalForward = Controller.proportional(() -> forwardsSecondarykP);
                Controller primaryTranslationalLateral = Controller.proportional(() -> strafePrimarykP);
                Controller secondaryTranslationalLateral = Controller.proportional(() -> strafeSecondarykP);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01481359723763826));
                c.brake.set(Controller.proportionalFeedforward(0.01259155765199252));

                c.headingFeedback.set(Controller.proportional(() -> headingkP1));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0660333992778969, 0.006067100659186965));

                // forward = run 4. strafe = average of runs 1-3 (run 4's strafe linear was NEGATIVE:
                // -0.0090 linear / 0.003676 quadratic -> predicted a negative stop distance below ~2.5 in/s)
                c.linearBrakeCoefficients.set(Matrix.diag(0.1129855126281048, 0.071310));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001341291605111977, 0.0019156));

                c.maxAchievableForwardVelocity.set(74.23167862709019);
                c.maxAchievableStrafeVelocity.set(52.73426486293636);
                c.naturalForwardDeceleration.set(natForwardDecelerationValue);
                c.naturalStrafeDeceleration.set(87.93054687598296);
                c.brakeAggression.set(brakeAgreesionValue);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
