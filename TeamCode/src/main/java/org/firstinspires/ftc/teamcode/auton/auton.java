package org.firstinspires.ftc.teamcode.auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auton.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subSystems.turret;

/**
 * Full auton: shoot preloads from the start, intake at the garden, shoot again, flower, park.
 * "Preload + Park" (AutonPreloadPark) runs only the short, safe version.
 *
 * Shots: each spot has its own rpm + hood below (live in Panels -> auton).
 * shoot() waits for the flywheel to be READY (max SHOT_TIMEOUT_MS) instead of a fixed wait.
 * The turret is LOCKED straight forward the whole auton: the robot's heading aims the shot.
 * At the end the pose is saved for teleop (PoseStorage) so the turret aims right after auton.
 */
@Configurable
@Autonomous(name = "4 balls + park auton")
public class auton extends OpMode {
    // ---- shot settings per spot (tune these) ----
    public static double SHOT1_RPM = 2090, SHOT1_HOOD = 0.11;   // from the start pose
    public static double SHOT2_RPM = 2090, SHOT2_HOOD = 0.11;   // at controlPoseTwo
    public static double SHOT1_FEED_MS = 1000, SHOT2_FEED_MS = 1500;
    public static double SHOT_TIMEOUT_MS = 4500;                 // shoot anyway if not READY by then

    turret turret = new turret();

    protected Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    protected final Pose startPose = p.of(15.5, 63, 270);
    private final Pose controlPoseThree = p.of(30, 63, 0);      // bend of the garden curve
    private final Pose gardenPose = p.of(48, 63, 0);
    private final Pose controlPose = p.of(38, 10, 45);          // bend of the curves through the middle
    private final Pose controlPoseTwo = p.of(15.5, -53, 90);    // second shooting spot
    private final Pose flowerReadyPose = p.of(24, -40, 270);
    private final Pose flowerPose = p.of(24, -60, 0);
    private final Pose park = p.of(60, -35, 180);
    autonIntake intake;
    protected Flywheel flywheel;

    // (score() / returnPose() removed: start and controlPoseOne were the SAME point = zero-length paths.
    //  We shoot straight from the start pose instead.)
    private Path garden() {
        return curve(startPose, controlPoseThree, gardenPose).linear(startPose, gardenPose);
    }
    private Path gardenShoot() {
        return curve(gardenPose, controlPose, controlPoseTwo).linear(gardenPose, controlPoseTwo);
    }
    private Path gardenFlower() {
        return line(controlPoseTwo, flowerReadyPose).linear(controlPoseTwo, flowerReadyPose);
    }
    private Path flower() {
        return line(flowerReadyPose, flowerPose).linear(flowerReadyPose, flowerPose);
    }
    private Path park() {
        return line(flowerPose, park).linear(flowerPose, park);
    }
    /** Start -> park, bending through controlPose (was: started at controlPose, where the robot never is). */
    private Path easyPark() {
        return curve(startPose, controlPose, park).linear(startPose, park);
    }

    @Override
    public void init() {
        Scheduler.reset();

        turret.instantiateSubsystem(this);
        turret.init(this);              // turret must point straight forward at init
        turret.lockMode = true;         // hold it there all auton
        turret.lockAngleDeg = 0;

        intake = new autonIntake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    /** Short + safe: shoot the preloads, park. */
    protected Command shootPark() {
        return sequential(
                flywheel.setShot(SHOT1_RPM, SHOT1_HOOD),
                flywheel.flywheelOn(),
                flywheel.intakeOn(),
                flywheel.transferOn(),
                flywheel.shoot(SHOT1_FEED_MS, SHOT_TIMEOUT_MS),
                flywheel.allOff(),
                follow(follower, easyPark())
        );
    }

    /** Full routine. The flywheel stays ON between the two shots (no 2 s spin-up again). */
    protected Command fullAuton() {
        return sequential(
                // ---- shot 1: preloads from the start ----
                flywheel.setShot(SHOT1_RPM, SHOT1_HOOD),
                flywheel.flywheelOn(),
                flywheel.intakeOn(),
                flywheel.transferOn(),
                flywheel.shoot(SHOT1_FEED_MS, SHOT_TIMEOUT_MS),
                // ---- garden: intake ----
                follow(follower, garden()),
                waitMs(2000),                                   // intaking
                // ---- shot 2 ----
                flywheel.setShot(SHOT2_RPM, SHOT2_HOOD),
                follow(follower, gardenShoot()),
                flywheel.shoot(SHOT2_FEED_MS, SHOT_TIMEOUT_MS),
                // ---- flower ----
                flywheel.flywheelOff(),
                follow(follower, gardenFlower()),
                flywheel.flowerOn(),
                waitMs(400),
                follow(follower, flower()),
                waitMs(3000),
                // ---- park ----
                follow(follower, park()),
                flywheel.allOff()
        );
    }

    protected Command routine() { return fullAuton(); }

    @Override
    public void start() {
        schedule(routine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        flywheel.periodic();
        turret.tUpdate();                // lockMode: just holds the turret at 0 deg

        Pose pose = follower.pose();
        PoseStorage.saveFromPedro(pose.x(), pose.y(), pose.heading());   // every loop: auton can end any time

        telemetry.addData("pose (x, y, h deg)", "%.1f, %.1f, %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("path", "mode %s  busy %s  to end %.1f in",
                follower.mode(), follower.isBusy(), follower.distanceToEndpoint());
        telemetry.addData("flywheel", "%s  rpm %.0f / %.0f  ready %s  feeding %s",
                flywheel.flywheelOn ? "ON" : "off", flywheel.rpm, Flywheel.TARGET_RPM, flywheel.ready, flywheel.feeding);
        telemetry.addData("turret hold err (deg)", "%.1f%s", Math.toDegrees(turret.pError),
                turret.faulted ? "  FAULT: " + turret.faultReason : "");
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.flywheelOn = false;
        flywheel.periodic();
    }
}
