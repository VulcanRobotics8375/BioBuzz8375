package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.auton.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "4 balls + park auton")
public class auton extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(13, 63, 270);
    private final Pose scorePose = p.of(30, 63, 270);
    private final Pose controlPose = p.of(39, 30, 215);
    private final Pose flowerPose = p.of(63, 23, 180);
    private final Pose flowerControlPose = p.of(50, 12, 0);
    private final Pose park = p.of(63, -36, 120);

    Intake intake = new Intake(hardwareMap);
    Flywheel flywheel = new Flywheel(hardwareMap);
    private Path score(){
        return line(startPose, scorePose).linear(startPose, scorePose);
    }
    private Path flower(){
        return curve(scorePose, controlPose,flowerPose).linear(scorePose, flowerPose);
    }
    private Path flowerScore(){
        return curve(flowerPose, flowerControlPose,scorePose).linear(flowerPose, scorePose);
    }
    private Path park(){
        return line(scorePose, park).linear(scorePose, park);
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    private Command auton(){
        return sequential(
                follow(follower, score()),
                //shoot
                flywheel.on(),
                intake.on(),
                //pause to allow time to shoot
                waitMs(1000),
                follow(follower, flower()),
                //intake
                flywheel.off(),
                waitMs(1000),
                follow(follower, flowerScore()),
                //shoot
                flywheel.on(),
                waitMs(1000),
                follow(follower, park())
        );
    }
    @Override
    public void start(){
        schedule(auton());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
