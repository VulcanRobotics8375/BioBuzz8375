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

import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.MotorOuttake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "4 balls + park auton")
public class auton extends OpMode {

    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(15.5, 63, 270);
    private final Pose scorePose = p.of(30, 63, 270);
    private final Pose controlPose = p.of(39, 30, 315);
    private final Pose flowerPose = p.of(55, 28, 0);
    private final Pose flowerControlPose = p.of(50, 12, 0);
    private final Pose park = p.of(63, -36, 120);
    autonIntake intake;
    Flywheel flywheel;

    private Path score(){
        return line(startPose, scorePose).linear(startPose, scorePose);
    }
    private Path flower(){
        return curve(startPose, controlPose,flowerPose).linear(startPose, flowerPose);
    }
    private Path flowerScore(){
        return curve(flowerPose, flowerControlPose,startPose).linear(flowerPose, startPose);
    }
    private Path park(){
        return line(startPose, park).linear(startPose, park);
    }

    @Override
    public void init() {
        Scheduler.reset();

        intake = new autonIntake(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        //transfer = new Transfer(hardwareMap);


        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    private Command auton(){
        return sequential(
                //(follower, score()),
                //shoot
                flywheel.toggleFlywheel(),
                waitMs(10000),
                flywheel.toggleIntake(),
                flywheel.toggleTransfer(),
                flywheel.toggleFeeding(),
                //pause to allow time to shoot
                waitMs(10000),
                follow(follower, flower()),
                //intake
                flywheel.toggleFlywheel(),
                //flywheel.toggleTransfer(),
                waitMs(3000),
                follow(follower, flowerScore()),
                //shoot
                flywheel.toggleFlywheel(),
                //flywheel.toggleTransfer(),
                waitMs(4000),
                //move robot to parking spot
                follow(follower, park()),
                //stop flywheel, intake, and transfer
                flywheel.toggleTransfer(),
                flywheel.toggleIntake(),
                flywheel.toggleFlywheel()
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

        flywheel.periodic();
    }
}
