package org.firstinspires.ftc.teamcode.subSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import robotcore.Subsystem;
@Configurable
public class outtake extends Subsystem {
    boolean onoff = false;
    public static double HOOD_POS;
    private Servo HoodServo = null;
    public DcMotorEx OuttakeMotor;

    @Override
    public void init(OpMode opMode) {
        //instantiateSubsystem(opMode);

        OuttakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        OuttakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        HoodServo = hardwareMap.get(Servo.class, "hood_servo");

    }

    public void runOuttake() {
        if (gamepad2.yWasPressed()){
            onoff = !onoff;
            if (onoff) {
                OuttakeMotor.setPower(-1.0);
            } else {
                OuttakeMotor.setPower(0.0);
            }
        }
    }
}




