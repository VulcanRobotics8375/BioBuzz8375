package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;

/**
 * Pinpoint check with the robot's REAL settings (offsets/resolution/directions from fieldCentricMecanum).
 * Motors never run, so you can push the robot by hand.
 *
 * Raw ticks: A = zero. 48 in push should be ~90,800 ticks.
 *
 * FLIP TEST (checks pod offsets, works no matter where the robot pivots):
 *   1. Tape the robot's outline on the floor. B = reset position to 0,0,0.
 *   2. Pick the robot up / turn it 180 deg and set it back in the SAME outline.
 *   3. Read x, y. Good offsets -> both near 0. Screen shows corrected offsets.
 */
@TeleOp(name = "Pinpoint Raw Test", group = "Test")
public class PinpointRawTest extends OpMode {
    static final double PUSH_IN = 48.0;
    fieldCentricMecanum drive = new fieldCentricMecanum();
    GoBildaPinpointDriver odo;
    int x0, y0;

    @Override
    public void init() {
        drive.instantiateSubsystem(this);
        drive.init(this);          // same offsets/resolution/directions as MainOpMode
        odo = drive.odo;
    }

    @Override
    public void loop() {
        odo.update();
        int ex = odo.getEncoderX(), ey = odo.getEncoderY();
        if (gamepad1.a) { x0 = ex; y0 = ey; }
        if (gamepad1.b) odo.resetPosAndIMU();   // keep robot still ~0.5 s after pressing
        int dx = ex - x0, dy = ey - y0;
        double mm = PUSH_IN * 25.4;

        double px = odo.getPosX(DistanceUnit.INCH), py = odo.getPosY(DistanceUnit.INCH);
        double h = odo.getHeading(AngleUnit.DEGREES);
        double xOff = odo.getXOffset(DistanceUnit.INCH), yOff = odo.getYOffset(DistanceUnit.INCH);

        telemetry.addLine("--- raw ticks (A = zero) ---");
        telemetry.addData("X pod ticks", dx);
        telemetry.addData("Y pod ticks", dy);
        telemetry.addData("X ticks/mm (48in fwd)", "%.2f", Math.abs(dx) / mm);
        telemetry.addData("Y ticks/mm (48in left)", "%.2f", Math.abs(dy) / mm);

        telemetry.addLine("--- position (B = reset) ---");
        telemetry.addData("x (in)", "%.2f", px);
        telemetry.addData("y (in)", "%.2f", py);
        telemetry.addData("heading (deg)", "%.1f", h);

        telemetry.addLine("--- flip test: after 180 deg back in outline ---");
        telemetry.addData("current offsets", "x %.3f  y %.3f", xOff, yOff);
        if (Math.abs(Math.abs(h) - 180) < 10) {
            telemetry.addData("CORRECTED offsets", "x %.3f  y %.3f", xOff - py / 2, yOff - px / 2);
        } else {
            telemetry.addLine("(turn the robot ~180 deg to see corrected offsets)");
        }
        telemetry.addData("Status", odo.getDeviceStatus());
        telemetry.update();
    }
}
