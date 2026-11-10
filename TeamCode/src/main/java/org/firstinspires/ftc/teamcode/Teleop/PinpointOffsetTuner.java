package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auton.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subSystems.fieldCentricMecanum;

/**
 * Finds the Pinpoint pod offsets by spinning in place.
 *
 * Idea: with offsets set to 0, Pinpoint tracks the point where the pod lines cross (P),
 * not the robot center (C). When the robot spins about C, P swings in a circle, so
 * Pinpoint reports  pos(theta) = (R(theta) - I) * r,  where r = P - C in the robot frame.
 * We least-squares fit r over every sample, then:
 *     xOffset = r.left     (how far LEFT of center the forward/X pod is)
 *     yOffset = r.forward  (how far FORWARD of center the strafe/Y pod is)
 *
 * "Center" here = the point a mecanum drivetrain naturally spins around (drivetrain center).
 * Measure TURRET_DX/DY from that same point.
 *
 * Use: put the robot on tiles with room to spin, press START, keep hands off.
 * Spins TURNS times one way, then TURNS times the other way, then shows the result.
 */
@TeleOp(name = "Pinpoint Offset Tuner", group = "Test")
public class PinpointOffsetTuner extends LinearOpMode {

    static final double SPIN_POWER = 0.35;  // slow = less wheel slip = robot stays centered
    static final double TURNS = 3;          // per direction

    fieldCentricMecanum drive = new fieldCentricMecanum();

    @Override
    public void runOpMode() {
        drive.instantiateSubsystem(this);
        drive.init(this);
        GoBildaPinpointDriver odo = drive.odo;

        double oldX = odo.getXOffset(DistanceUnit.INCH);
        double oldY = odo.getYOffset(DistanceUnit.INCH);

        // Zero the offsets so Pinpoint reports the pod-crossing point P.
        odo.setOffsets(0, 0, DistanceUnit.INCH);
        odo.resetPosAndIMU();      // robot must be still for ~0.25 s
        sleep(500);

        telemetry.addData("Current offsets (in)", "x %.3f  y %.3f", oldX, oldY);
        telemetry.addLine("Clear space around the robot, then press START. Hands off.");
        telemetry.update();
        waitForStart();

        // Least-squares accumulators: r = sum(M^T p) / sum(2 - 2cos)
        double sumF = 0, sumL = 0, sumW = 0;
        // For a quality check (residual) we keep the raw sums needed to recompute later
        java.util.ArrayList<double[]> samples = new java.util.ArrayList<>();

        for (int dir : new int[]{1, -1}) {
            double turned = 0;
            odo.update();
            double lastH = odo.getHeading(AngleUnit.RADIANS);

            while (opModeIsActive() && turned < TURNS * 2 * Math.PI) {
                drive.drive(0, 0, dir * SPIN_POWER);
                odo.update();

                double h = odo.getHeading(AngleUnit.RADIANS);
                turned += Math.abs(AngleUnit.normalizeRadians(h - lastH));
                lastH = h;

                double px = odo.getPosX(DistanceUnit.INCH);
                double py = odo.getPosY(DistanceUnit.INCH);
                double c = Math.cos(h), s = Math.sin(h);
                sumF += (c - 1) * px + s * py;
                sumL += -s * px + (c - 1) * py;
                sumW += 2 - 2 * c;
                samples.add(new double[]{h, px, py});

                telemetry.addData("Direction", dir > 0 ? "first" : "second");
                telemetry.addData("Turns", "%.2f / %.0f", turned / (2 * Math.PI), TURNS);
                telemetry.addData("Raw pos (in)", "x %.2f  y %.2f", px, py);
                telemetry.update();
            }
            drive.drive(0, 0, 0);
            sleep(500);
        }
        drive.drive(0, 0, 0);

        if (sumW < 1e-6) {
            telemetry.addLine("No rotation recorded. Did the robot spin?");
            telemetry.update();
            while (opModeIsActive()) idle();
            return;
        }

        double rF = sumF / sumW;   // P is this far FORWARD of center
        double rL = sumL / sumW;   // P is this far LEFT of center

        // Residual: how well the circle fits. Big = robot wandered (wheel slip) -> rerun slower.
        double sq = 0;
        for (double[] smp : samples) {
            double c = Math.cos(smp[0]), s = Math.sin(smp[0]);
            double ex = smp[1] - ((c - 1) * rF - s * rL);
            double ey = smp[2] - (s * rF + (c - 1) * rL);
            sq += ex * ex + ey * ey;
        }
        double rms = Math.sqrt(sq / samples.size());

        double newX = rL, newY = rF;

        // Put the old offsets back so nothing else is affected until you edit the code.
        odo.setOffsets(oldX, oldY, DistanceUnit.INCH);

        while (opModeIsActive()) {
            telemetry.addLine("=== RESULT ===");
            telemetry.addData("NEW xOffset (in)", "%.3f   (X/forward pod, left +)", newX);
            telemetry.addData("NEW yOffset (in)", "%.3f   (Y/strafe pod, forward +)", newY);
            telemetry.addData("Old offsets (in)", "x %.3f  y %.3f", oldX, oldY);
            telemetry.addData("Fit error RMS (in)", "%.2f  %s", rms,
                    rms < 0.5 ? "(good)" : "(robot wandered - rerun, lower SPIN_POWER)");
            telemetry.addLine("Paste into fieldCentricMecanum:");
            telemetry.addLine(String.format("odo.setOffsets(%.3f, %.3f, DistanceUnit.INCH);", newX, newY));
            telemetry.update();
        }
    }
}
