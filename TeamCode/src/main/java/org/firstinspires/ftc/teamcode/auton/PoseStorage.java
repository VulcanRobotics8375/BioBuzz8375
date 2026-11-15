package org.firstinspires.ftc.teamcode.auton;

/**
 * Hands the robot's pose from auton to teleop (static = survives between OpModes, not app restarts).
 * Stored in the TELEOP field frame (the Panels frame MainOpMode uses).
 */
public class PoseStorage {
    public static boolean valid = false;
    public static double x, y, heading;     // inches, radians - teleop frame
    public static long savedAtMs = 0;

    /** Pedro (auton) frame -> teleop frame. Pedro uses the OLD teleop frame; teleop was rotated -90 deg:
     *  (x, y) -> (y, -x), heading - 90 deg. Same rule as the goals/start pose rotation. */
    public static void saveFromPedro(double px, double py, double ph) {
        x = py;
        y = -px;
        heading = ph - Math.PI / 2;
        valid = true;
        savedAtMs = System.currentTimeMillis();
    }

    /** True if auton saved a pose recently (so teleop right after auton uses it, a later practice run doesn't). */
    public static boolean fresh(double maxAgeS) {
        return valid && (System.currentTimeMillis() - savedAtMs) / 1000.0 < maxAgeS;
    }
}
