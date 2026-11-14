package org.firstinspires.ftc.teamcode.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Real controllers (Driver Hub) + the Panels gamepads (controller plugged into the laptop, Panels -> Gamepad tab)
 * MERGED: either one works. Buttons = pressed on either; sticks/triggers = Panels if it's moving, else the real one.
 *
 * Use g1 / g2 instead of gamepad1 / gamepad2. They are SEPARATE objects on purpose: the SDK rewrites gamepad1
 * every loop, so copying Panels input into it would make xWasPressed() fire every loop while held.
 * Call update() once at the START of every loop.
 */
@Configurable
public class PanelsPads {
    public static boolean ENABLED = true;   // false = only the real controllers

    public final Gamepad g1 = new Gamepad(), g2 = new Gamepad();

    public void update(OpMode op) {
        if (ENABLED) {
            g1.copy(PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(op.gamepad1));
            g2.copy(PanelsGamepad.INSTANCE.getSecondManager().asCombinedFTCGamepad(op.gamepad2));
        } else {
            g1.copy(op.gamepad1);
            g2.copy(op.gamepad2);
        }
    }
}
