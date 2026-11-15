package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.LED;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * LED test. Finds EVERY digital device in the robot config (whatever it's named) and lets you
 * set each one by hand, so you can see which name / channel / state actually lights what.
 * Works with BOTH config types: "Digital Device" and "LED". HIGH/LOW below = the real pin voltage
 * (the SDK's LED type is backwards: enable(true) = pin LOW - this test handles that for you).
 *
 * gamepad1:
 *   dpad up/down  pick a device
 *   A             toggle the picked device HIGH / LOW
 *   X             all LOW        Y  all HIGH
 *   B             BLINK all (1 Hz) on/off - easy to see if anything responds at all
 *
 * Things to check if nothing lights:
 *   - Config: each LED = "Digital Device" on the EXPANSION hub (not the Control Hub) if that's where it's plugged.
 *   - Port 0-1 has TWO channels: 0 and 1. REV devices often use the SECOND pin = channel 1.
 *   - Some LEDs light on LOW, not HIGH -> if the "LOW" one is lit, set IntakeSensor.LED_ON_STATE = false.
 *   - REV Digital LED Indicator (one red/green part, both channels): red/green/amber = the 4 HIGH/LOW combos.
 */
@TeleOp(name = "LED Test", group = "Test")
public class LedTest extends OpMode {
    private final List<String> names = new ArrayList<>();
    private final List<Object> leds = new ArrayList<>();   // DigitalChannel or LED
    private String allDevices = "";
    private final List<Boolean> states = new ArrayList<>();
    private int pick = 0;
    private boolean blink = false;
    private String error = "";

    @Override
    public void init() {
        for (Map.Entry<String, DigitalChannel> e : hardwareMap.digitalChannel.entrySet()) {
            try {
                DigitalChannel c = e.getValue();
                c.setMode(DigitalChannel.Mode.OUTPUT);
                add(e.getKey() + " (Digital)", c);
            } catch (Exception ex) {
                error += e.getKey() + ": " + ex.getMessage() + "  ";
            }
        }
        for (Map.Entry<String, LED> e : hardwareMap.led.entrySet()) add(e.getKey() + " (LED type)", e.getValue());
        for (HardwareDevice d : hardwareMap) {   // everything in the config, to spot wrong types / names
            allDevices += hardwareMap.getNamesOf(d) + "=" + d.getClass().getSimpleName() + "  ";
        }
        telemetry.addData("LEDs found", names.isEmpty() ? "NONE - check the robot config" : names.toString());
        telemetry.addData("whole config", allDevices);
        telemetry.update();
    }

    @Override
    public void loop() {
        int n = leds.size();
        if (n == 0) {
            telemetry.addLine("No Digital Device / LED in the config. Add them on the EXPANSION hub digital port (led_ball = 0, led_zone = 1), save + ACTIVATE the config.");
            telemetry.addData("whole config", allDevices);
            telemetry.update();
            return;
        }

        if (gamepad1.dpadUpWasPressed()) pick = (pick + n - 1) % n;
        if (gamepad1.dpadDownWasPressed()) pick = (pick + 1) % n;
        if (gamepad1.aWasPressed()) { blink = false; states.set(pick, !states.get(pick)); }
        if (gamepad1.xWasPressed()) { blink = false; for (int i = 0; i < n; i++) states.set(i, false); }
        if (gamepad1.yWasPressed()) { blink = false; for (int i = 0; i < n; i++) states.set(i, true); }
        if (gamepad1.bWasPressed()) blink = !blink;

        boolean blinkHigh = (System.currentTimeMillis() / 500) % 2 == 0;
        for (int i = 0; i < n; i++) {
            boolean s = blink ? blinkHigh : states.get(i);
            try {
                setPin(leds.get(i), s);
            } catch (Exception ex) {
                error = names.get(i) + ": " + ex.getMessage();
            }
        }

        telemetry.addLine("dpad = pick   A = toggle   X = all LOW   Y = all HIGH   B = blink");
        telemetry.addData("mode", blink ? ("BLINK  (now " + (blinkHigh ? "HIGH" : "LOW") + ")") : "manual");
        for (int i = 0; i < n; i++) {
            boolean s = blink ? blinkHigh : states.get(i);
            telemetry.addLine(String.format("%s %-22s  pin %s   [%s]", i == pick ? ">" : " ", names.get(i),
                    s ? "HIGH" : "LOW", ((HardwareDevice) leds.get(i)).getConnectionInfo()));
        }
        if (!error.isEmpty()) telemetry.addData("ERROR", error);
        telemetry.update();
    }

    @Override
    public void stop() {
        for (Object c : leds) {
            try { setPin(c, false); } catch (Exception ignored) { }
        }
    }

    private void add(String name, Object dev) {
        setPin(dev, false);
        names.add(name);
        leds.add(dev);
        states.add(false);
    }

    /** Set the real pin voltage, whatever the config type is. */
    static void setPin(Object dev, boolean high) {
        if (dev instanceof DigitalChannel) ((DigitalChannel) dev).setState(high);
        else if (dev instanceof LED) ((LED) dev).enable(!high);   // SDK LED: enable(true) = pin LOW
    }
}
