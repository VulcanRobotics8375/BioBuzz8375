package robotcore;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Subsystem {
    protected HardwareMap hardwareMap;

    protected Telemetry telemetry;

    protected Gamepad gamepad1;

    protected Gamepad gamepad2;

    /** false = skip telemetry this loop (building strings costs loop time). MainOpMode turns it on ~10x/s. */
    public static boolean showTelemetry = true;


    public void instantiateSubsystem(OpMode opMode){
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
    }


//    public void initialize(OpMode opMode) {
//        instantiateSubsystem(opMode);
//        init(opMode);
//    } //^^ some weird chat method to fix our abstract class issues
//    public abstract void init();

    /** Use these gamepads instead of the OpMode's (e.g. PanelsPads). Call AFTER init(). */
    public void useGamepads(Gamepad g1, Gamepad g2) {
        this.gamepad1 = g1;
        this.gamepad2 = g2;
    }

    public abstract void init(OpMode opMode);

    //public abstract void init(OpMode opMode);
}