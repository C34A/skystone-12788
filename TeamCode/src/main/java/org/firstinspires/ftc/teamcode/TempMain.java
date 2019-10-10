
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="temp", group="Iterative Opmode")
@Disabled
public class TempMain extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lowerMotor = null;
    private DcMotor upperMotor1 = null;
    private DcMotor upperMotor2 = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lowerMotor  = hardwareMap.get(DcMotor.class, "lowerArm");
        upperMotor1 = hardwareMap.get(DcMotor.class, "upperArm1");
        upperMotor2 = hardwareMap.get(DcMotor.class, "upperArm2");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        //reset motor encoders
        lowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set motors to built in PID positional control mode
        lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        lowerMotor.setTargetPosition(0);
        upperMotor1.setTargetPosition(0);
        upperMotor2.setTargetPosition(0);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor encoder ticks", "lower (%.2f), upper 1 (%.2f), upper 2 (%.2f)", lowerMotor.getCurrentPosition(), upperMotor1.getCurrentPosition(), upperMotor2.getCurrentPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
