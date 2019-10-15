
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

    //degrees arm movement per encoder tick (incl gears between motors and arm)
    private final double LOWERARM_MOTOR_DEG_PER_TICK = 360.0 / 3892;
    private final double UPPERARM_MOTOR_DEG_PER_TICK = 360.0 / 537.6;
    //lengths of arm segments
    private final double LOWERARM_LENGTH = 30.0;
    private final double UPPERARM_LENGTH = 20.0;
    //maximum reach of arm
    private final double MAXREACH = LOWERARM_LENGTH + UPPERARM_LENGTH;
    private final double MINREACH = LOWERARM_LENGTH - UPPERARM_LENGTH;

    private enum ArmMode {
        STOPPED, RUNNING, CANNOT_REACH;
    }
    /*
    private final int ARM_STOPPED = 0;
    private final int ARM_RUNNING = 1;
    private final int ARM_CANNOT_REACH = 2;
    */

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lowerMotor = null;
    private DcMotor upperMotor1 = null;
    private DcMotor upperMotor2 = null;

    ArmMode armStatus = ArmMode.STOPPED;

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

        //set motors to built in PIbD positional control mode
        lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private double targetX = 0.0;
    private double targetY = MINREACH;

    //todo: temp run to angle vars, replace w/ positional control
    private double lowerTargetAngle = 0.0;
    private double upperTargetAngle = 0.0;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //X button to run arm, Y button to stop arm.
        if(gamepad1.x) {
            armStatus = ArmMode.RUNNING;
        } else if (gamepad1.y) {
            armStatus =  ArmMode.STOPPED;
            //maybe todo: find a way like motor mode to fully stop?
        }

        //change target by left stick
        lowerTargetAngle += gamepad1.left_stick_y;
        upperTargetAngle += gamepad1.right_stick_y;

        double targetDist = Math.sqrt(targetX * targetX + targetY * targetY);
        if(targetDist > MAXREACH || targetDist < MINREACH) {
            //armStatus = ARM_CANNOT_REACH * armStatus; //if not running this will remain ARM_STOPPED, if running this will be ARM_CANNOT_REACH
            armStatus = (armStatus == ArmMode.RUNNING)? ArmMode.CANNOT_REACH : ArmMode.STOPPED; //if running change to cannot reach, otherwise stay stopped.
        }


        //no code for ARM_STOPPED, because the motor modes are set to stopped on button press.
        if(armStatus == ArmMode.STOPPED || armStatus == ArmMode.CANNOT_REACH) {
            //if stopped, do not move. NOTE: maybe i should use the motor stopped mode here? not sure if it is bad to do that in loop() though...
            lowerMotor.setTargetPosition(lowerMotor.getCurrentPosition());
            upperMotor1.setTargetPosition(upperMotor1.getCurrentPosition());
            upperMotor2.setTargetPosition(upperMotor2.getCurrentPosition());
        } else if(armStatus == ArmMode.RUNNING) {
            //todo: inverse kinematics code here!
            //double angle1 = Math.acos(()/(-2*))


            //run to angle
            int lowerTargetTicks = (int) Math.round(lowerTargetAngle / LOWERARM_MOTOR_DEG_PER_TICK);
            int upperTargetTicks = (int) Math.round(upperTargetAngle / UPPERARM_MOTOR_DEG_PER_TICK);

            lowerMotor.setTargetPosition(lowerTargetTicks);
            upperMotor1.setTargetPosition(upperTargetTicks);
            upperMotor2.setTargetPosition(upperTargetTicks);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor encoder ticks", "lower (%.2f), upper 1 (%.2f), upper 2 (%.2f)", lowerMotor.getCurrentPosition(), upperMotor1.getCurrentPosition(), upperMotor2.getCurrentPosition());
        //show target X,Y
        telemetry.addData("Target x,y", "x:(%.2f), y:(%.2f)", targetX, targetY);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
