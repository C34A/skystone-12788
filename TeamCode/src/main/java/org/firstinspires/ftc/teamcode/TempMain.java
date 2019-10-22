
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="temp", group="Iterative Opmode")
@Disabled
public class TempMain extends OpMode {

    //uses positional IK control if true, uses angle based control if false.
    private boolean positionalControl = false;

    //degrees arm movement per encoder tick (incl gears between motors and arm)
    private final double LOWERARM_MOTOR_DEG_PER_TICK = 360.0 / 3892;
    private final double UPPERARM_MOTOR_DEG_PER_TICK = 360.0 / 537.6;
    //lengths of arm segments
    private final double LOWERARM_LENGTH = 30.0;
    private final double UPPERARM_LENGTH = 20.0;
    //maximum reach of arm
    private final double MAXREACH = LOWERARM_LENGTH + UPPERARM_LENGTH;
    private final double MINREACH = LOWERARM_LENGTH - UPPERARM_LENGTH;

    private final double INITIAL_LOWERANGLE = 40.0;
    private final double INITIAL_UPPERANGLE = -130.0;

    private enum ArmMode {
        STOPPED, RUNNING, CANNOT_REACH, INIT_UNSTOW, DUCK
    }

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lowerMotor = null;
    private DcMotor upperMotor1 = null;
    private DcMotor upperMotor2 = null;

    ArmMode armStatus = ArmMode.INIT_UNSTOW;

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

    private double targetX = 24.0;
    private double targetY = 10.0;

    private double lowerTargetAngle = 40.0;
    private double upperTargetAngle = -130.0;

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

        if(positionalControl) {
            targetX += gamepad1.left_stick_x;
            targetY += gamepad1.left_stick_y;
        } else {
            //change target by left stick
            lowerTargetAngle += gamepad1.left_stick_y;
            upperTargetAngle += gamepad1.right_stick_y;
        }

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
        } else if(armStatus == ArmMode.INIT_UNSTOW) {
            if(runtime.seconds() < 2.5) {
                lowerMotor.setTargetPosition((int) Math.round((90.0 - INITIAL_LOWERANGLE) / LOWERARM_MOTOR_DEG_PER_TICK));
            } else {
                armStatus = ArmMode.STOPPED;
            }
        } else if(armStatus == ArmMode.RUNNING) {
            //todo: inverse kinematics code here!
            //double angle1 = Math.acos(()/(-2*))
            if(positionalControl) {
                //the angle between a direct line to the target point from the base of the arm
                double D1 = Math.atan2(targetY, targetX);
                //the angle between the lower arm section and D1
                double D2 = lawOfCosines(targetDist, LOWERARM_LENGTH, UPPERARM_LENGTH);
                //the absolute angle of the lower arm section related to the horizontal axis of the robot
                double lowerSectionAngle = D1 + D2;
                //the internal angle between the arm segments
                double internalAngle = lawOfCosines(LOWERARM_LENGTH, UPPERARM_LENGTH, targetDist);
                //the absolute angle of the upper arm section related to the horizontal axis of the robot
                double upperAbsoluteAngle = lowerSectionAngle - (Math.PI - internalAngle);

                lowerTargetAngle = lowerSectionAngle;
                upperTargetAngle = upperAbsoluteAngle;
            }

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
        telemetry.addData("Target x,y", "x:(%.2f), y:(%.2f)", targetX, targetY);
        telemetry.addData("Target angles", "lower:(%.2f), y:(%.2f)", lowerTargetAngle, upperTargetAngle);
    }

    //law of cosines, solved to find angle C
    private double lawOfCosines(double a, double b, double c) {
        return Math.acos((a*a + b*b - c*c) / (2 * a * b));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
