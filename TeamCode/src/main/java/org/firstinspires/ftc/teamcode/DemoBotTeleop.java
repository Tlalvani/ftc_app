/*
We can win
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */




@TeleOp(name="DemoBotTeleop", group="VV")  // @Autonomous(...) is the other common choice

public class DemoBotTeleop extends OpMode
{
    /* Declare OpMode members. */

    DcMotor Right;
    DcMotor Left;


    @Override
    public void init() {
        Left = hardwareMap.dcMotor.get("Left");
        Right = hardwareMap.dcMotor.get("Right");

        // Set to FORWARD if using AndyMark motors
         telemetry.addData("Status", "Initialized");


        Left.setDirection(DcMotor.Direction.FORWARD);

        // Set to REVERSE if using AndyMark motors
        Right.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


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
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        float Ch1 = gamepad1.right_stick_x;
        float Ch3 = -gamepad1.left_stick_y;
        float Ch4 = gamepad1.left_stick_x;


        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float right = Ch3 + Ch1;

        float left = Ch3 - Ch1;


        right = (float)scaleInput(right);
        left = (float)scaleInput(left);

        Left.setPower(left);
        Right.setPower(right);





        telemetry.addData("Left: ", Left.getPower());
        telemetry.addData("Right: ", Right.getPower());


        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
