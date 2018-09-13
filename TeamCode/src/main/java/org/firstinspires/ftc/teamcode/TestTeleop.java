/*
We can win
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;


@TeleOp(name = "RRNicholasTest", group = "RR")  // @Autonomous(...) is the other common choice

public class TestTeleop extends OpMode {
DcMotor Left;
     DcMotor Right;
    boolean mode = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

       Left = hardwareMap.dcMotor.get("left");
        Right = hardwareMap.dcMotor.get("right");
        //Motor direction
        Left.setDirection(DcMotor.Direction.FORWARD);
        Right.setDirection(DcMotor.Direction.REVERSE);

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


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double speed = -0.5;
        if (gamepad1.x) {
            mode = true;
            }
            if (gamepad1.y) {
mode = false;
        }


        if(!mode) {
            float Rtrig = gamepad1.right_stick_y;
            float Ltrig = gamepad1.left_stick_y;

      /*  if (Rtrig >= 0.5) {
            Right.setPower(speed);
        } else if ( Ltrig >= 0.5) {
            Left.setPower(speed);
        } else if (Rtrig <= -0.5) {
            Right.setPower(speed * -1);
        } else if (Ltrig <= -0.5) {
            Left.setPower(speed * -1);
        } else {
Left.setPower(0);
            Right.setPower(0);
        } */


            Left.setPower(Ltrig);
            Right.setPower(Rtrig);
        } else {
            float Ch1 = -gamepad1.left_stick_x;
            float Ch3 = -gamepad1.left_stick_y;



            // tank drive
            // note that if y equal -1 then joystick is pushed all of the way forward.
            float right = Ch3 + Ch1;

            float left = Ch3 - Ch1;
        Left.setPower(left * -1);
            Right.setPower(right * -1);
        }
        }
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
