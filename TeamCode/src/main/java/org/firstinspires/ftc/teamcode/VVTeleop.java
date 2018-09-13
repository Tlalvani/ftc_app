/*
We can win
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="ThisisVVTeleop", group="VV")  // @Autonomous(...) is the other common choice

public class VVTeleop extends OpMode
{
    /* Declare OpMode members. */

double stopperposition;
    DcMotor LF;
     DcMotor LB;
    DcMotor RB;
     DcMotor RF;
    DcMotor RW;
    DcMotor LW;
    DcMotor Intake;
    DcMotor LED;
    Servo Stopper;
    Servo RedHitter;
    Servo BlueHitter;
    Servo RedAuto;
    Servo BlueAuto;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        LB = hardwareMap.dcMotor.get("LB");
        LF = hardwareMap.dcMotor.get("LF");
        RB = hardwareMap.dcMotor.get("RB");
        RF = hardwareMap.dcMotor.get("RF");
        LW = hardwareMap.dcMotor.get("LW");
        RW = hardwareMap.dcMotor.get("RW");
        Intake = hardwareMap.dcMotor.get("Intake");
        LED = hardwareMap.dcMotor.get("LED");
        Stopper = hardwareMap.servo.get("Stopper");
        RedHitter = hardwareMap.servo.get("RedHitter");
        BlueHitter = hardwareMap.servo.get("BlueHitter");
        RedAuto = hardwareMap.servo.get("RedAuto");
        BlueAuto = hardwareMap.servo.get("BlueAuto");
        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        LW.setDirection(DcMotor.Direction.FORWARD);
        RW.setDirection(DcMotor.Direction.REVERSE);

        // Set to FORWARD if using AndyMark motors`
         telemetry.addData("Status", "Initialized");
        LW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



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
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        float Ch1 = gamepad1.right_stick_x + gamepad2.right_stick_x/2;
        float Ch3 = -gamepad1.left_stick_y + -gamepad2.left_stick_y/2;
        float Ch4 = gamepad1.left_stick_x;


        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float rightfront = Ch3 - Ch1 - Ch4;
        float rightback = Ch3 - Ch1 + Ch4;
        float leftfront = Ch3 + Ch1 + Ch4;
        float leftback = Ch3 + Ch1 - Ch4;
        float intake = -gamepad1.right_trigger + +gamepad1.left_trigger;






        // clip the right/left values so that the values never exceed +/- 1
        rightback = Range.clip(rightback, -1, 1);
        leftback = Range.clip(leftback, -1, 1);
        rightfront = Range.clip(rightfront, -1, 1);
        leftfront = Range.clip(leftfront, -1, 1);
        intake = Range.clip(intake, -1, 1);
        stopperposition = Range.clip(stopperposition, 0, 1);





        {
            if (gamepad1.right_bumper) {

                stopperposition = .7;
            } else {

                stopperposition = .45;
            }
        }
        {if (gamepad1.a) {

            LW.setPower(.5);
            RW.setPower(.5);
        }

        else if (gamepad1.x) {

            LW.setPower(.75);
            RW.setPower(.75);
        }
            else if (gamepad1.b) {

            LW.setPower(1);
            RW.setPower(1);
        }

        else if (gamepad1.y) {

            LW.setPower(.25);
            RW.setPower(.25);
        }

        else if (gamepad1.dpad_down) {

            LW.setPower(-1);
            RW.setPower(-1);
        }




          else  if (gamepad2.a) {

                LW.setPower(.5);
                RW.setPower(.5);
            }

            else if (gamepad2.x) {

                LW.setPower(.75);
                RW.setPower(.75);
            }
            else if (gamepad2.b) {

                LW.setPower(1);
                RW.setPower(1);
            }

            else if (gamepad2.y) {

                LW.setPower(.25);
                RW.setPower(.25);
            }

            else if (gamepad2.dpad_down) {

                LW.setPower(-1);
                RW.setPower(-1);
            }

        else  {

            LW.setPower(0);
            RW.setPower(0);
        }

        }


        { if (gamepad1.dpad_left){
            BlueHitter.setPosition(1);
            RedHitter.setPosition(-1);
        }

        else if (gamepad1.dpad_right){
            BlueHitter.setPosition(-1);
            RedHitter.setPosition(1);
        } }

        if(gamepad1.left_bumper){
            RedAuto.setPosition(.1);
            BlueAuto.setPosition(.55);
        }

        else
        {
            RedAuto.setPosition(.6);
            BlueAuto.setPosition(0);
        }
        {
            if (gamepad2.right_bumper) {
LED.setPower(1);
            }
            else if (gamepad2.left_bumper) {
                LED.setPower(0);
            }
        }






        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightfront = (float)scaleInput(rightfront);
        leftfront =  (float)scaleInput(leftfront);
        rightback = (float)scaleInput(rightback);
        leftback =  (float)scaleInput(leftback);
        intake =  (float)scaleInput(intake);

        // write the values to the motors
        RF.setPower(rightfront);
        LF.setPower(leftfront);
        RB.setPower(rightback);
        LB.setPower(leftback);
        Intake.setPower(intake);

        Stopper.setPosition(stopperposition);
        //telemetry.addData("LF: ", LF.getCurrentPosition());
        //telemetry.addData("RF: ", RF.getCurrentPosition());
        //telemetry.addData("RB: ", RB.getCurrentPosition());
        //telemetry.addData("LB: ", LB.getCurrentPosition());



        telemetry.addData("LB: ", LB.getPower());
        telemetry.addData("RB: ", RB.getPower());
        telemetry.addData("LF: ", LF.getPower());
        telemetry.addData("RF: ", RF.getPower());
        telemetry.addData("Intake: ", Intake.getPower());
        telemetry.addData("RW: ", RW.getPower());
        telemetry.addData("LW: ", LW.getPower());

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
