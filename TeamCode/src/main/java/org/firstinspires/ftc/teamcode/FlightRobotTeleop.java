/*
We can win
*/
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "CarerTestTeleop", group = "RR")  // @Autonomous(...) is the other common choice

public class CarterTestTeleop extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
public PropellerLeftFront("PLF"), PropellerRightFront("PRF"), PropellerLeftBack("PLB"), PropellerRightBack("PRB"),
RightWheel("RW"), LeftWheel("LW"), Dropper("DR"), Intake("IN")
    /* Declare OpMode members. */
    DcMotor PLF;
    DcMotor PLB;
    DcMotor PRB;
    DcMotor PRF;
    DcMotor RW;
    DcMotor LW;
    DcMotor DR;
    DcMotor IN;

    //Variables
    double brake = 0;
    double hover = 0.5;
    double rise = 1;
    double driveforward = 1;
    double Intake = 1;
    double Outtake = -1;
    double Drop = 1
    //HardwareMapping

    @Override
    public void init()
    {
        PLB = hardwareMap.dcMotor.get("PLB");
        PLF = hardwareMap.dcMotor.get("PLF");
        PRB = hardwareMap.dcMotor.get("PRB");
        PRF = hardwareMap.dcMotor.get("PRF");
        RW = hardwareMap.dcMotor.get("RW");
        LW = hardewareMap.dcMotor.get("LW");
        DR = hardwareMap.dcMotor.get("DR");
        IN = hardwareMap.dcMotor.get("IN";
        PLB.setDirection(DcMotor.Direction.FORWARD);
        PLF.setDirection(DcMotor.Direction.REVERSE);
        PRB.setDirection(DcMotor.Direction.REVERSE);
        PRF.setDirection(DcMotor.Direction.FORWARD);
        RW.setDirection(DcMotor.Direction.FORWARD);
        LW.setDirection(DcMotor.Direction.FORWARD);
        DR.setDirection(DcMotor.Direction.FORWARD);
        IN.setDirection(DcMotor.Direction.FORWARD);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    //variables

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    //Gamepad Programming

    @Override
    public void loop() {
            //Gamepad1 Starts
            //Rise
            if(gamepad1.dpad_up)
            {
                PRF.setPower(rise);
                PLB.setPower(rise);
                PRB.setPower(rise);
                PLF.setPower(rise);
            }
            //Forwards
            else if(gamepad1.left_stick_y > 0.5)
            {
                PRF.setPower(hover);
                PLB.setPower(rise);
                PRB.setPower(rise);
                PLF.setPower(hover);
            }
            //Backwards
            else if(gamepad1.left_stick_y < -0.5)
            {
                PRF.setPower(rise);
                PLB.setPower(hover);
                PRB.setPower(hover);
                PLF.setPower(rise);
            }
            //Right
            else if(gamepad1.left_stick_x > 0.5)
            {
                PRF.setPower(hover);
                PLB.setPower(rise);
                PRB.setPower(hover);
                PLF.setPower(rise);
            }
            //Left
            else if(gamepad1.left_stick_x < -0.5)
            {
                PRF.setPower(rise);
                PLB.setPower(hover);
                PRB.setPower(rise);
                PLF.setPower(hover);
            }
            //If None are Active
            else {
                PRF.setPower(hover);
                PLB.setPower(hover);
                PRB.setPower(hover);
                PLF.setPower(hover);
            }
            //Gamepad2 Starts(OnGround Driving)
            //Forward
            if(gamepad2.left_stick_y > 0.5)
            {
                RW.setPower(driveforward)
                LW.setPower(driveforward)
            }
            //Backward
            else if(gamepad2.left_stick_y < -0.5)
            {
                RW.setPower(-driveforward)
                LW.setPower(-driveforward)
            }
            //Turn Right
            else if(gamepad2.right_stick_x > 0.5)
            {
                RW.setPower(-driveforward)
                LW.serPower(driveforward)
            }
            //Turn Left
            else if(gamepad2.rightstick_x < -0.5)
            {
                RW.setPower(driveforward)
                LW.setPower(-driveforwsrd)
            }
            //Turn on Intake
            else if(gamepad2.right_trigger)
            {
                IN.setPower(Intake)
            }
            //Turn on Outtake
            else if(gamepad2.left_trigger)
            {
                IN.setPower(Outtake)
            }
            //Raise Lift
            else if(gamepad2.dpad_up)
            {
                DR.setPower(-Drop)
            }
            //Lower Lift
            else if(gamepad2.dpad_down)
            {
             DR.setPower(Drop)
            }
            //When no Buttons are Pressed
            else{
            RW.setPower(brake)
            LW.setPower(brake)
            IN.setPower(brake)
            DR.serPower(brake)
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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

    public void DriveControls(float slowdivisor) {
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        float Ch1 = gamepad1.right_stick_x / slowdivisor;
        float Ch3 = -gamepad1.left_stick_y / slowdivisor;
        float Ch4 = gamepad1.left_stick_x / slowdivisor;


        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float rightfront = Ch3 - Ch1 - Ch4;
        float rightback = Ch3 - Ch1 + Ch4;
        float leftfront = Ch3 + Ch1 + Ch4;
        float leftback = Ch3 + Ch1 - Ch4;


        // clip the right/left values so that the values never exceed +/- 1
        rightback = Range.clip(rightback, -1, 1);
        leftback = Range.clip(leftback, -1, 1);
        rightfront = Range.clip(rightfront, -1, 1);
        leftfront = Range.clip(leftfront, -1, 1);


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightfront = (float) scaleInput(rightfront);
        leftfront = (float) scaleInput(leftfront);
        rightback = (float) scaleInput(rightback);
        leftback = (float) scaleInput(leftback);


        // write the values to the motors
        RF.setPower(rightfront);
        LF.setPower(leftfront);
        RB.setPower(rightback);
        LB.setPower(leftback);



    }

}




