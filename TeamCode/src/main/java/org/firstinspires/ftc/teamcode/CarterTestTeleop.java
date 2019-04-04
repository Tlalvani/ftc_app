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


    /* Declare OpMode members. */
    DcMotor LF;
    DcMotor LB;
    DcMotor RB;
    DcMotor RF;
    DcMotor Intake;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {

        LB = hardwareMap.dcMotor.get("LB");
        LF = hardwareMap.dcMotor.get("LF");
        RB = hardwareMap.dcMotor.get("RB");
        RF = hardwareMap.dcMotor.get("RF");
        Intake = hardwareMap.dcMotor.get("Intake");
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);



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


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("runtime", runtime.seconds());

       float left = gamepad1.left_stick_y;
       float right = gamepad1.right_stick_y;
        float intake = gamepad1.right_trigger;

        right = (float)scaleInput(right);
        left = (float)scaleInput(left);
        intake =  (float)scaleInput(intake);

        LB.setPower(left);
        LF.setPower(left);
        RB.setPower(right);
        RF.setPower(right);
       Intake.setPower(intake);





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




