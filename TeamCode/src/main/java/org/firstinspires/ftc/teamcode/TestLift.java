package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 *
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front
 */

@TeleOp(name = "TestLiftTeleop", group = "RR2")  // @Autonomous(...) is the other common choice
public class TestLift extends OpMode {

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    RR2HardwareDrivebase robot = new RR2HardwareDrivebase();

    /* Constructor */
    @Override
    public void init() {

        robot.init(hardwareMap);


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



if (gamepad1.left_trigger >= 0.9) {
    robot.Lift1.setPower(0.5);
} else {
    robot.Lift1.setPower(0);
}
        if (gamepad1.right_trigger >= 0.9) {
            robot.Lift2.setPower(0.5);
        } else {
            robot.Lift2.setPower(0);
        }


        if (gamepad1.left_bumper) {
            robot.Lift3.setPower(0.5);
        } else {
    robot.Lift3.setPower(0);
        }




        telemetry.addData("Lift1: ", robot.Lift1.getPower());
        telemetry.addData("Lift2: ", robot.Lift2.getPower());
        telemetry.addData("Lift3: ", robot.Lift3.getPower());
    }

    @Override
    public void stop() {
    }
}







