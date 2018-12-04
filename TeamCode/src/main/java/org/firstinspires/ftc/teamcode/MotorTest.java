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

@TeleOp(name = "MotorTest", group = "RR2")  // @Autonomous(...) is the other common choice
public class MotorTest extends OpMode {

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


      /* if(gamepad1.a){ robot.LF.setPower(.2);}
       else{robot.LF.setPower(0);}
       if(gamepad1.b){robot.LB.setPower(.2);}
       else{robot.LB.setPower(0);}
*/

      robot.LF.setPower(gamepad1.left_trigger);
      robot.LB.setPower(gamepad1.right_trigger);


        telemetry.addData("LF: ", robot.LF.getPower());
        telemetry.addData("LB: ", robot.LB.getPower());
        telemetry.addData("RF: ", robot.RF.getPower());
        telemetry.addData("RB: ", robot.RB.getPower());
        telemetry.addData("Door: ", robot.Dropper1.getPosition());
        telemetry.addData("Lift Encoders", robot.LiftCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop() {
    }
}







