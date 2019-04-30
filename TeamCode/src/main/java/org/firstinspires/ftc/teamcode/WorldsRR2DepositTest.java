package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front
 */

@TeleOp(name = "RR2DepositIntakeTest", group = "RR2")  // @Autonomous(...) is the other common choice
public class WorldsRR2DepositTest extends OpMode {

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
        robot.IntakeFlipper.setPosition(.9);
        robot.Intake.setPower(.5);



        telemetry.addData("LF: ", robot.LF.getCurrentPosition());
        telemetry.addData("LB: ", robot.LB.getCurrentPosition());
        telemetry.addData("RF: ", robot.RF.getCurrentPosition());
        telemetry.addData("RB: ", robot.RB.getCurrentPosition());


        telemetry.addData("RF: ", robot.RF.getPower());
        telemetry.addData("LB: ", robot.LB.getPower());
        telemetry.addData("RB: ", robot.RB.getPower());
        telemetry.addData("LF: ", robot.LF.getPower());

        telemetry.addData("Lift Encoders", robot.LiftCurrentPosition());

        telemetry.addData("Extend Lift Encoders", robot.IntakeLift.getCurrentPosition());

        telemetry.update();

    }

    @Override
    public void stop() {
    }
}







