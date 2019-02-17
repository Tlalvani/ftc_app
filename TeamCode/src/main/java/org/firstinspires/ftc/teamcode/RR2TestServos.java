package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "TestServos", group = "RR2")  // @Autonomous(...) is the other common choice
public class RR2TestServos extends RR2AutoClasses {

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    RR2HardwareDrivebase robot = new RR2HardwareDrivebase();


    /* Constructor */
    public void runOpMode() throws InterruptedException {
        double count=.5;
        initSensors();


    waitForStart();

    while(opModeIsActive()){
        if(gamepad1.right_bumper){
            count += .02;
        }
       robot.IntakeFlipper.setPosition(count);


        telemetry.addData("Count", count);
        telemetry.update();
    }
    }

      }








