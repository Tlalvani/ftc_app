package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 *
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front
 */
public class RR2Teleop extends OpMode {

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
        float Ch1 = gamepad1.right_stick_x;
        float Ch3 = -gamepad1.left_stick_y;


        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float right = Ch3 + Ch1;

        float left = Ch3 - Ch1;


        right = (float)robot.scaleInput(right);
        left = (float)robot.scaleInput(left);

        robot.LF.setPower(left);
        robot.RF.setPower(right);
        robot.LB.setPower(left);
        robot.RB.setPower(right);





        telemetry.addData("Left: ", robot.LF.getPower());
        telemetry.addData("Right: ", robot.RF.getPower());
    }

    @Override
    public void stop() {
    }
}






