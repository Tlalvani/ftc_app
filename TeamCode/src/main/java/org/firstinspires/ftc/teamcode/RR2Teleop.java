package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "RR2Teleop", group = "RR2")  // @Autonomous(...) is the other common choice
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


        float right = Ch3 - Ch1;
        float left = Ch3 + Ch1;
        float intake = gamepad1.right_trigger - gamepad1.left_trigger;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        intake = Range.clip(intake, -1, 1);


        right = (float) robot.scaleInput(right);
        left = (float) robot.scaleInput(left);

        robot.LF.setPower(left);
        robot.RF.setPower(right);
        robot.LB.setPower(left);
        robot.RB.setPower(right);
        robot.Intake.setPower(intake);


        if (gamepad2.b) {
            robot.DoorClose();
        }

        if(gamepad2.a){
            robot.DoorOpen();
        }

        if (gamepad2.left_bumper) {
            robot.RetractArm();
        } else if (gamepad2.right_bumper) {
            robot.DeployArm();
        }
        else if (gamepad2.y){
            robot.DeployArmFurther();
        }

        if (gamepad2.dpad_left){
            robot.latchOff();
        } else if (gamepad2.dpad_right) {
            robot.latchOn();
        }

        if (gamepad2.dpad_down||gamepad1.dpad_down) {
            robot.autoLiftDown();
        }

        else if (gamepad2.dpad_up||gamepad1.dpad_up){
           robot.autoLiftUp();
        }

        else{robot.Lift((gamepad2.right_trigger - gamepad2.left_trigger) /* * (gamepad2.right_trigger - gamepad2.left_trigger) * (gamepad2.right_trigger - gamepad2.left_trigger)*/);}


    /*    if (gamepad2.b) {
            robot.Lift((gamepad2.right_trigger - gamepad2.left_trigger) * (gamepad2.right_trigger - gamepad2.left_trigger) * (gamepad2.right_trigger - gamepad2.left_trigger));
        } else {
        robot.Lift((gamepad2.right_trigger - gamepad2.left_trigger) * (gamepad2.right_trigger - gamepad2.left_trigger) * (gamepad2.right_trigger - gamepad2.left_trigger) / 2);
    } */

        telemetry.addData("Left: ", robot.LF.getPower());
        telemetry.addData("Right: ", robot.RF.getPower());
        telemetry.addData("Door: ", robot.Dropper1.getPosition());
        telemetry.addData("Lift Encoders", robot.LiftCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop() {
    }
}







