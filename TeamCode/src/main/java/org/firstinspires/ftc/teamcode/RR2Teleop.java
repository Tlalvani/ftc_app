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
        robot.latchOff();

if(gamepad1.right_trigger > .1){
    robot.DoorOpen();
    robot.RetractArm();
    robot.Bucket.setPosition(robot.BucketHome);
    robot.SortLatch.setPosition(robot.SortLatchClose);

}
        if (gamepad2.b || gamepad1.b) {
            robot.DoorClose();
            robot.SortLatch.setPosition(robot.SortLatchClose);
        }

        if(gamepad1.y){
        robot.DoorOpen();
        }
        if(gamepad2.a || gamepad1.a){
          robot.SortLatch.setPosition(robot.SortLatchOpen);
        }

        if (gamepad2.left_bumper) {
            robot.RetractArm();
        } else if (gamepad2.right_bumper) {
            robot.DeployArm();
        }

        if (gamepad2.dpad_left){
            robot.latchOff();
        } else if (gamepad2.dpad_right) {
            robot.latchOn();
        }


        robot.Intake.setPower(intake);

        if (gamepad2.dpad_down||gamepad1.left_bumper) {
            robot.autoLiftDown();

        }

        else if (gamepad2.dpad_up||gamepad1.right_bumper){
           robot.autoLiftUp();

        }

        else if(gamepad2.x || gamepad1.dpad_up){
            robot.hangLiftUp();
            robot.Hook.setPosition(0);

        }

        else if(gamepad1.x){
            robot.Lift(1);
        }
        else{robot.Lift((gamepad2.right_trigger) - gamepad2.left_trigger);
        robot.Hook.setPosition(1);
        }



        telemetry.addData("LF: ", robot.LF.getCurrentPosition());
        telemetry.addData("LB: ", robot.LB.getCurrentPosition());
        telemetry.addData("RF: ", robot.RF.getCurrentPosition());
        telemetry.addData("RB: ", robot.RB.getCurrentPosition());
        telemetry.addData("Door: ", robot.Dropper1.getPosition());
        telemetry.addData("Lift Encoders", robot.LiftCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop() {
    }
}







