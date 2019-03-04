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

@TeleOp(name = "RR2Teleop", group = "RR2")  // @Autonomous(...) is the other common choice
public class MecanumRR2Teleop extends OpMode {

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
        float Ch4 = gamepad1.left_stick_x;


        float rightfront = Ch3 - Ch1 - Ch4;
        float rightback = Ch3 - Ch1 + Ch4;
        float leftfront = Ch3 + Ch1 + Ch4;
        float leftback = Ch3 + Ch1 - Ch4;



        rightback = Range.clip(rightback, -1, 1);
        leftback = Range.clip(leftback, -1, 1);
        rightfront = Range.clip(rightfront, -1, 1);
        leftfront = Range.clip(leftfront, -1, 1);




        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightfront = (float) robot.scaleInput(rightfront);
        leftfront = (float) robot.scaleInput(leftfront);
        rightback = (float) robot.scaleInput(rightback);
        leftback = (float) robot.scaleInput(leftback);


        // write the values to the motors
        robot.RF.setPower(rightfront);
        robot.LF.setPower(leftfront);
        robot.RB.setPower(rightback);
        robot.LB.setPower(leftback);
        robot.latchOff();


        //Auto Set when intake runs
        if (gamepad1.right_trigger > .1) {
            robot.Intake.setPower(-.75);
            robot.Intake2.setPower(-.75);
            if(robot.LiftCurrentPosition() < 100){
                robot.DoorOpen();
                robot.RetractArm();
                robot.Bucket.setPosition(robot.BucketHome);
                robot.SortLatch.setPosition(robot.SortLatchClose);
            }
        }
        else if(gamepad1.left_trigger>.1){
            robot.Intake.setPower(.75);
            robot.Intake2.setPower(.75);
        }
        else{
            robot.Intake.setPower(0);
            robot.Intake2.setPower(0);
        }
        //Sort Latch
        if (gamepad1.b) {
            robot.DoorClose();
            robot.SortLatch.setPosition(robot.SortLatchClose);
        }

        if (gamepad1.a) {
            robot.SortLatch.setPosition(robot.SortLatchOpen);
            robot.DoorOpen();
        }

        if (gamepad2.a){
            robot.DoorOpen();
        }

        //Servo Arm Deploy
        if (gamepad2.left_bumper) {
            robot.RetractArm();
        } else if (gamepad2.right_bumper) {
            robot.DeployArm();
        }

        //Latch
        if (gamepad2.dpad_left) {
            robot.latchOff();
        } else if (gamepad2.dpad_right) {
            robot.latchOn();
        }

        //Lift

        if (gamepad1.x){
            robot.AutoLiftingDown = true;
            robot.AutoLiftingUp = false;
        }
        else if (gamepad1.y) {
            robot.AutoLiftingUp = true;
            robot.AutoLiftingDown = false;
        }

        else if (gamepad1.dpad_up) {
            robot.Lift(1);
            robot.AutoLiftingUp = false;
            robot.AutoLiftingDown = false;
            robot.Hook.setPosition(0);
            robot.Bucket.setPwmDisable();
        }

        else if (gamepad2.x || gamepad1.dpad_right) {
            robot.hangLiftUp();
            robot.Hook.setPosition(0);
            robot.Bucket.setPwmDisable();
            robot.AutoLiftingDown = false;
            robot.AutoLiftingUp = false;

        }
        else if (gamepad1.dpad_left) {
            robot.Lift(-1);
            robot.AutoLiftingDown = false;
            robot.AutoLiftingUp = false;
        }

        else if(robot.AutoLiftingDown){
            robot.autoLiftDown();
        }

        else if(robot.AutoLiftingUp){
            robot.DeployArm();
            robot.autoLiftUp();

        }

        else {
            robot.Lift((gamepad2.right_trigger) - gamepad2.left_trigger);
            robot.Hook.setPosition(1);
        }

if(gamepad1.right_stick_button){
            robot.Bucket.setPosition(.05);
}
        //Extending Intake

        if(gamepad1.left_bumper && gamepad1.right_bumper){
            robot.IntakeLift.setPower(0);
            robot.IntakeFlipper.setPosition(.65);
        }
       else if(gamepad1.right_bumper){
            robot.IntakeLift.setPower(1);
        }
        else if (gamepad1.left_bumper){
            robot.IntakeLift.setPower(-1);

        }
        else{
            robot.IntakeLift.setPower(0);

        }

        if(robot.IntakeLift.getCurrentPosition()<150){
            robot.IntakeLatchOpen();
        }
        else{
            robot.IntakeLatchClose();
        }

       if(robot.IntakeLift.getCurrentPosition()<300){
            robot.IntakeFlipper.setPosition(robot.intakedown);
        }



        else if(gamepad1.left_bumper||gamepad1.right_bumper){
            if(gamepad1.right_trigger>.1){
                robot.IntakeFlipper.setPosition(robot.intakedown);
            }
            else{ robot.IntakeFlipper.setPosition(.65);}

        }

        else{
            robot.IntakeFlipper.setPosition(robot.intakedown);
            robot.IntakeLatchClose();
        }
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







