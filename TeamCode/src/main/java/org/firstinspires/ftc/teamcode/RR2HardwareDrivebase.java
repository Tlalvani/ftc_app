package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * This is NOT an opmode.
 * <p>
 * <p>
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front
 */
public class RR2HardwareDrivebase {
    //Lift Values
    int LiftMax = 1770;
    int LiftHang = 710;
    int AutoLiftHang = 720;
    int LiftMin = 0;

    double SortLatchClose = .73;
    double SortLatchOpen = .3;

    double BucketHome = .85;
    double BucketDeploy = .1;

    //IMU VALUES
    double divisorforimu = 10;
    double maxspeedimu = .5;
    double minspeedimu = .3;
    double currentangle = 0;
    double AngleTolerance = 1.2;

    /* Public OpMode members. */
    public DcMotor LF, RF, LB, RB, Intake, Lift1, Lift2, Lift3;
    public Servo Door, Dropper1, Dropper2, HangLatch, Hook, SortLatch;
    public ServoImplEx Bucket;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public RR2HardwareDrivebase() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LB = hwMap.dcMotor.get("LB");
        RB = hwMap.dcMotor.get("RB");
        RF = hwMap.dcMotor.get("RF");
        LF = hwMap.dcMotor.get("LF");
        Intake = hwMap.dcMotor.get("Intake");
        Lift1 = hwMap.dcMotor.get("Lift1");
        Lift2 = hwMap.dcMotor.get("Lift2");
        Lift3 = hwMap.dcMotor.get("Lift3");
        Door = hwMap.servo.get("Door");
        Dropper1 = hwMap.servo.get("Dropper1");
        Dropper2 = hwMap.servo.get("Dropper2");
        HangLatch = hwMap.servo.get("HangLatch");
        Hook = hwMap.servo.get("Hook");
        Bucket = hwMap.get(ServoImplEx.class, "Bucket");
        SortLatch = hwMap.servo.get("SortLatch");


        RB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Lift1.setDirection(DcMotor.Direction.FORWARD);
        Lift2.setDirection(DcMotor.Direction.REVERSE);
        Lift3.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        Intake.setPower(0);
        Lift1.setPower(0);
        Lift2.setPower(0);
        Lift3.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

/*
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); */

       Lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void Lift(double power) {
        Lift1.setPower(power);
        Lift2.setPower(power);
        Lift3.setPower(power);
    }

    public void LiftPosition(int liftposition) {
        Lift1.setTargetPosition(liftposition);
        Lift2.setTargetPosition(liftposition);
        Lift3.setTargetPosition(liftposition);
        Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void LiftWithEncoders() {
        Lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int LiftCurrentPosition() {
        int position = (Lift1.getCurrentPosition() + Lift2.getCurrentPosition() + Lift3.getCurrentPosition()) / 3;
        return position;
    }

    public void DoorOpen() {
        Door.setPosition(0);
    }

    public void DoorRamp(){
        Door.setPosition(0.3);
    }
    public void DoorClose() {
        Door.setPosition(1);
    }

    public void arm(double position, double positionTwo) {
        Dropper1.setPosition(positionTwo);
        Dropper2.setPosition(position);
    }

    public void DeployArm() {
        //arm(0.7, 0.3);
        arm(0.83, 0.17);
    }

    public void DeployArmFurther(){
        arm(0.83, 0.17);
    }

    public void RetractArm() {
        arm(0.14, 0.88);
    }

    public void latchOn() {
        HangLatch.setPosition(0);
    }

    public void latchOff() {
        HangLatch.setPosition(0.5);
    }

    public void autoLiftUp() {

        if (LiftCurrentPosition() < LiftMax-50) {
            DoorClose();
            Lift(1);
            if (LiftCurrentPosition() > LiftMax / 5  && LiftCurrentPosition() < LiftMax / 2 ) {
                DeployArm();
                DeployArm();

            }
            else if (LiftCurrentPosition() > LiftMax / 1.4){
                Bucket.setPosition(BucketDeploy);
            }
        } else {
            Lift(0);
        }
    }

public void hangLiftUp(){
    if (LiftCurrentPosition() < LiftHang) {
        Lift(1);

    } else {
        Lift(0);
    }
}
    public void autoLiftDown() {
        if (LiftCurrentPosition() > LiftMin) {
            Lift(-1);
            if (LiftCurrentPosition() > LiftMax / 2) {
                Bucket.setPosition(BucketHome);
            }

            else{

                DoorOpen();
                RetractArm();
            }


        } else {
            Lift(0);
        }
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


}






