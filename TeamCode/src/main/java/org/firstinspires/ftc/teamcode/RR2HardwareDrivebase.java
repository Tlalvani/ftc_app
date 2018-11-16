package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    int LiftMax = 2000;
    int LiftHang = 820;
    int LiftMin = 0;

    //IMU VALUES
    double divisorforimu = 250.0;
    double maxspeedimu = .25;
    double minspeedimu = .045;
    double currentangle = 0;
    double AngleTolerance = 2.2;

    boolean LiftingUp;
    boolean LiftingDown;
    /* Public OpMode members. */
    public DcMotor LF, RF, LB, RB, Intake, Lift1, Lift2, Lift3;
    public Servo Door, Dropper1, Dropper2, HangLatch;


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


        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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
        Door.setPosition(1);
    }

    public void DoorClose() {
        Door.setPosition(0);
    }

    public void arm(double position, double positionTwo) {
        Dropper1.setPosition(positionTwo);
        Dropper2.setPosition(position);
    }

    public void DeployArm() {
        arm(0.7, 0.3);
    }

    public void RetractArm() {
        arm(0.15, 0.85);
    }

    public void latchOn() {
        HangLatch.setPosition(0);
    }

    public void latchOff() {
        HangLatch.setPosition(1);
    }

    public void autoLiftUp() {
        if (LiftCurrentPosition() < 1900) {
            LiftingUp = true;
        } else {
            LiftingUp = false;
        }

     /*   if(LiftingUp) {
            DoorClose();
            LiftPosition(LiftMax);
            Lift(.75);
            if (Lift1.isBusy() & Lift2.isBusy() & Lift3.isBusy()) {
                if (LiftCurrentPosition() > LiftMax / 2) {
                    DeployArm();
                }
            } */

        if (LiftingUp) {
            DoorClose();
            Lift(.75);
            if (LiftCurrentPosition() > LiftMax / 2) {
                DeployArm();
            }

        } else {
            Lift(0);
        }
    }


    public void autoLiftDown() {
        LiftPosition(LiftMin);
        Lift(-.75);
        if (Lift1.isBusy() & Lift2.isBusy() & Lift3.isBusy()) {
            if (LiftCurrentPosition() < LiftMax / 2) {
                RetractArm();
            }
        }
        Lift(0);
        DoorOpen();
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






