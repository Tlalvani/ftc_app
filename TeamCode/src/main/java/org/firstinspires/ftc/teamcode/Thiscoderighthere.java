package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 *
 *
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front


 */
public class RRHardwareDrivebase
{

    //IMU VALUES
    double divisorforimu = 250.0;
    double otherdivisorforimu = 200.0;
    double maxspeedimu = .25;
    double minspeedimu = .045;
    double currentangle = 0;


    int colummn = 0;
    /* Public OpMode members. */
    public DcMotor  LF   = null;
    public DcMotor  RF  = null;
    public DcMotor RB    = null;
    public DcMotor LB   = null;

    public DcMotor Lift1   = null;
    public DcMotor Lift2  = null;

    public DcMotor RelicLift1   = null;
    public DcMotor RelicLift2  = null;

    public Servo LeftClaw = null;
    public Servo RightClaw = null;

    public Servo TopLeftClaw = null;
    public Servo TopRightClaw = null;

    public Servo RelicClaw = null;
    public Servo RelicThing = null;
    public CRServo RelicRotater = null;

    public Servo JewelArm = null;
    public Servo JewelHitter = null;

    public Servo Hitter = null;

    public CRServo RightIntake = null;
    public CRServo LeftIntake = null;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;



 /* Declare OpMode members. */
    double RelicClawOpen = .3;
    double RelicClawClosed = .9;

  double relicThingPosition = .7;
    double CurrentLiftPosition = 0;
    double LiftThreshold = 300;

   //Savox Servos theoretically max at .2 and .8 as they only range from 900 to 2100 pwm

    double BottomOneBlockLeft = .6;
    double BottomOneBlockRight = .4;
    double TopOneBlockLeft = .5;
    double TopOneBlockRight = .5;


    double BottomCloseLeft = .8;
    double BottomCloseRight = .2;

    double TopCloseLeft = .8;
    double TopCloseRight = .2;

    double BottomOpenLeft = .09;
    double BottomOpenRight = .85;

    double DumpClawLeft = .37;
    double DumpClawRight = .7;
    double TopDumpClawLeft = .3;
    double TopDumpClawRight = .7;

    double BottomStraightLeft = .27;
    double BottomStaightRight = .8;
    double TopStraightLeft = .2;
    double TopStraightRight = .8;


    double BottomShutdownLeft = .83;
    double BottomShutdownRight = .17;

    double JewelHitterHome = .2;
    double JewelHitterOut = .75;
    double JewelHitterLeft = 1;
    double JewelHitterRight = .5;

    double JewelArmHome = 1;
    double JewelArmMiddle = .3;
    double JewelArmDown = .45;

    double HitterHome = .22;

    double AngleTolerance = 2.2;

public boolean red = false;
public boolean blue = false;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public RRHardwareDrivebase(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LB  = hwMap.dcMotor.get("LB");
        RB = hwMap.dcMotor.get("RB");
        RF = hwMap.dcMotor.get("RF");
        LF = hwMap.dcMotor.get("LF");
        Lift1 = hwMap.dcMotor.get("Lift1");
        Lift2 = hwMap.dcMotor.get("Lift2");
        RelicLift1 = hwMap.dcMotor.get("RelicLift1");
        RelicLift2 = hwMap.dcMotor.get("RelicLift2");

        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

       LeftClaw  = hwMap.get(Servo.class, "LeftClaw");
        RightClaw = hwMap.get(Servo.class, "RightClaw");
        TopLeftClaw  = hwMap.get(Servo.class, "TopLeftClaw");
        TopRightClaw = hwMap.get(Servo.class, "TopRightClaw");

        RelicClaw = hwMap.get(Servo.class, "RelicClaw");
        RelicThing = hwMap.get(Servo.class, "RelicThing");
        RelicRotater = hwMap.get(CRServo.class, "RelicRotater");

        JewelArm = hwMap.get(Servo.class, "JewelArm");
        JewelHitter = hwMap.get(Servo.class, "JewelHitter");

        Hitter = hwMap.get(Servo.class, "Hitter");

        LeftIntake = hwMap.get(CRServo.class, "LeftIntake");
        RightIntake = hwMap.get(CRServo.class, "RightIntake");


        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);

        Lift1.setDirection(DcMotor.Direction.REVERSE);
        Lift2.setDirection(DcMotor.Direction.REVERSE);

        RelicLift1.setDirection(DcMotor.Direction.FORWARD);
        RelicLift2.setDirection(DcMotor.Direction.REVERSE);



        // Set all motors to zero power
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        Lift1.setPower(0);
        Lift2.setPower(0);
        RelicLift1.setPower(0);
        RelicLift2.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        RelicLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RelicLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LiftWithEncoders();

        RelicLift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RelicLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RelicLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RelicLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //JewelHitter.setPosition(JewelHitterLeft);
        JewelArm.setPosition(JewelArmHome);
        Hitter.setPosition(HitterHome);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */

//FUNCTIONS

    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

//DRIVE FUNCTIONS
    public void Drive(double leftpower, double rightpower) {

        LF.setPower(leftpower);
        RF.setPower(rightpower);
        LB.setPower(leftpower);
        RB.setPower(rightpower);
    }


    public void Strafe(double leftforwardpower, double rightforwardpower, double leftbackpower, double rightbackpower) {

        LF.setPower(leftforwardpower);
        RF.setPower(rightforwardpower);
        LB.setPower(leftbackpower);
        RB.setPower(rightbackpower);
    }

    public void DriveTargetPosition(int LFpower, int LBpower, int RFpower, int RBpower){
        LB.setTargetPosition(LB.getCurrentPosition() + LBpower);
        RB.setTargetPosition(RB.getCurrentPosition() + RBpower);
        LF.setTargetPosition(LF.getCurrentPosition() + LFpower);
        RF.setTargetPosition(RF.getCurrentPosition() + RFpower);

        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void DrivebaseBusy(){
        while (LB.isBusy() & RF.isBusy() & LF.isBusy() & RB.isBusy()) {
    }
    }


    public void DrivebaseWithEncoders(){
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //LIFT FUNCTIONS
    public void LiftPosition(int liftposition){
        Lift1.setTargetPosition(liftposition);
        Lift2.setTargetPosition(liftposition);
        Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Lift(double liftpower) {
        Lift1.setPower(liftpower);
        Lift2.setPower(liftpower);
    }

    public void BusyLift(){
        while (Lift1.isBusy() & Lift2.isBusy()) {
        }
    }

    public void LiftWithEncoders(){
        Lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void GetLiftPosition(){
        CurrentLiftPosition = (Lift1.getCurrentPosition() + Lift2.getCurrentPosition())/2;
    }


    public void LiftHome(){
        LiftPosition(0);
        Lift(-.3);
        BusyLift();
        Lift(0);
        LiftWithEncoders();
    }

    //CLAW FUNCTIONS

    public void IntakeOne(){
        LeftClaw.setPosition(BottomOneBlockLeft);
        RightClaw.setPosition(BottomOneBlockRight);
        TopLeftClaw.setPosition(TopOneBlockLeft);
        TopRightClaw.setPosition(TopOneBlockRight);
    }

    public void IntakeTopOne(){

        TopLeftClaw.setPosition(TopOneBlockLeft);
        TopRightClaw.setPosition(TopOneBlockRight);
    }

    public void IntakeBottomTwo(){

    }

    public void OpenClaw(){
        LeftClaw.setPosition(BottomOpenLeft);
        RightClaw.setPosition(BottomOpenRight);
        TopLeftClaw.setPosition(BottomOpenLeft);
        TopRightClaw.setPosition(BottomOpenRight);
    }

    public void TopOpenClaw(){
        TopLeftClaw.setPosition(BottomOpenLeft);
        TopRightClaw.setPosition(BottomOpenRight);
    }

    public void AutoStraightClaw(){
        TopLeftClaw.setPosition(TopStraightLeft);
        TopRightClaw.setPosition(TopStraightRight);

    }

    public void DumpClaw(){
        LeftClaw.setPosition(DumpClawLeft);
        RightClaw.setPosition(DumpClawRight);
        TopLeftClaw.setPosition(TopDumpClawLeft);
        TopRightClaw.setPosition(TopDumpClawRight);
    }

    public void TeleDumpClaw(){
        LeftClaw.setPosition(DumpClawLeft);
        RightClaw.setPosition(DumpClawRight);
        TopLeftClaw.setPosition(TopDumpClawLeft);
        TopRightClaw.setPosition(TopDumpClawRight);
        Hitter.setPosition(.65);
    }

    public void TeleDumpTopClaw(){
        TopLeftClaw.setPosition(TopDumpClawLeft);
        TopRightClaw.setPosition(TopDumpClawRight);
        Hitter.setPosition(.65);
    }

    public void DumpTopClaw(){
        TopLeftClaw.setPosition(TopDumpClawLeft);
        TopRightClaw.setPosition(TopDumpClawRight);
    }


    public void DumpBottomClaw(){
        LeftClaw.setPosition(DumpClawLeft);
        RightClaw.setPosition(DumpClawRight);
    }

    public void CloseClaw(){
        LeftClaw.setPosition(BottomCloseLeft);
        RightClaw.setPosition(BottomCloseRight);
        TopLeftClaw.setPosition(TopCloseLeft);
        TopRightClaw.setPosition(TopCloseRight);
    }

    public void CloseTopClaw(){
        TopLeftClaw.setPosition(TopCloseLeft);
        TopRightClaw.setPosition(TopCloseRight);
    }

    public void CloseBottomClaw(){
        LeftClaw.setPosition(BottomCloseLeft);
        RightClaw.setPosition(BottomCloseRight);
    }

    public void StraightBottomRightClaw(){
        RightClaw.setPosition(BottomStaightRight);

    }

    public void StraightBottomLeftClaw(){

        LeftClaw.setPosition(BottomStraightLeft);
    }

    public void StraightClaw(){
        LeftClaw.setPosition(BottomStraightLeft);
        RightClaw.setPosition(BottomStaightRight);
        TopLeftClaw.setPosition(TopStraightLeft);
        TopRightClaw.setPosition(TopStraightRight);
    }

    public void BottomStraightClaw(){
        LeftClaw.setPosition(BottomStraightLeft);
        RightClaw.setPosition(BottomStaightRight);
    }


    public void StraightTopClaw(){
        TopLeftClaw.setPosition(TopStraightLeft);
        TopRightClaw.setPosition(TopStraightRight);
    }


    public void BottomShutdownClaw(){
        LeftClaw.setPosition(BottomShutdownLeft);
        RightClaw.setPosition(BottomShutdownRight);
    }

    public void RunIntake(){
        LeftIntake.setPower(-1);
        RightIntake.setPower(1);
    }

    public void ReverseIntake(){
        LeftIntake.setPower(1);
        RightIntake.setPower(-1);
    }

    public void StopIntake(){
        LeftIntake.setPower(-0.00001);
        RightIntake.setPower(0);
    }

//AUTO FUNCTIONS

public void LoadVuforia(){

}

public void RedAutoStart() {
    //Ascend and Open Claw
    // RelicThing.setPosition(relicThingPosition);

    PutDownJewelHitter();
    waitForTick(1000);
    PutDownJewelArm();
    IntakeTopOne();
    waitForTick(1000);
    DetectJewelColor();

    waitForTick(1000);
    RedKnockOffJewel();

    LiftPosition(500);
    Lift(.3);
    BusyLift();
    Lift(0);
    LiftWithEncoders();

    waitForTick(300);
    // Descend
    JewelArmReturn();
    // waitForTick(300);

    //Raise Lift Slightly
}


    public void BlueAutoStart() {
        //Ascend and Open Claw
        //RelicThing.setPosition(relicThingPosition);
PutDownJewelHitter();
        waitForTick(1000);
        PutDownJewelArm();
        IntakeTopOne();
        waitForTick(1000);
     /*   LiftPosition(600);
        Lift(.3);
        BusyLift();
        Lift(0);
        LiftWithEncoders(); */
        DetectJewelColor();
     //   AutoStraightClaw();
        waitForTick(1000);

        BlueKnockOffJewel();

        LiftPosition(500);
        Lift(.3);
        BusyLift();
        Lift(0);
        LiftWithEncoders();
        waitForTick(300);
        // Descend
    /*    LiftPosition(-50);
        Lift(-.5);
        BusyLift();
        Lift(0);
        LiftWithEncoders(); */
     //   waitForTick(400);
        //Close Claw
      //  IntakeTopOne();
        JewelArmReturn();
      //  waitForTick(300);

        //Raise Lift Slightly

    }


    public void RedAutoMultiStart() {
        //Ascend and Open Claw
        // RelicThing.setPosition(relicThingPosition);
        PutDownJewelHitter();
        waitForTick(1000);
        PutDownJewelArm();
        IntakeTopOne();
        waitForTick(1000);

        DetectJewelColor();

        waitForTick(1000);
        RedKnockOffJewel();

        LiftPosition(900);
        Lift(.3);
        BusyLift();
        Lift(0);
        LiftWithEncoders();

        waitForTick(300);
        // Descend
        JewelArmReturn();
        // waitForTick(300);

        //Raise Lift Slightly
    }

    public void NearMulti(){
        RunIntake();

        DriveTargetPosition(1000,1000,1000,1000);
        Drive(.5,.5);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
       waitForTick(2000);
        IntakeOne();
        StopIntake();
waitForTick(500);
       LiftPosition(1300);
       Lift(.3);
       BusyLift();
       Lift(0);
       LiftWithEncoders();
       waitForTick(500);

        DriveTargetPosition(-700,-700,-700,-700);
        Drive(-.75,-.75);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(500);
    }

public void RedStrafetoTarget(){
    DriveTargetPosition(700,-700,-700,700);
    Drive(.12,.12);
    DrivebaseBusy();
    Drive(0,0);
    DrivebaseWithEncoders();
}
    public void KnockLeftJewel(){
JewelHitter.setPosition(JewelHitterLeft);
    }



    public void KnockRightJewel(){
JewelHitter.setPosition(JewelHitterRight);
    }

    public void RedDriveOffBoard(){

        DriveTargetPosition(800,800,800,800);
        Drive(.2,.2);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();


    }

    public void BlueDriveOffBoard(){

        DriveTargetPosition(700,700,700,700);
        Drive(.2,.2);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();

    }
public void RedStrafeOffBoard(){
    DriveTargetPosition(900,-900,-900,900);
    Drive(.2,.2);
    DrivebaseBusy();
    Drive(0,0);
    DrivebaseWithEncoders();

}


    public void RedDriveAfterBoard(){
        DriveTargetPosition(300,300,300,300);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(250);

    }


    public void RedDrivetoBoard(){
        DriveTargetPosition(750,750,750,750);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(250);

    }

    public void BlueStrafeOffBoard(){
        DriveTargetPosition(-900,900,900,-900);
        Drive(.2,.2);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();

    }

    public void BlueDriveAfterBoard(){

        DriveTargetPosition(300,300,300,300);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(250);
    }

    public void DrivetoFarRedBox(){


        DriveTargetPosition(800,800,800,800);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(250);

        DriveTargetPosition(200,-200,-200,200);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();

      /*  DriveTargetPosition(200,-200,-200,200);
        Drive(.25,.25);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(500);

      /*  DriveTargetPosition(1500,1500,1500,1500);
        Drive(.5,.5);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();*/
    }


    public void DrivetoFarBlueBox(){


        DriveTargetPosition(750,750,750,750);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(250);

     /* DriveTargetPosition(-325,325,325,-325);
        Drive(.25,.25);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        waitForTick(500); */

      /*  DriveTargetPosition(1500,1500,1500,1500);
        Drive(.5,.5);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();*/
    }

public void LeftPutInBox() {
    if (colummn == 1) {
        DriveintoBox();
    }
    else if (colummn == 2){
        StrafetoRightCenterofBox();
        waitForTick(250);
        DriveintoBox();
    }

    else if (colummn == 3) {
        StrafetoRightofBox();
        waitForTick(250);
        DriveintoBox();
    }


    else {
        DriveintoBox();

    }
}

    public void RightPutInBox() {
        if (colummn == 3) {
            DriveintoBox();
        }
        else if (colummn == 2){
            StrafetoLeftCenterofBox();
            waitForTick(250);
            DriveintoBox();
        }

        else if (colummn == 1) {
            StrafetoLeftofBox();
            waitForTick(250);
            DriveintoBox();
        }


        else {
            DriveintoBox();

        }
    }

    public void LeftMultiPutInBox() {
        if (colummn == 1) {
            MultiDriveintoBox();
        }
        else if (colummn == 2){
            StrafetoRightCenterofBox();
            waitForTick(250);
            MultiDriveintoBox();
        }

        else if (colummn == 3) {
            StrafetoRightofBox();
            waitForTick(250);
            MultiDriveintoBox();
        }


        else {
            MultiDriveintoBox();

        }
    }

    public void RightMultiPutInBox() {
        if (colummn == 3) {
            MultiDriveintoBox();
        }
        else if (colummn == 2){
            StrafetoLeftCenterofBox();
            waitForTick(250);
            MultiDriveintoBox();
        }

        else if (colummn == 1) {
            StrafetoLeftofBox();
            waitForTick(250);
            MultiDriveintoBox();
        }


        else {
            MultiDriveintoBox();

        }
    }

    public void StrafetoLeftofBox(){
        DriveTargetPosition(-500,500,500,-500);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
        DrivebaseWithEncoders();

    }

    public void StrafetoRightofBox(){
        DriveTargetPosition(500,-500,-500,500);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
    }


    public void StrafetoRightCenterofBox(){
        DriveTargetPosition(250,-250,-250,250);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
    }


    public void StrafetoLeftCenterofBox(){
        DriveTargetPosition(-250,250,250,-250);
        Drive(.3,.3);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
    }


    public void DriveintoBox()
    {
        DriveTargetPosition(300, 300, 300, 300);
        Drive(.2, .2);
        DrivebaseBusy();
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);
        StraightTopClaw();
        waitForTick(200);
        DriveTargetPosition(-500, -500, -500, -500);
    }

    public void MultiDriveintoBox()
    {
        DriveTargetPosition(300, 300, 300, 300);
        Drive(.2, .2);
        DrivebaseBusy();
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);
        StraightClaw();
        waitForTick(200);
        DriveTargetPosition(-500, -500, -500, -500);
    }

    public void RedNearPush(){
        DriveTargetPosition(150, 150, 150, 150);
        Drive(.2, .2);
        DrivebaseBusy();
        Drive(0, 0);
        DrivebaseWithEncoders();
    }
    public void RedNearStrafe(){
        DriveTargetPosition(-200,200,200,-200);
        Drive(.2,.2);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
    }

    public void BlueNearPush() {
        DriveTargetPosition(150, 150, 150, 150);
        Drive(.2, .2);
        DrivebaseBusy();
        Drive(0, 0);
        DrivebaseWithEncoders();
}

    public void BlueNearStrafe(){
        DriveTargetPosition(150,-150,-150,150);
        Drive(.12,.12);
        DrivebaseBusy();
        Drive(0,0);
        DrivebaseWithEncoders();
    }

    public void PutDownJewelHitter(){
        JewelHitter.setPosition(JewelHitterOut);
    }
    public void PutDownJewelArm(){
        JewelArm.setPosition(JewelArmDown);


    }
    public void DetectJewelColor(){

        if (sensorColor.red() > sensorColor.blue() & sensorColor.red() > 15) {
            red = true;
            blue = false;
        }

        else if (sensorColor.red() < sensorColor.blue() & sensorColor.blue() > 15) {
            red = false;
            blue = true;
        }


    }

    public void RedKnockOffJewel(){

        if ((red == true) & (blue == false)) {
            KnockLeftJewel();
        }

        else if ((blue == true) & (red == false)){
            KnockRightJewel();
        }

        else {
            JewelArm.setPosition(JewelArmHome);
           // JewelHitter.setPosition(JewelArmHome);
        }
    }

    public void BlueKnockOffJewel(){

        if ((red == false) & (blue == true)) {
            KnockLeftJewel();
        }

        else if ((blue == false) & (red == true)){
            KnockRightJewel();
        }

        else {
            JewelArm.setPosition(JewelArmHome);
            //JewelHitter.setPosition(JewelArmHome);
        }
    }

    public void RedJewelProgram() {
    PutDownJewelArm();
    waitForTick(500);
    DetectJewelColor();
    waitForTick(1000);
    RedKnockOffJewel();
}

public void JewelArmReturn(){
    JewelArm.setPosition(JewelArmHome);
   // JewelHitter.setPosition(JewelArmHome);
}

public void HammerTime(){


    waitForTick(250);
   DrivebaseWithEncoders();
    Drive(.3, .3);
   waitForTick(1750);
    Drive(0, 0);
    DrivebaseWithEncoders();
    waitForTick(250);
    ReverseIntake();
    DriveTargetPosition(-200, -200, -200, -200);
    Drive(-.2, -.2);
    DrivebaseBusy();
    Drive(0, 0);
    DrivebaseWithEncoders();
    waitForTick(250);
StopIntake();
}


    public void MultiHammerTime(){


        waitForTick(250);
        DrivebaseWithEncoders();
        Drive(.2, .2);
        waitForTick(1750);
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);
        DriveTargetPosition(-200, -200, -200, -200);
        Drive(-.2, -.2);
        DrivebaseBusy();
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);

    }


    public void LastHit() {


        DrivebaseWithEncoders();
        Drive(.4, 0);
        waitForTick(1500);
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);
        DriveTargetPosition(-200, -200, -200, -200);
        Drive(-.2, -.2);
        DrivebaseBusy();
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);




    }

    public void RightLastHit() {


        DrivebaseWithEncoders();
        Drive(0, .4);
        waitForTick(1500);
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);
        DriveTargetPosition(-200, -200, -200, -200);
        Drive(-.2, -.2);
        DrivebaseBusy();
        Drive(0, 0);
        DrivebaseWithEncoders();
        waitForTick(250);




    }}






