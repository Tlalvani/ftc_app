package org.firstinspires.ftc.teamcode;

/*
Modern Robotics Range Sensors Example
Created 10/31/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "range28" (MRI Range Sensor with default I2C address 0x28
I2CDevice "range2a" (MRI Color Sensor with I2C address 0x2a

ModernRoboticsI2cGyro is not being used because it does not support .setI2CAddr().

To change range sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "Checker Standard Blue Auto", group = "VV")

public class VVCheckerBlueStandardAuto extends LinearOpMode {

    /* Declare OpMode members. */


    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;
    DcMotor Intake;
    DcMotor RW;
    DcMotor LW;
    Servo Stopper;
    Servo Hitter;
    Servo BlueAuto;
    Servo RedAuto;

    ColorSensor sensorRGB;
    DeviceInterfaceModule cdim;

    OpticalDistanceSensor v_sensor_ods;

    boolean a_ods_white_tape_detected ()

    {
        //
        // Assume not.
        //
        boolean l_return = false;

        if (v_sensor_ods != null)
        {
            //
            // Is the amount of light detected above the threshold for white
            // tape?
            //
            if (v_sensor_ods.getLightDetected () > 0.4)
            {
                l_return = true;
            }
        }

        //
        // Return
        //
        return l_return;

    }


    static final int LED_CHANNEL = 0;
    int encoders = -3400;
    int firstbutton = 198;
    int secondbutton = 180;
    int thirdbutton = 89;
    int fourthbutton = 76;
    int emptyspace = 160;



    byte[] rangeAcache;
    byte[] rangeCcache;
    byte[] rangeBcache;

    I2cDevice rangeA;
    I2cDevice rangeC;
    I2cDevice rangeB;
    I2cDeviceSynch rangeAreader;
    I2cDeviceSynch rangeCreader;
    I2cDeviceSynch rangeBreader;


    @Override
    public void runOpMode() throws InterruptedException {


        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        boolean bLedOn = false;


        // get a reference to our DeviceInterfaceModule object.
        cdim = hardwareMap.deviceInterfaceModule.get("cdim");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("color1");

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);


        LB = hardwareMap.dcMotor.get("LB");
        LF = hardwareMap.dcMotor.get("LF");
        RB = hardwareMap.dcMotor.get("RB");
        RF = hardwareMap.dcMotor.get("RF");
        Intake = hardwareMap.dcMotor.get("Intake");
        LW = hardwareMap.dcMotor.get("LW");
        RW = hardwareMap.dcMotor.get("RW");
        Stopper = hardwareMap.servo.get("Stopper");
        Hitter = hardwareMap.servo.get("BlueHitter");
        BlueAuto = hardwareMap.servo.get("BlueAuto");
        RedAuto = hardwareMap.servo.get("RedAuto");
        rangeA = hardwareMap.i2cDevice.get("range28");
        rangeC = hardwareMap.i2cDevice.get("range2a");
        rangeB = hardwareMap.i2cDevice.get("range80");
        v_sensor_ods = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        rangeAreader = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);
        rangeCreader = new I2cDeviceSynchImpl(rangeC, I2cAddr.create8bit(0x2a), false);
        rangeBreader = new I2cDeviceSynchImpl(rangeB, I2cAddr.create8bit(0x80), false);


        rangeAreader.engage();
        rangeCreader.engage();
        rangeBreader.engage();


        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        LW.setDirection(DcMotor.Direction.FORWARD);
        RW.setDirection(DcMotor.Direction.REVERSE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set to FORWARD if using AndyMark motors
        telemetry.addData("Status", "Initialized");

        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (v_sensor_ods != null)
        {
            v_sensor_ods.getLightDetected ();

        }



        waitForStart();


        Hitter.setPosition(.5);
        BlueAuto.setPosition(0);
        RedAuto.setPosition(.6);



        telemetry.addData("White Line", v_sensor_ods.getLightDetected ());
        telemetry.addData("LED", bLedOn ? "On" : "Off");
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue ", sensorRGB.blue());
        telemetry.addData("RF: ", RF.getCurrentPosition());
        telemetry.addData("RB: ", RB.getCurrentPosition());
        telemetry.addData("LB: ", LB.getCurrentPosition());
        telemetry.addData("LF: ", LF.getCurrentPosition());
        telemetry.update();

        {
            Stopper.setPosition(.45);
            v_sensor_ods.getLightDetected();
            LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LB.setTargetPosition(-100);
            RB.setTargetPosition(-100);
            LF.setTargetPosition(-100);
            RF.setTargetPosition(-100);

            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LB.setPower(-.25);
            RB.setPower(-.25);
            LF.setPower(-.25);
            RF.setPower(-.25);


            while (LB.isBusy() & RF.isBusy() & LF.isBusy() & RB.isBusy()) {
            }


            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);

            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

           sleep(750);
            RW.setPower(.75);
            LW.setPower(.35);
            sleep(1000);
            Stopper.setPosition(.7);
            sleep(1000);
            Stopper.setPosition(.45);
            Intake.setPower(-1);
            sleep(2000);
            Stopper.setPosition(.7);
            sleep(1500);
            Intake.setPower(0);
            RW.setPower(0);
            LW.setPower(0);
            Stopper.setPosition(.45);

            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LB.setTargetPosition(LB.getCurrentPosition() - 300);
            RB.setTargetPosition(RB.getCurrentPosition() - 300);
            LF.setTargetPosition(LF.getCurrentPosition() - 300);
            RF.setTargetPosition(RF.getCurrentPosition() - 300);

            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LB.setPower(-.3);
            RB.setPower(-.3);
            LF.setPower(-.3);
            RF.setPower(-.3);


            while (LB.isBusy() & RF.isBusy() & LF.isBusy() & RB.isBusy()) {
            }


            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);

            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

           RB.setTargetPosition(RB.getCurrentPosition() + encoders);
            LF.setTargetPosition(LF.getCurrentPosition() +  encoders);



            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(-.50);
            LF.setPower(-.50);


            while (RB.isBusy() & LF.isBusy()) {

                    if (RB.getCurrentPosition() < -3000 || LF.getCurrentPosition() < -3000) {
                        BlueAuto.setPosition(.55);
                    }


            }


            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);

            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            RB.setTargetPosition(RB.getCurrentPosition() + 900);
            LB.setTargetPosition(900);
            RF.setTargetPosition(900);
            LF.setTargetPosition(LF.getCurrentPosition() + 900);


            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LB.setPower(.25);
            RB.setPower(.25);
            LF.setPower(.25);
            RF.setPower(.25);


            while (LB.isBusy() & RF.isBusy() & RB.isBusy() & LF.isBusy()) {
            }


            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);
            BlueAuto.setPosition(0);


            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            v_sensor_ods.getLightDetected();



            telemetry.addData("White Line", v_sensor_ods.getLightDetected ());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("RF: ", RF.getCurrentPosition());
            telemetry.addData("RB: ", RB.getCurrentPosition());
            telemetry.addData("LB: ", LB.getCurrentPosition());
            telemetry.addData("LF: ", LF.getCurrentPosition());
            telemetry.update();
v_sensor_ods.getLightDetected();
            sleep(300);



while (!a_ods_white_tape_detected() & opModeIsActive()) {
  v_sensor_ods.getLightDetected();
    LB.setPower(-.25);
    RB.setPower(-.25);
    LF.setPower(-.25);
    RF.setPower(-.25);
}

            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);





            telemetry.addData("White Line", v_sensor_ods.getLightDetected ());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("RF: ", RF.getCurrentPosition());
            telemetry.addData("RB: ", RB.getCurrentPosition());
            telemetry.addData("LB: ", LB.getCurrentPosition());
            telemetry.addData("LF: ", LF.getCurrentPosition());
            telemetry.update();

            sleep(300);


            rangeAcache = rangeAreader.read(0x04, 2);  //Read 2 bytes starting at 0x04
            rangeCcache = rangeCreader.read(0x04, 2);
            rangeBcache = rangeBreader.read(0x04, 2);


            int ERUS = rangeCcache[0] & 0xFF;   //Ultrasonic value is at index 0. & 0xFF creates a value between 0 and 255 instead of -127 to 128
            int ELUS = rangeAcache[0] & 0xFF;
            int ELODS = rangeAcache[1] & 0xFF;

            int EFUS = rangeBcache[0] & 0xFF;
            int EFODS = rangeBcache[1] & 0xFF;
            sleep(150);

            telemetry.addData("ERUS", ERUS);
            telemetry.update();
            if (ERUS >= 16 & ERUS <= 21) {

                LB.setPower(.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(.25);
                sleep(300);
                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);

            }

            else if (ERUS >= 22 ) {

                LB.setPower(.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(.25);
                sleep(600);
                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);

            }

            else {LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);}

            telemetry.addData("White Line", v_sensor_ods.getLightDetected ());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("RF: ", RF.getCurrentPosition());
            telemetry.addData("RB: ", RB.getCurrentPosition());
            telemetry.addData("LB: ", LB.getCurrentPosition());
            telemetry.addData("LF: ", LF.getCurrentPosition());
            telemetry.update();
            sleep(100);


            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
            if (sensorRGB.red() > 190 & sensorRGB.red() > sensorRGB.blue()) {

                Hitter.setPosition(1);
                sleep(200);
                LB.setPower(.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(.25);

                sleep(1350);

                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);
                sleep(200);

                LB.setPower(-.25);
                RB.setPower(.25);
                LF.setPower(.25);
                RF.setPower(-.25);
                sleep(300);
                Hitter.setPosition(.5);
                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);
                sleep(300);
                Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
                if (sensorRGB.red() > 190 & sensorRGB.red() > sensorRGB.blue()){
                    //CHECKER

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(4000);

                    Hitter.setPosition(1);
                    sleep(200);

                    LB.setPower(.25);
                    RB.setPower(-.25);
                    LF.setPower(-.25);
                    RF.setPower(.25);
                    sleep(1250);

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(200);

                    LB.setPower(-.3);
                    RB.setPower(.3);
                    LF.setPower(.3);
                    RF.setPower(-.3);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);
                }


                else {

                    //CHECKER

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(250);

                    LB.setPower(-.3);
                    RB.setPower(.3);
                    LF.setPower(.3);
                    RF.setPower(-.3);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);



                    Hitter.setPosition(.5);
                }          }
            else if (sensorRGB.blue() > 190 & sensorRGB.blue() > sensorRGB.red()){
                Hitter.setPosition(-1);
                sleep(200);
                LB.setPower(.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(.25);

                sleep(1350);

                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);
                sleep(200);

                LB.setPower(-.25);
                RB.setPower(.25);
                LF.setPower(.25);
                RF.setPower(-.25);
                sleep(300);
                Hitter.setPosition(.5);
                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);
                sleep(300);
                Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
                if (sensorRGB.red() > 190 & sensorRGB.red() > sensorRGB.blue()){
                   //CHECKER

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(4000);

                    Hitter.setPosition(1);
                    sleep(200);

                    LB.setPower(.25);
                    RB.setPower(-.25);
                    LF.setPower(-.25);
                    RF.setPower(.25);
                    sleep(1250);

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(200);

                    LB.setPower(-.3);
                    RB.setPower(.3);
                    LF.setPower(.3);
                    RF.setPower(-.3);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);
                }


                else {
                    //CHECKER



                    LB.setPower(-.2);
                    RB.setPower(.2);
                    LF.setPower(.2);
                    RF.setPower(-.2);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);


                    Hitter.setPosition(.5);
                }
            }




            v_sensor_ods.getLightDetected();
            sleep(100);

            while (!a_ods_white_tape_detected() & opModeIsActive()) {
                v_sensor_ods.getLightDetected();
                LB.setPower(-.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(-.25);
            }
            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);
           /* sleep(100);
           LB.setPower(.15);
            RB.setPower(-.15);
            LF.setPower(.15);
            RF.setPower(-.15);
            sleep(500); */
            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);



            telemetry.addData("White Line", v_sensor_ods.getLightDetected ());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("RF: ", RF.getCurrentPosition());
            telemetry.addData("RB: ", RB.getCurrentPosition());
            telemetry.addData("LB: ", LB.getCurrentPosition());
            telemetry.addData("LF: ", LF.getCurrentPosition());
            telemetry.update();

            sleep(100);
            rangeAcache = rangeAreader.read(0x04, 2);  //Read 2 bytes starting at 0x04
            rangeCcache = rangeCreader.read(0x04, 2);
            rangeBcache = rangeBreader.read(0x04, 2);


            int RUS = rangeCcache[0] & 0xFF;   //Ultrasonic value is at index 0. & 0xFF creates a value between 0 and 255 instead of -127 to 128
            int LUS = rangeAcache[0] & 0xFF;
            int RODS = rangeCcache[1] & 0xFF;
            int LODS = rangeAcache[1] & 0xFF;

            int FUS = rangeBcache[0] & 0xFF;
            int FODS = rangeBcache[1] & 0xFF;


            telemetry.addData("RUS", RUS);
            telemetry.update();
            sleep(150);

            if (RUS >= 16 & RUS <= 21) {

                LB.setPower(.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(.25);
                sleep(300);
                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);

            }

           else if (RUS >= 22 ) {

                LB.setPower(.35);
                RB.setPower(-.35);
                LF.setPower(-.35);
                RF.setPower(.35);
                sleep(600);
                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);

            }

            else {LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);}

            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);
sleep(100);
            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
            sleep(300);
            telemetry.addData("White Line", v_sensor_ods.getLightDetected ());
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("RF: ", RF.getCurrentPosition());
            telemetry.addData("RB: ", RB.getCurrentPosition());
            telemetry.addData("LB: ", LB.getCurrentPosition());
            telemetry.addData("LF: ", LF.getCurrentPosition());
            telemetry.update();
            Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
            sleep(200);
            if (sensorRGB.red() > 200 & sensorRGB.red() > sensorRGB.blue()) {

                Hitter.setPosition(1);
                sleep(200);
                LB.setPower(.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(.25);
                sleep(1350);

                LB.setPower(-.3);
                RB.setPower(.3);
                LF.setPower(.3);
                RF.setPower(-.3);
                Hitter.setPosition(.5);
                sleep(300);
                Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
                if (sensorRGB.red() > 190 & sensorRGB.red() > sensorRGB.blue()){
                    //CHECKER

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(4000);

                    Hitter.setPosition(1);
                    sleep(200);

                    LB.setPower(.25);
                    RB.setPower(-.25);
                    LF.setPower(-.25);
                    RF.setPower(.25);
                    sleep(1250);

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(200);

                    LB.setPower(-.3);
                    RB.setPower(.3);
                    LF.setPower(.3);
                    RF.setPower(-.3);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);
                }


                else {

                    //CHECKER

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(250);

                    LB.setPower(-.3);
                    RB.setPower(.3);
                    LF.setPower(.3);
                    RF.setPower(-.3);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);

                    Hitter.setPosition(.5);
                }


                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);
                sleep(250);




                Hitter.setPosition(.5);
            }
            else if (sensorRGB.blue() > 190 & sensorRGB.blue() > sensorRGB.red()){
                Hitter.setPosition(-1);
                sleep(200);
                LB.setPower(.25);
                RB.setPower(-.25);
                LF.setPower(-.25);
                RF.setPower(.25);

                sleep(1350);

                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);
                sleep(200);

                LB.setPower(-.3);
                RB.setPower(.3);
                LF.setPower(.3);
                RF.setPower(-.3);
                sleep(300);
                Hitter.setPosition(.5);
                LB.setPower(0);
                RB.setPower(0);
                LF.setPower(0);
                RF.setPower(0);
                sleep(300);
                Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
                if (sensorRGB.red() > 190 & sensorRGB.red() > sensorRGB.blue()){
                    //CHECKER

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(4000);

                    Hitter.setPosition(1);
                    sleep(200);

                    LB.setPower(.25);
                    RB.setPower(-.25);
                    LF.setPower(-.25);
                    RF.setPower(.25);
                    sleep(1250);

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);
                    sleep(200);

                    LB.setPower(-.3);
                    RB.setPower(.3);
                    LF.setPower(.3);
                    RF.setPower(-.3);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);
                }


                else {
                    //CHECKER



                    LB.setPower(-.3);
                    RB.setPower(.3);
                    LF.setPower(.3);
                    RF.setPower(-.3);
                    sleep(300);

                    LB.setPower(-.3);
                    RB.setPower(-.3);
                    LF.setPower(-.3);
                    RF.setPower(-.3);
                    sleep(750);

                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);

                    Hitter.setPosition(.5);
                }
            }
            LF.setTargetPosition(LF.getCurrentPosition() + 3450);

            RB.setTargetPosition(RB.getCurrentPosition() + 3450);

            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            RF.setPower(0);
            LB.setPower(0);
            LF.setPower(.75);
            RB.setPower(.75);


            while (RB.isBusy() & LF.isBusy()) {
            }


            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);

            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LB.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            RF.setPower(0);
            sleep(50000000);





        while (opModeIsActive()) {

            v_sensor_ods.getLightDetected ();






            LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            {

               /* if (LUS <= 18) {
                    LB.setPower(.15);
                    RB.setPower(-.15);
                    LF.setPower(-.15);
                    RF.setPower(.15);

                } */
                if (FUS <= 30) {
                    LB.setPower(0);
                    RB.setPower(0);
                    LF.setPower(0);
                    RF.setPower(0);

                }

                else {LB.setPower(-.2);
                    RB.setPower(-.2);
                    LF.setPower(-.2);
                    RF.setPower(-.2);}
            }


        }
        }
    }
}
