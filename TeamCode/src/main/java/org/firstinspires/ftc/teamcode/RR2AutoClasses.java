package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

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
abstract public class RR2AutoClasses extends LinearOpMode {


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    RR2HardwareDrivebase robot = new RR2HardwareDrivebase();

    /* Constructor */
    public RR2AutoClasses() {

    }


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AS+iypf/////AAABmVA+Shgi7EdsqH2d8iljwuh6/bNz0HoePOJW6LbOge+udoIv+21ZVTVE7Rbb6pcLmDZS0fP37WoY3OfgCQZBXOdLEpSXhJMHZuyD1V42qLPR9R+FarnuNWS21fr+EhRFPNatPM+riV2eQCS0WroErVmpvwDJUxQCI5Uk9ekS3TPW+9oEf/7V1OUr29wH6lMAwx0SOwTGW37/eaYWOpYpJRPJt9AtLrKupxQc6n9p87o/7oCBifNw0DLLb4L8WN3FaqhHuZ7iWYH3f1D7qcKetbfPW6oOEcTPhIhkERNOn+qiCj8zqJ626Bp1YLtOEEAdP+m25g7D3sJiwW32g/ekIFF8xNTNXwS830fKUHjB2z+D";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;


//FUNCTIONS

    public void initSensors() {
        robot.init(hardwareMap);



        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters imuparameters = new BNO055IMU.Parameters();
        imuparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuparameters.loggingEnabled = true;
        imuparameters.loggingTag = "IMU";
        imuparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

    }

    //DRIVE FUNCTIONS
    public void Drive(double leftpower, double rightpower) {

        robot.LF.setPower(leftpower);
        robot.RF.setPower(rightpower);
        robot.LB.setPower(leftpower);
        robot.RB.setPower(rightpower);
    }

    public void DriveTargetPosition(int LFpower, int LBpower, int RFpower, int RBpower) {
        robot.LB.setTargetPosition(robot.LB.getCurrentPosition() + LBpower);
        robot.RB.setTargetPosition(robot.RB.getCurrentPosition() + RBpower);
        robot.LF.setTargetPosition(robot.LF.getCurrentPosition() + LFpower);
        robot.RF.setTargetPosition(robot.RF.getCurrentPosition() + RFpower);

        robot.LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void DrivebaseBusy() {
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy()) {
        }
    }


    public void DrivebaseWithEncoders() {
        robot.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void BusyLift() {
        while (robot.Lift1.isBusy() & robot.Lift2.isBusy() & robot.Lift3.isBusy()) {
        }
    }


    public void Unlatch(){
        robot.latchOff();
        sleep(1500);
        robot.LiftPosition(robot.LiftHang);
        robot.Lift(.5);
        BusyLift();
        robot.Lift(0);
        sleep(500);
        robot.Hook.setPosition(0);
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void imu(double degrees) {
        robot.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();
        double leftSpeedt; //Power to feed the motors
        double rightSpeedt;
        double Z = angles.firstAngle;

        while (angles.firstAngle < (degrees - robot.AngleTolerance) || angles.firstAngle > (degrees + robot.AngleTolerance) & opModeIsActive()) {
            // position turns left, think of a x,y coordinate system
            telemetry.update();

            leftSpeedt = -((degrees - angles.firstAngle) / robot.divisorforimu);  //Calculate speed for each side
            rightSpeedt = ((degrees - angles.firstAngle) / robot.divisorforimu);  //See Gyro Straight video for detailed explanation

            telemetry.update();
            if (leftSpeedt > 0) {
                leftSpeedt = Range.clip(leftSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }


            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }
            telemetry.update();
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            telemetry.update();

            idle();
        }
        Drive(0, 0);

    }


    public void oneeightyimu(double degrees) {
        telemetry.update();
        double leftSpeedt; //Power to feed the motors
        double rightSpeedt;

        double Z = angles.firstAngle;


        while (robot.currentangle < (180 - robot.AngleTolerance) || robot.currentangle > (180 + robot.AngleTolerance) & opModeIsActive()) {
            // positie turns left, think of a x,y coordinate system
            telemetry.update();

            if (angles.firstAngle < 0) {
                robot.currentangle = angles.firstAngle + 360;
            } else {
                robot.currentangle = angles.firstAngle;
            }

            leftSpeedt = -((180 - robot.currentangle) / robot.divisorforimu);  //Calculate speed for each side
            rightSpeedt = ((180 - robot.currentangle) / robot.divisorforimu);  //See Gyro Straight video for detailed explanation

            telemetry.update();
            if (leftSpeedt > 0) {
                leftSpeedt = Range.clip(leftSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }


            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            telemetry.update();

            idle();
        }
        Drive(0, 0);

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    public int DetectMineral() {
        int value = 3; //Center by Default
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left"); //1
                            telemetry.addData("Value:", value);
                            value = 1;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right"); //2
                            value = 2;
                            telemetry.addData("Value:", value);
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center"); //3
                            value = 3;
                            telemetry.addData("Value:", value);
                        }
                    }
                }
                telemetry.update();

            }
        }
        return value;
    }
}





