package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
abstract public class RR2AutoClasses extends LinearOpMode
{



    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;





    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    RR2HardwareDrivebase        robot   = new RR2HardwareDrivebase();
    /* Constructor */
    public RR2AutoClasses(){

    }




//FUNCTIONS

public void initSensors() {
        robot.init(hardwareMap);

    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = true;
    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    // Set up our telemetry dashboard
    composeTelemetry();

}

//DRIVE FUNCTIONS
    public void Drive(double leftpower, double rightpower) {

        robot.LF.setPower(leftpower);
        robot.RF.setPower(rightpower);
        robot.LB.setPower(leftpower);
        robot.RB.setPower(rightpower);
    }

    public void DriveTargetPosition(int LFpower, int LBpower, int RFpower, int RBpower){
        robot.LB.setTargetPosition(robot.LB.getCurrentPosition() + LBpower);
        robot.RB.setTargetPosition(robot.RB.getCurrentPosition() + RBpower);
        robot.LF.setTargetPosition(robot.LF.getCurrentPosition() + LFpower);
        robot.RF.setTargetPosition(robot.RF.getCurrentPosition() + RFpower);

        robot.LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void DrivebaseBusy(){
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy()) {
    }
    }


    public void DrivebaseWithEncoders(){
        robot.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }




    public void BusyLift(){
        while (robot.Lift1.isBusy() & robot.Lift2.isBusy() & robot.Lift3.isBusy()) {
        }
    }






    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void imu(double degrees){
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
                leftSpeedt = Range.clip(leftSpeedt, robot.minspeedimu, robot.maxspeedimu); }
            else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu)); }



            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, robot.minspeedimu, robot.maxspeedimu); }
            else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu)); }
            telemetry.update();
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            telemetry.update();

            idle();
        }
        Drive(0,0);

    }


    public void oneeightyimu(double degrees){
        telemetry.update();
        double leftSpeedt; //Power to feed the motors
        double rightSpeedt;

        double Z = angles.firstAngle;


        while (robot.currentangle < (180 - robot.AngleTolerance) || robot.currentangle > (180 + robot.AngleTolerance) & opModeIsActive()) {
            // positie turns left, think of a x,y coordinate system
            telemetry.update();

            if(angles.firstAngle < 0) {
                robot.currentangle = angles.firstAngle + 360;
            }

            else {
                robot.currentangle = angles.firstAngle;
            }

            leftSpeedt = -((180- robot.currentangle) / robot.divisorforimu);  //Calculate speed for each side
            rightSpeedt = ((180 - robot.currentangle) / robot.divisorforimu);  //See Gyro Straight video for detailed explanation

            telemetry.update();
            if (leftSpeedt > 0) {
                leftSpeedt = Range.clip(leftSpeedt, robot.minspeedimu, robot.maxspeedimu); }
            else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu)); }



            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, robot.minspeedimu, robot.maxspeedimu); }
            else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu)); }
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            telemetry.update();

            idle();
        }
        Drive(0,0);

    }
}






