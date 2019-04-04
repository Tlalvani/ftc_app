package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Deprecated
@Disabled
@Autonomous(name="IMUCalB", group="Test")  // @Autonomous(...) is the other common choice
abstract public class RR2MecanumIMUCalibrate extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();



        waitForStart();
        robot.DrivebaseWithEncoders();

        while(opModeIsActive()) {

            imu(90);
        }

        }

    }


