package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="RR2GyroTest", group="Test")  // @Autonomous(...) is the other common choice
public class RR2GyroTest extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();
   //         robot.LiftWithEncoders();
        waitForStart();

        while(opModeIsActive()) {

            imu(0);


        }

    }
}

