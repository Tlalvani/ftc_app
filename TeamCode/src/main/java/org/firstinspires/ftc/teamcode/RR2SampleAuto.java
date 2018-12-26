package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="RR2SampleAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2SampleAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {


        initSensors();
        DetectMineral();

        waitForStart();
        if (tfod != null) {
            tfod.activate();
        }
        while(opModeIsActive()) {

            if(DetectMineral()==1){
                imu(15);
                telemetry.addData("value:", DetectMineral());

            }
            else if (DetectMineral()==2)
            {
                imu(-15);
                telemetry.addData("value:", DetectMineral());

            }
            else{
                imu(0);
                telemetry.addData("value:", DetectMineral());
            }

                   }

    }
}

