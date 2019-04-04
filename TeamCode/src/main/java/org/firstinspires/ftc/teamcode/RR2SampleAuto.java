package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Deprecated
@Autonomous(name="RR2SampleAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2SampleAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
int sample = 0;


        initSensors();


        waitForStart();

        while(opModeIsActive()) {
            sample = DetectMineral();
            Unlatch();
            DriveFromLander();

            if(sample==1){
                imu(60);
                telemetry.addData("value:", sample);
                telemetry.update();


            }
            else if (sample==2)
            {
                imu(-60);
                telemetry.addData("value:", sample);
                telemetry.update();


            }
            else{
                imu(0);
                telemetry.addData("value:", sample);
                telemetry.update();

            }

            DriveTargetPosition(1000,1000,1000,1000);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);
            DriveTargetPosition(-1000,-1000,-1000,-1000);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);
            sleep(100000000);
                   }

    }
}

