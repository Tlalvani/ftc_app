package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="RR2NearSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2MecanumNearSideAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();


        waitForStart();

        int sample = DetectMineral();
        while(opModeIsActive()) {




            Unlatch();
            DriveTargetPosition(400,400,400,400);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            ZeroLift();

         //   DepositTeamMarker();

            Sample(sample);
            imu(79);
            DriveTargetPosition(1600,1600,1600,1600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            imu(-40);

            DriveTargetPosition(-500,500,500,-500);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            DriveTargetPosition(800,800,800,800);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            DepositTeamMarker();

            DriveTargetPosition(-1600,-1600,-1600,-1600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            robot.DeployArm();





         //Sample(sample);

     /*       imu(65);
            DriveTargetPosition(2350,2350,2350,2350);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            sleep(1000);
            imu(-40);
          DriveTargetPosition(1900,1900,1900,1900);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);
            robot.Intake.setPower(-1);
            sleep(2000);
            robot.Intake.setPower(0);
            imu(-44);
            DriveTargetPosition(-3400,-3400,-3400,-3400);
            Drive(.65,.65);
            DrivebaseBusy();
            Drive(0,0);
            robot.DeployArm();*/
            sleep(100000);

            /*
            DriveTargetPosition(-500,-500,500,500);
            Drive(.75,.75);
            DrivebaseBusy();
            Drive(0,0);
            DriveTargetPosition(-3600,-3600,-3600,-3600);
            Drive(.65,.65);
            DrivebaseBusy();
            Drive(0,0);
            robot.DeployArm();
            sleep(100000);
*/

        }

    }
}

