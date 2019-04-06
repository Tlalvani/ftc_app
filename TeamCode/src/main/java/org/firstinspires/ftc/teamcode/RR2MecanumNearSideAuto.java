package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Deprecated
@Autonomous(name="RR2MecanumNearSideAuto", group="Test")  // @Autonomous(...) is the other common choice
abstract public class RR2MecanumNearSideAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
        Servo Intake2;
initSensors();
DetectMineral();


        waitForStart();

        int sample = DetectMineral();
        while(opModeIsActive()) {




            Unlatch();

            DriveTargetPosition(50,50,50,50);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);

            ZeroLift();

            robot.IntakeFlipper.setPosition(robot.intakedown);
            robot.ExtendLiftPosition(700);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);
            robot.IntakeFlipper.setPosition(.75);

            DriveTargetPosition(1000,1000,1000,1000);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            robot.IntakeFlipper.setPosition(robot.intakedown);
            robot.Intake.setPower(.75);
          //  robot.Intake2.setPower(.75);
            sleep(2000);
            robot.IntakeFlipper.setPosition(.75);
            sleep(2000);

            DriveTargetPosition(-1050,-1050,-1050,-1050);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);


            robot.IntakeFlipper.setPosition(robot.intakedown);
            robot.Intake.setPower(0);
          //  robot.Intake2.setPower(0);
            sleep(1000);
            robot.ExtendLiftPosition(250);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);

            DriveTargetPosition(250,250,250,250);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);

           Sample(sample); //Here3
            imu(0);

            DriveTargetPosition(-350,-350,-350,-350);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            RightAngleTurnTargetPosition(1800,1800);
            Drive(0, .2);
            RightDrivebaseBusy();
            Drive(0,0);

            DriveTargetPosition(1350,1350,1350,1350);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);


            RightAngleTurnTargetPosition(400,400);
            Drive(0,.2);
            RightDrivebaseBusy();
            Drive(0,0);


            DriveTargetPosition(100,100,100,100);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);


/*
            DriveTargetPosition(1000,1000,1000,1000);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            DepositTeamMarker();

            DriveTargetPosition(-1600,-1600,-1600,-1600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            robot.DeployArm(); */
            sleep(100000);

/*
   DriveTargetPosition(450,450,450,450);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            ZeroLift();

         //   DepositTeamMarker();

            Sample(sample);
            imu(79);
            DriveTargetPosition(1700,1700,1700,1700);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            imu(-40);

            DriveTargetPosition(-500,500,500,-500);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            DriveTargetPosition(1000,1000,1000,1000);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            DepositTeamMarker();

            DriveTargetPosition(-1600,-1600,-1600,-1600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            robot.DeployArm();


*/

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

