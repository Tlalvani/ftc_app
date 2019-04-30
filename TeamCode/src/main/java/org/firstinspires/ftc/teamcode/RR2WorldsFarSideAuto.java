package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="RR2FarSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2WorldsFarSideAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();

   //         robot.LiftWithEncoders();
        waitForStart();
        int sample = DetectMineral();

        while(opModeIsActive()) {

            Unlatch();
        /*    DriveTargetPosition(450,450,450,450);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0); */

            DriveTargetPosition(500,500,500,500);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            ZeroLift();

            FarSample(sample);

            DriveTargetPosition(-600,-600,-600,-600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

          imu(0);
          RightAngleTurnTargetPosition(1800,1800);
            Drive(0,.2);
            RightDrivebaseBusy();
            Drive(0,0);

            DriveTargetPosition(1000,1000,1000,1000);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            RightAngleTurnTargetPosition(900,900);
            Drive(0,.2);
            RightDrivebaseBusy();
            Drive(0,0);

            DriveTargetPosition(200,-200,-200,200);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);




            robot.ExtendLiftPosition(1500);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);
            robot.IntakeFlipper.setPosition(robot.intakedown);
            robot.Intake.setPower(-1);

            sleep(1000);

            robot.Intake.setPower(0);
            robot.ExtendLiftPosition(0);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);
            robot.IntakeFlipper.setPosition(robot.intakedown);

            DriveTargetPosition(-100,100,100,-100);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);


            DriveTargetPosition(-1600,-1600,-1600,-1600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);


            sleep(1000000);
        }

    }
}

