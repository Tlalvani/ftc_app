package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="RR2NearSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2WorldsNearSideAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {

initSensors();
DetectMineral();


        waitForStart();

        int sample = DetectMineral();
        while(opModeIsActive()) {


            robot.RetractArm();
            robot.Bucket.setPosition(robot.BucketHome);
            robot.SortLatch.setPosition(robot.SortLatchClose);

            Unlatch();


            DriveTargetPosition(600,600,600,600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            ZeroLift();

            robot.ExtendLiftPosition(150);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);



            robot.ExtendLiftPosition(1200);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);
            sleep(1000);
            robot.IntakeFlipper.setPosition(robot.intakedown);
            robot.Intake.setPower(-1);
            sleep(2000);
            robot.Intake.setPower(0);
            robot.IntakeFlipper.setPosition(robot.intakeup);
            robot.ExtendLiftPosition(0);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);


            DriveTargetPosition(-50,-50,-50,-50);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            Sample(sample);

            sleep(1000);
robot.Intake.setPower(0);

            robot.AutoLiftingUp = true;

            while(robot.AutoLiftingUp){
                robot.autoLiftUp();
            }

            DriveTargetPosition(-300,-300,-300,-300);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            robot.SortLatch.setPosition(robot.SortLatchOpen);
            sleep(1000);
            DriveTargetPosition(300,300,300,300);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);

            robot.ExtendLiftPosition(150);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);

            robot.AutoLiftingDown = true;
            while(robot.AutoLiftingDown){
                robot.autoLiftDown();
            }


            robot.ExtendLiftPosition(0);
            robot.IntakeLift.setPower(1);
            IntakeLiftBusy();
            robot.IntakeLift.setPower(0);

            DriveTargetPosition(-2400,2400,2400,-2400);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
/*
            DriveTargetPosition(-1050,-1050,-1050,-1050);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);


            robot.IntakeFlipper.setPosition(robot.intakedown);
            robot.Intake.setPower(0);
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
*/
            sleep(100000);




        }

    }
}

