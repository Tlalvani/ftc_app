package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Deprecated
@Disabled
@Autonomous(name="RR2DoubleSampleFarSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2FarSideDoubleSampleAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();

   //         robot.LiftWithEncoders();
        waitForStart();
        int sample = DetectMineral();
        int value = 0;

        while(opModeIsActive()) {

            Unlatch();
          DriveFromLander();
          ZeroLift();

          value = sample;
          FarSample(sample);

            imu(90);

            DriveTargetPosition(2300,2300,2300,2300);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);

            imu(130);

            if(value == 1) {
                DriveTargetPosition(1800, 1800, 1800, 1800);
                Drive(.4, .4);
                DrivebaseBusy();
                Drive(0, 0);

                robot.Intake.setPower(-1);
                sleep(2000);
                robot.Intake.setPower(0);

                DriveTargetPosition(-500,-500,500,500);
                Drive(.75,.75);
                DrivebaseBusy();
                Drive(0,0);

                DriveTargetPosition(900,900,900,900);
                Drive(.4,.4);
                DrivebaseBusy();
                Drive(0,0);
                DriveTargetPosition(-900,-900,-900,-900);
                Drive(.4,.4);
                DrivebaseBusy();
                Drive(0,0);

                DriveTargetPosition(500,500,-500,-500);
                Drive(.75,.75);
                DrivebaseBusy();
                Drive(0,0);

            }

            else if(value == 2) {
                DriveTargetPosition(800, 800, 800, 800);
                Drive(.4, .4);
                DrivebaseBusy();
                Drive(0, 0);

                DriveTargetPosition(-750,-750,750,750);
                Drive(.75,.75);
                DrivebaseBusy();
                Drive(0,0);

                DriveTargetPosition(900,900,900,900);
                Drive(.4,.4);
                DrivebaseBusy();
                Drive(0,0);
                DriveTargetPosition(-900,-900,-900,-900);
                Drive(.4,.4);
                DrivebaseBusy();
                Drive(0,0);

                DriveTargetPosition(750,750,-750,-750);
                Drive(.75,.75);
                DrivebaseBusy();
                Drive(0,0);
                imu(130);

                DriveTargetPosition(1000, 1000, 1000, 1000);
                Drive(.4, .4);
                DrivebaseBusy();
                Drive(0, 0);

                robot.Intake.setPower(-1);
                sleep(2000);
                robot.Intake.setPower(0);
            }

            else {
                DriveTargetPosition(1800, 1800, 1800, 1800);
                Drive(.4, .4);
                DrivebaseBusy();
                Drive(0, 0);

                robot.Intake.setPower(-1);
                sleep(2000);
                robot.Intake.setPower(0);

                DriveTargetPosition(-1000,-1000,1000,1000);
                Drive(.75,.75);
                DrivebaseBusy();
                Drive(0,0);

                DriveTargetPosition(1300,1300,1300,1300);
                Drive(.4,.4);
                DrivebaseBusy();
                Drive(0,0);
                DriveTargetPosition(-1300,-1300,-1300,-1300);
                Drive(.4,.4);
                DrivebaseBusy();
                Drive(0,0);

                DriveTargetPosition(1000,1000,-1000,-1000);
                Drive(.75,.75);
                DrivebaseBusy();
                Drive(0,0);

            }


            imu(133);

            DriveTargetPosition(-3100,-3100,-3100,-3100);
            Drive(.65,.65);
            DrivebaseBusy();
            Drive(0,0);

            robot.DeployArm();
            sleep(100000);
/*
            robot.LiftPosition(robot.LiftHang);
            robot.Lift(1);
            DriveTargetPosition(1120,1120,1120,1120);
            DrivebaseBusy();
            Drive(1,1); */
/*
            DriveTargetPosition(500,500,500,500);
            DriveTargetPosition(-1920,-1920,1920,1920);
            Drive(.75,.75);
            DrivebaseBusy();
            Drive(0,0);
            DriveTargetPosition(-2500,-2500,-2500,-2500);
            Drive(.65,.65);
            DrivebaseBusy();
            Drive(0,0);
            robot.DeployArm();
            sleep(100000);
            */
        }

    }
}

