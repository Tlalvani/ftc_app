package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="RR2FarSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2MecanumFarSideAuto extends RR2AutoClasses
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
            DriveTargetPosition(400,400,400,400);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            ZeroLift();

          FarSample(sample);
            imu(90);

            DriveTargetPosition(2300,2300,2300,2300);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);

            imu(130);

            DriveTargetPosition(1800,1800,1800,1800);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);

            robot.Intake.setPower(-1);
            sleep(2000);
            robot.Intake.setPower(0);

            imu(133);

            DriveTargetPosition(-3200,-3200,-3200,-3200);
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

