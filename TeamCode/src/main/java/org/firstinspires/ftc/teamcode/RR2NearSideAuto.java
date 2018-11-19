package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="RR2NearSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2NearSideAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();

   //         robot.LiftWithEncoders();
        waitForStart();

        if(opModeIsActive()) {
/*
            robot.LiftPosition(robot.LiftHang);
            robot.Lift(1);
            */

            DriveTargetPosition(3150,3150,3150,3150);
            Drive(.65,.65);
            DrivebaseBusy();
            Drive(0,0);
            robot.Intake.setPower(-1);
            sleep(2000);
            robot.Intake.setPower(0);
            //imu(275);
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


        }

    }
}

