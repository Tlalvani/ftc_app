package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="RR2NearSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2NearSideAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();
robot.latchOn();

        waitForStart();

        if(opModeIsActive()) {
/*
            robot.LiftPosition(robot.LiftHang);
            robot.Lift(1);
            */
            robot.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//robot.latchOff();
//robot.hangLiftUp();
            //imu(90);



            DriveTargetPosition(200,200,200,200);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            sleep(1000);
            imu(65);
            DriveTargetPosition(2000,2000,2000,2000);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            sleep(1000);
            imu(-39);
          DriveTargetPosition(1800,1800,1800,1800);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);
            robot.Intake.setPower(-1);
            sleep(2000);
            robot.Intake.setPower(0);
            DriveTargetPosition(-3500,-3500,-3500,-3500);
            Drive(.65,.65);
            DrivebaseBusy();
            Drive(0,0);
            robot.DeployArm();
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

