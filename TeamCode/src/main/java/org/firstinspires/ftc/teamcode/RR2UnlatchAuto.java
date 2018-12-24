package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="RR2UnlatchAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2UnlatchAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();

   //         robot.LiftWithEncoders();
        waitForStart();

        while(opModeIsActive()) {
/*
            robot.LiftPosition(robot.LiftHang);
            robot.Lift(1);
            */
            robot.LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.latchOff();
        sleep(3000);

       robot.LiftPosition(robot.LiftHang);
       robot.Lift(.5);
       BusyLift();
       robot.Lift(0);

        sleep(1000);
            robot.Hook.setPosition(0);
            //imu(90);
  /*          DriveTargetPosition(3150,3150,3150,3150);
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
*/

        }

    }
}

