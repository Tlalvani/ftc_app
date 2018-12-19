package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="RR2GyroTest", group="Test")  // @Autonomous(...) is the other common choice
public class RR2GyroTest extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();
   //         robot.LiftWithEncoders();
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




            imu(-45);
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

