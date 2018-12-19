package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="RR2FarSideAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2FarSideAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();
DetectMineral();

   //         robot.LiftWithEncoders();
        waitForStart();

        if(opModeIsActive()) {
            DriveTargetPosition(600,600,600,600);
            Drive(.2,.2);
            DrivebaseBusy();
            Drive(0,0);
            sleep(1000);

            imu(90);

            DriveTargetPosition(2100,2100,2100,2100);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);

            imu(133);

            DriveTargetPosition(1800,1800,1800,1800);
            Drive(.4,.4);
            DrivebaseBusy();
            Drive(0,0);

            robot.Intake.setPower(-1);
            sleep(2000);
            robot.Intake.setPower(0);

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

