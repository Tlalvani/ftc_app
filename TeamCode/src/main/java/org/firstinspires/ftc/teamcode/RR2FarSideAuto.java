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

   //         robot.LiftWithEncoders();
        waitForStart();

        if(opModeIsActive()) {
/*
            robot.LiftPosition(robot.LiftHang);
            robot.Lift(1);
            DriveTargetPosition(1120,1120,1120,1120);
            DrivebaseBusy();
            Drive(1,1); */

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
        }

    }
}

