package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="TestAuto", group="Test")  // @Autonomous(...) is the other common choice
public class TestAutoExample extends TestAutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
initSensors();

LiftWithEncoders();
        waitForStart();

        if(opModeIsActive()) {

            LiftTargetPosition(robot.LiftHang);
            Lift(1);
            DriveTargetPosition(1120,1120,1120,1120);
            DrivebaseBusy();
            Drive(1,1);
        }

    }
}

