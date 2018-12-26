package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="RR2UnlatchAuto", group="Test")  // @Autonomous(...) is the other common choice
public class RR2UnlatchAuto extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {

        initSensors();
        robot.latchOn();
        waitForStart();

        while(opModeIsActive()) {

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
        sleep(1000000);

        }

    }
}

