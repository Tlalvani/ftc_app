package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Deprecated
@Disabled
@Autonomous(name="RR2MecanumExtendingLiftTest", group="Test")  // @Autonomous(...) is the other common choice
abstract public class RR2MecanumExtendingLifttest extends RR2AutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

   //         robot.LiftWithEncoders();
        waitForStart();

robot.IntakeFlipper.setPosition(.75);

sleep(1000000);
        }

    }


