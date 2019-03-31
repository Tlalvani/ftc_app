package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="RR2ExtendingLiftTest", group="Test")  // @Autonomous(...) is the other common choice
public class RR2MecanumExtendingLifttest extends RR2AutoClasses
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


