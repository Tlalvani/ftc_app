/*
We can win
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "RRTeleop", group = "RR")  // @Autonomous(...) is the other common choice

public class RRSupersTeleop extends OpMode {

    RRHardwareDrivebase robot = new RRHardwareDrivebase();
    private ElapsedTime runtime = new ElapsedTime();
    /* Declare OpMode members. */
    boolean teleop = true;
    boolean endgame = false;
    boolean balance = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("runtime", runtime.seconds());
        robot.JewelArm.setPosition(robot.JewelArmHome);
        //  robot.RelicThing.setPosition(robot.relicThingPosition);


        if (gamepad1.dpad_left) {
            teleop = true;
            endgame = false;
            balance = false;
        } else if (gamepad1.dpad_right) {
            endgame = true;
            teleop = false;
            balance = false;
        } else if (gamepad1.dpad_up) {
            endgame = false;
            teleop = false;
            balance = true;
        }


//TELEOP
        if (teleop & !endgame & !balance) {

            DriveControls(1);
            float Lift = -gamepad2.right_trigger + gamepad2.left_trigger + gamepad1.right_trigger - gamepad1.left_trigger;
            float RelicLift = -gamepad2.left_stick_y;

            Lift = Range.clip(Lift, -1, 1);


            if (gamepad1.right_bumper) {
                robot.RunIntake(); }
            else if (gamepad1.a) {
                robot.ReverseIntake();
            }
        /*    else if (gamepad1.dpad_down) {
                robot.LeftIntake.setPower(-1);
            } */
            else {
                robot.StopIntake();
            }

            robot.Lift1.setPower(Lift);
            robot.Lift2.setPower(Lift);

            robot.RelicLift1.setPower(RelicLift);
            robot.RelicLift2.setPower(RelicLift);

            robot.GetLiftPosition();

            if (robot.TopLeftClaw.getPosition() != robot.TopOneBlockLeft & (robot.TopRightClaw.getPosition() != robot.TopOneBlockRight) ) {
                if(gamepad1.left_stick_button){
                    robot.TopLeftClaw.setPosition(robot.BottomOpenLeft);
                }

                else if ( (robot.TopRightClaw.getPosition() == robot.TopStraightRight) ){
                 robot.TopLeftClaw.setPosition(robot.TopStraightLeft);
                }
            }


// Lift is fully up
            if (robot.CurrentLiftPosition > robot.LiftThreshold) {

                if (robot.LeftClaw.getPosition() >= robot.BottomShutdownLeft & (robot.RightClaw.getPosition() <= robot.BottomShutdownRight) & (robot.CurrentLiftPosition > 500) ) {
                    robot.BottomStraightClaw();
                }


                {
                    if (gamepad1.dpad_down || gamepad2.dpad_down) {
                        robot.CloseClaw();
                    }  else if (gamepad1.y) {
                        robot.BottomStraightClaw();
                    } else if (gamepad1.left_bumper) {
                        robot.IntakeOne();
                    }  else if (gamepad1.b) {
                        robot.StraightClaw();
                    }

                    if (gamepad1.x) {
                        //Does not dump bottom claw if it is closed

                        robot.TeleDumpClaw();


                    } else {
                        robot.Hitter.setPosition(robot.HitterHome);
                    }

                }
            }
            //Lift is partially up
        //    else if (robot.CurrentLiftPosition <= robot.LiftThreshold) {
else {
               // robot.BottomStraightClaw();g

                if (robot.TopLeftClaw.getPosition() > robot.TopStraightLeft & robot.TopLeftClaw.getPosition() <  robot.TopOneBlockLeft & (robot.TopRightClaw.getPosition() < robot.TopStraightRight & robot.TopRightClaw.getPosition() >  robot.TopOneBlockRight) ) {
                    robot.StraightTopClaw();
                }

               robot.BottomShutdownClaw();
                if (gamepad2.dpad_up) {
                    robot.Hitter.setPosition(.65);
                } else {
                    robot.Hitter.setPosition(robot.HitterHome);
                }

                {
                    if (gamepad1.dpad_down || gamepad2.dpad_down) {
                        robot.CloseTopClaw();
                    } else if (gamepad1.left_bumper) {
                        robot.IntakeTopOne();
                    } else if (gamepad1.x) {
                        robot.DumpTopClaw();
                    } else if (gamepad1.y) {
                        robot.BottomStraightClaw();
                    } else if (gamepad1.b) {
                        robot.StraightTopClaw();
                    }

                }
            }


        }
        //ENDGAME
        else if (!teleop & endgame & !balance) {
            DriveControls(1);
            float RelicLift = -(gamepad1.right_trigger) + (gamepad1.left_trigger);
            RelicLift = Range.clip(RelicLift, -1, 1);
            RelicLift = (float) scaleInput(RelicLift);

            robot.RelicLift1.setPower(RelicLift);
            robot.RelicLift2.setPower(RelicLift);

            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                robot.CloseClaw();
            }

            if (gamepad1.a) {
                robot.RelicClaw.setPosition(robot.RelicClawClosed);
            } else if (gamepad1.x) {
                robot.RelicClaw.setPosition(robot.RelicClawOpen);
            }

            else if (gamepad1.left_stick_button) {
                robot.RelicClaw.setPosition(0);
            }

            {
                if (gamepad1.b) {
                    robot.RelicThing.setPosition(0);
                } else if (gamepad1.y) {
                    robot.RelicThing.setPosition(.8);
                }
            }

            if (gamepad1.left_bumper) {
                robot.RelicRotater.setPower(1);
            } else if (gamepad1.right_bumper) {
                robot.RelicRotater.setPower(-1);
            } else {
                robot.RelicRotater.setPower(0);
            }
        }


        //BALANCE
        else if (!teleop & !endgame & balance) {
            if (gamepad1.right_trigger > .1) {
                robot.Drive(1, 1);
            } else if (gamepad1.left_trigger > .1) {
                robot.Drive(-1, -1);
            } else {
                DriveControls(3);
            }

        }

        telemetry.addData("Lift1: ", robot.Lift1.getCurrentPosition());
        telemetry.addData("Lift2: ", robot.Lift2.getCurrentPosition());

        telemetry.addData("RelicLift1: ", robot.RelicLift1.getCurrentPosition());
        telemetry.addData("RelicLift2: ", robot.RelicLift2.getCurrentPosition());

        telemetry.addData("LF: ", robot.LF.getCurrentPosition());
        telemetry.addData("RF: ", robot.RF.getCurrentPosition());
        telemetry.addData("RB: ", robot.RB.getCurrentPosition());
        telemetry.addData("LB: ", robot.LB.getCurrentPosition());

        telemetry.addData("RF: ", robot.RF.getPower());
        telemetry.addData("LB: ", robot.LB.getPower());
        telemetry.addData("RB: ", robot.RB.getPower());
        telemetry.addData("LF: ", robot.LF.getPower());

        telemetry.addData("Lift1: ", robot.Lift1.getPower());
        telemetry.addData("Lift2: ", robot.Lift2.getPower());

        telemetry.addData("Right Claw: ", robot.RightClaw.getPosition());
        telemetry.addData("Left Claw: ", robot.LeftClaw.getPosition());

        telemetry.addData("Top Right Claw: ", robot.TopRightClaw.getPosition());
        telemetry.addData("Top Left Claw: ", robot.TopLeftClaw.getPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    public void DriveControls(float slowdivisor) {
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        float Ch1 = gamepad1.right_stick_x / slowdivisor;
        float Ch3 = -gamepad1.left_stick_y / slowdivisor;
        float Ch4 = gamepad1.left_stick_x / slowdivisor;


        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float rightfront = Ch3 - Ch1 - Ch4;
        float rightback = Ch3 - Ch1 + Ch4;
        float leftfront = Ch3 + Ch1 + Ch4;
        float leftback = Ch3 + Ch1 - Ch4;


        // clip the right/left values so that the values never exceed +/- 1
        rightback = Range.clip(rightback, -1, 1);
        leftback = Range.clip(leftback, -1, 1);
        rightfront = Range.clip(rightfront, -1, 1);
        leftfront = Range.clip(leftfront, -1, 1);


        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightfront = (float) scaleInput(rightfront);
        leftfront = (float) scaleInput(leftfront);
        rightback = (float) scaleInput(rightback);
        leftback = (float) scaleInput(leftback);


        // write the values to the motors
        robot.RF.setPower(rightfront);
        robot.LF.setPower(leftfront);
        robot.RB.setPower(rightback);
        robot.LB.setPower(leftback);
    }

}
