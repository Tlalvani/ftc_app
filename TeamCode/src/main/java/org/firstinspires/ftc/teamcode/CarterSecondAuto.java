/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//1120 is 1 rotation

@Autonomous(name="Carter2ndAuto", group="Pushbot")
public class CarterSecondAuto extends TestAutoClasses {


    /* Declare OpMode members. */


   int smallvalue = 420;
    int bigvalue = 720;





    @Override
    public void runOpMode() {
    /*    LB = hardwareMap.dcMotor.get("LB");
        LF = hardwareMap.dcMotor.get("LF");
        RB = hardwareMap.dcMotor.get("RB");
        RF = hardwareMap.dcMotor.get("RF");
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);

*/
        initSensors();




        waitForStart();

        goTo(smallvalue,smallvalue,0.5);
        imu (-90);
        goTo(smallvalue,smallvalue,0.5);
        imu (-90);
        goTo(smallvalue,smallvalue,0.5);
        imu (-90);
        goTo(smallvalue,smallvalue,0.5);
        imu (-90);



    }

    public void goTo(int lmotor, int rmotor, double power) {
        robot.LF.setTargetPosition( robot.LF.getCurrentPosition() + lmotor);
        robot.RF.setTargetPosition( robot.RF.getCurrentPosition() + rmotor);
        robot.LB.setTargetPosition( robot.LB.getCurrentPosition() + lmotor);
        robot.RB.setTargetPosition( robot.RB.getCurrentPosition() + rmotor);
        robot.LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LF.setPower(power * 1);
        robot.RF.setPower(power * 1);
        robot.LB.setPower(power * 1);
        robot.RB.setPower(power * 1);
        while ( robot.LF.isBusy() &&  robot.RF.isBusy()) {

        }
        robot.RF.setPower(0);
        robot.LF.setPower(0);
        robot.RB.setPower(0);
        robot.LB.setPower(0);
        robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    }




