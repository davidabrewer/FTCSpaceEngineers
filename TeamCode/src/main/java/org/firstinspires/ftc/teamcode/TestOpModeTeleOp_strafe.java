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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Strafing TeleOp", group="Iterative Opmode")

public class TestOpModeTeleOp_strafe extends BaseOpMode
{
    public static int targetposition=-1;
    public static int upperarmtarget=0;
    public static int lowerarmtarget=0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        this.initVariables();
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
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double powerMultiplier=1.0;


        if(gamepad1.right_trigger>0.0D)
        {
            powerMultiplier=.75;
        }
        strafeMode(powerMultiplier);

        if(gamepad2.left_stick_y>0)
        {
            targetposition=-1;
            if(lowerArm.getCurrentPosition() <= 0) {
                lowerArm.setPower(gamepad2.left_stick_y);
            }
            if(upperArm.getCurrentPosition() <= 0) {
                upperArm.setPower(gamepad2.left_stick_y);
            }
        }
        else if (gamepad2.left_stick_y<0)
        {
            targetposition=-1;
            if(lowerArm.getCurrentPosition() > -4980) {
                lowerArm.setPower(gamepad2.left_stick_y);
            }
            else
            {
                lowerArm.setPower(0);
            }
            if(upperArm.getCurrentPosition() > -4930) {
                upperArm.setPower(gamepad2.left_stick_y);
            }
            else
            {
                lowerArm.setPower(0);
            }
        }
        else
        {
            if(targetposition!=-1) {
                if(upperArm.getCurrentPosition()>upperarmtarget && Math.abs(upperArm.getCurrentPosition()-upperarmtarget) > 100)
                {
                    upperArm.setPower(-1);
                }
                else if(upperArm.getCurrentPosition()<upperarmtarget && Math.abs(upperArm.getCurrentPosition()-upperarmtarget) > 100)
                {
                    upperArm.setPower(1);
                }
                else{
                    upperArm.setPower(0);

                }
                if(lowerArm.getCurrentPosition()>lowerarmtarget && Math.abs(lowerArm.getCurrentPosition()-lowerarmtarget) > 100)
                {
                    lowerArm.setPower(-1);
                }
                else if(lowerArm.getCurrentPosition()<lowerarmtarget && Math.abs(lowerArm.getCurrentPosition()-lowerarmtarget) > 100)
                {
                    lowerArm.setPower(1);
                }
                else{
                    lowerArm.setPower(0);

                }
            }
            else
            {
                lowerArm.setPower(0);
                upperArm.setPower(0);
            }

        }

      //  if(gamepad2.right_stick_y!=0 && Math.abs(armRotator.getCurrentPosition())<490)
        if(gamepad2.right_trigger!=0)

            {
            armRotator.setPower(gamepad2.right_trigger);

        }
        else
        {
            armRotator.setPower(0);
        }

        if(gamepad2.right_stick_x!=0 && armServo.getPosition()<2)
        {
            armServo.setPosition(armServo.getPosition() + gamepad2.right_stick_x * .010);
        }
        if(gamepad2.dpad_up)
        {
            armServo.setPosition(0.619);
        }
        if(gamepad2.dpad_right)
        {
            armServo.setPosition(0.954);
        }
        if(gamepad2.dpad_left)
        {
            armServo.setPosition(.427);
        }
      /*  if(gamepad2.dpad_down)
        {
            armServo.setPosition(1);
        }*/

        if(gamepad2.right_bumper)
        {
            clawServo.setPosition(1);
        }

        if(gamepad2.left_bumper)
        {
            clawServo.setPosition(0);
        }
        if (gamepad2.a)
        { targetposition=0;
            upperarmtarget=0;
            lowerarmtarget=0;
        }
        if (gamepad2.b)
        { targetposition=1;
            upperarmtarget=-2206;
            lowerarmtarget=-1963;
        }
        if(gamepad2.y)
        { targetposition=2;
            upperarmtarget=-3752;
            lowerarmtarget=-3464;
        }
        if(gamepad2.x)
        { targetposition=3;
            upperarmtarget=-4934;
            lowerarmtarget=-4925;
        }

        telemetry.addData("upper arm", upperArm.getCurrentPosition());
        telemetry.addData("lower arm", lowerArm.getCurrentPosition());
        telemetry.addData("left stick y", gamepad2.left_stick_y);
        telemetry.addData("arm rotator", armServo.getPosition());
        telemetry.addData("upperarmpower", upperArm.getPower());
        telemetry.addData("uppertarget", upperarmtarget);
        telemetry.addData("lowertarget", lowerarmtarget);
        telemetry.update();  telemetry.update();



















    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
