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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.ColorSensor;

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

public abstract class BaseOpMode extends OpMode
{
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor lowerArm = null;
    public DcMotor armRotator = null;
    public Servo armServo = null;
    public Servo clawServo = null;
    public DcMotor upperArm = null;

    public NormalizedColorSensor rightSensor;
    public NormalizedColorSensor leftSensor;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    public void initVariables() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_Rear_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_Front_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_Rear_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_Front_drive");
        lowerArm = hardwareMap.get(DcMotor.class, "lowerArm");
        armRotator = hardwareMap.get(DcMotor.class, "armRotator");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        upperArm = hardwareMap.get(DcMotor.class, "upperArm");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        lowerArm.setDirection(DcMotor.Direction.FORWARD);
        upperArm.setDirection(DcMotor.Direction.FORWARD);
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    public void test(){
        double Speed = gamepad1.left_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = gamepad1.right_stick_x;
        double MAX_SPEED = 1.0;
        holonomic(Speed, Turn, Strafe, MAX_SPEED );
    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED){

//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

        double Magnitude = Math.abs(Speed) + Math.abs(Turn) + Math.abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        leftFrontDrive.setPower(Math.scalb((Speed + Turn - Strafe),(int)Math.round(Magnitude)));




        if (leftRearDrive != null) {
            leftRearDrive.setPower(Math.scalb((Speed + Turn + Strafe),(int)Math.round(Magnitude)));
        }
        rightFrontDrive.setPower(Math.scalb((Speed - Turn + Strafe),(int)Math.round(Magnitude)));
        if (rightRearDrive != null) {
            rightRearDrive.setPower(Math.scalb((Speed - Turn - Strafe),(int)Math.round(Magnitude)));
        }
    }





    public void strafeMode(double multiplier)
    {
        double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y) *multiplier;
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftRearDrive.setPower(v3);
        rightRearDrive.setPower(v4);
    }



    public void move(double speed, double direction, double time, boolean useGyro)
    {
        ElapsedTime runtime = new ElapsedTime();
        double r = speed;
        double robotAngle = direction;
        double rightX = 0;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        runtime.reset();
        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftRearDrive.setPower(v3);
        rightRearDrive.setPower(v4);
        while(runtime.milliseconds()<time)
        {

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);


    }

    public void rotate(double angle)
    {

    }


}
