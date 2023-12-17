package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;


@Config
@Autonomous(name = "Testing Autonomous", group = "Autonomous")
public class TestingAutonomous extends LinearOpMode {

    enum TESTMETHOD {FORWARD,
                    LEFT,
                    RIGHT,
                    ROTATE,
                    RAISEARM}



    public static Double TOLERANCE =50.0;
    public static Double RRC=1.0;
    public static Double FLC=1.0;
    public static Double RLC=1.0;
    public static Double DISTANCE =12.0;
    public static Double RFC = 1.0;
    public static Double ANGLE =90.0;
    public static Double POWER =.5;
    public static Double TICKS =8192.0;
    public static TESTMETHOD METHOD=TESTMETHOD.FORWARD;
    public static int ARMTARGET=848;
    public static double ARMPOWER=1.0;

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor lowerArm = null;
    public DcMotor armRotator = null;
    public DcMotor upperArm = null;
    public Servo armServo = null;
    public Servo clawServo = null;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;



    @Override
    public void runOpMode() throws InterruptedException {
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_Rear_drive");
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
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftEncoder = hardwareMap.get(DcMotorEx.class, "right_Front_drive");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "left_Front_drive");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "left_Rear_drive");

        waitForStart();

        switch(METHOD)
        {
            case FORWARD:
                telemetry.addLine("Driving forward");
                telemetry.update();
                //sleep(1000);
                driveForward(DISTANCE, TOLERANCE, POWER);
                break;
            case LEFT:
                telemetry.addLine("Driving left");
                telemetry.update();
                driveLeft(DISTANCE, TOLERANCE, POWER);
                break;
            case RIGHT:
                telemetry.addLine("Driving right");
                telemetry.update();
                driveRight(DISTANCE, TOLERANCE, POWER);
                break;
            case ROTATE:
                telemetry.addLine("Rotating");
                telemetry.update();
                //rotate(ANGLE, POWER);
                break;
            case RAISEARM:
                telemetry.addLine("Raising arm");
                telemetry.update();
                //raiseArm();
                break;
        }


    }

    private void raiseArm() {
        /*upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setTargetPosition(2300);
        upperArm.setPower(.75);
        upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        /*armRotator.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );
        armRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotator.setDirection(DcMotorSimple.Direction.FORWARD);
        armRotator.setTargetPosition(ARMTARGET);
        armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotator.setPower(ARMPOWER); */
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() < 1000) {
        armRotator.setPower(1);
        }
        armRotator.setPower(0);
    }

    private void driveLeft(Double distance, Double tolerance, Double power) {
        double leftFrontPower=-power * FLC;
        double rightFrontPower =power * RFC;
        double leftRearPower =power * RLC;
        double rightRearPower =-power * RRC;
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Reset Encoder");
        telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
        telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
        telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());

        telemetry.update();
        while(Math.abs(frontEncoder.getCurrentPosition())< distance*TICKS*.05)
        {
           /* if(leftEncoder.getCurrentPosition() > tolerance || rightEncoder.getCurrentPosition() > tolerance)
            {
                rightFrontPower = rightFrontPower * (1 - 0);
                leftFrontPower = leftFrontPower * (1 - 0);
            }
            if(leftEncoder.getCurrentPosition() < tolerance || rightEncoder.getCurrentPosition() < tolerance)
            {
                rightFrontPower = rightFrontPower * (1 + 0);
                leftFrontPower = leftFrontPower * (1 + 0);

            }*/
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftRearDrive.setPower(leftRearPower);
            rightRearDrive.setPower(rightRearPower);
            leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
            telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
            telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());
            telemetry.addData("right rear actual", rightRearDrive.getPower());
            telemetry.addData("right rear supposed", rightRearPower);
            telemetry.addData("right front actual", rightFrontDrive.getPower());
            telemetry.addData("right front supposed", rightFrontPower);
            telemetry.addData("left rear actual", leftRearDrive.getPower());
            telemetry.addData("left rear supposed", leftRearPower);
            telemetry.addData("left front actual", leftFrontDrive.getPower());
            telemetry.addData("left front supposed", leftFrontPower);

            telemetry.update();

        }
        rightRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
    }

    private void driveRight(Double distance, Double tolerance, Double power) {
        ElapsedTime timer = new ElapsedTime();
        double leftFrontPower=power * (1.1);
        double rightFrontPower = -power * 1.2;
        double leftRearPower =-power;
        double rightRearPower =power * (1.05);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Reset Encoder");
        telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
        telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
        telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());

        telemetry.update();
        timer.reset();

        while(Math.abs(frontEncoder.getCurrentPosition())< distance*TICKS*.05)
        {
            double diff = (leftEncoder.getCurrentPosition());

           /* if(Math.abs(diff) > tolerance)
            {
                rightFrontPower = power * (1 - diff/Math.abs(frontEncoder.getCurrentPosition()));
                leftRearPower = power * (1 - diff/Math.abs(frontEncoder.getCurrentPosition()));
                leftFrontPower = power * (1 + diff/Math.abs(frontEncoder.getCurrentPosition()));
                rightRearPower = power * (1 + diff/Math.abs(frontEncoder.getCurrentPosition()));

            } */
            /*if(Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition()) < -1*tolerance)
            {
                rightFrontPower = power * (1 + CORRECTION);
                rightRearPower = power * (1 + CORRECTION);

            }*/
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftRearDrive.setPower(leftRearPower);
            rightRearDrive.setPower(rightRearPower);
            leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
            telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
            telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());
            telemetry.addData("right rear actual", rightRearDrive.getPower());
            telemetry.addData("right rear supposed", rightRearPower);
            telemetry.addData("right front actual", rightFrontDrive.getPower());
            telemetry.addData("right front supposed", rightFrontPower);
            telemetry.addData("left rear actual", leftRearDrive.getPower());
            telemetry.addData("left rear supposed", leftRearPower);
            telemetry.addData("left front actual", leftFrontDrive.getPower());
            telemetry.addData("left front supposed", leftFrontPower);




            telemetry.update();

        }


        rightRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);

        sleep(30000);

    }

    private void driveForward(Double distance, Double tolerance, Double power) {
        ElapsedTime timer = new ElapsedTime();
        double leftFrontPower=power;
        double rightFrontPower = power;
        double leftRearPower =power;
        double rightRearPower =power;
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Reset Encoder");
        telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
        telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
        telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());

        telemetry.update();
        timer.reset();

        while((leftEncoder.getCurrentPosition())< distance*TICKS*4/22.25  )
        {
            double diff = Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition());

            if(Math.abs(diff) > tolerance)
            {
                rightFrontPower = power * (1 + diff/Math.abs(leftEncoder.getCurrentPosition()));
                rightRearPower = power * (1 + diff/Math.abs(leftEncoder.getCurrentPosition()));
            }
            /*if(Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition()) < -1*tolerance)
            {
                rightFrontPower = power * (1 + CORRECTION);
                rightRearPower = power * (1 + CORRECTION);

            }*/
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftRearDrive.setPower(leftRearPower);
            rightRearDrive.setPower(rightRearPower);
            leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
            telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
            telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());
            telemetry.addData("right rear actual", rightRearDrive.getPower());
            telemetry.addData("right rear supposed", rightRearPower);
            telemetry.addData("right front actual", rightFrontDrive.getPower());
            telemetry.addData("right front supposed", rightFrontPower);
            telemetry.addData("left rear actual", leftRearDrive.getPower());
            telemetry.addData("left rear supposed", leftRearPower);
            telemetry.addData("left front actual", leftFrontDrive.getPower());
            telemetry.addData("left front supposed", leftFrontPower);




            telemetry.update();

        }


        rightRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);

        sleep(30000);

    }

    private void rotate(Double angle, Double power)
    {

    }


}


