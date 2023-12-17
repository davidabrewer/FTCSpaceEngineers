package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous(name = "PowerPlayAutonomousLeft", group = "Autonomous")
@Disabled
public class LeftAutonomous extends LinearOpMode {
    OpenCvCamera webcam;
    int zoneNumber = 0;
    public static int OVERRIDEZONE=0;

    SignalDetectPipeline signalPipeline;
    public ElapsedTime runtime = new ElapsedTime();
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

        // initialize all hardware


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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        signalPipeline = new SignalDetectPipeline();

        webcam.setPipeline(signalPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error on camera" + errorCode);
                telemetry.update();

            }
        });

        telemetry.addLine("Waiting for start");

        telemetry.addData("redvalue", signalPipeline.redColorValue);
        telemetry.addData("greenvalue", signalPipeline.greenColorValue);
        telemetry.addData("bluevalue", signalPipeline.blueColorValue);
        telemetry.update();


        waitForStart();


        while (zoneNumber == 0 && opModeIsActive() && !isStopRequested()) {
            //            webcam.startStreaming(320, 240,OpenCvCameraRotation.UPSIDE_DOWN);
            telemetry.addLine("In the loop");
            telemetry.addData("redvalue", signalPipeline.redColorValue);
            telemetry.addData("greenvalue", signalPipeline.greenColorValue);
            telemetry.addData("bluevalue", signalPipeline.blueColorValue);
            telemetry.update();

            if (signalPipeline.blueColorValue > 80) {
                zoneNumber = 2;
            } else if (signalPipeline.redColorValue > 80) {
                zoneNumber = 3;
            } else if (signalPipeline.greenColorValue > 130) {
                zoneNumber = 1;
            }
        }
        telemetry.addData("redvalue", signalPipeline.redColorValue);
        telemetry.addData("greenvalue", signalPipeline.greenColorValue);
        telemetry.addData("bluevalue", signalPipeline.blueColorValue);
        telemetry.addData("zoneNumber", zoneNumber);
        telemetry.update();
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setTargetPosition(2300);
        upperArm.setPower(.75);
        upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (runtime.milliseconds() < 1000) {
            armRotator.setPower(1);
        }
        armRotator.setPower(0);
        if(OVERRIDEZONE!=0)
        {
            zoneNumber=OVERRIDEZONE;
        }
        switch (zoneNumber)
        {
            case 3:
                move(1, 5.50, 1300, false);
                rotate(1,100);
                move(.75, .875, 1150, false);
                break;
            case 2:
                //move(.75, .655, 1500, false);
                move(.75, .865, 1300, false);
               break;
            case 1:
                move(1, 2.3, 1600, false);
                move(.75, .875, 1200, false);
                break;
        }

   /*   upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setTargetPosition(-4934);
        upperArm.setPower(.75);
        upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerArm.setTargetPosition(-4909);
        lowerArm.setPower(.75);
        lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); */
        //  move(.5, Math.PI/4, 2000, false);
    }

    public void strafeMode(double multiplier) {
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y) * multiplier;
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFrontDrive.setPower(v1);
        rightFrontDrive.setPower(v2);
        leftRearDrive.setPower(v3);
        rightRearDrive.setPower(v4);
    }


    public void move(double speed, double direction, double time, boolean useGyro) {
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
        while (runtime.milliseconds() < time) {

        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);


    }

    public void rotate(double speed, double time)
    {
        ElapsedTime runtime = new ElapsedTime();
        double r = speed;

        double rightX = 0;
        final double v1 = r;
        final double v2 = r * -1;
        final double v3 = r ;
        final double v4 = r * -1;

        runtime.reset();

        while(runtime.milliseconds()<time)
        {
            leftFrontDrive.setPower(v1);
            rightFrontDrive.setPower(v2);
            leftRearDrive.setPower(v3);
            rightRearDrive.setPower(v4);
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public static class SignalDetectPipeline extends OpenCvPipeline {

        private final int TOPLEFTX = 130;
        private final int TOPLEFTY = 90;


        private final int BOTTOMRIGHTX = 165;
        private final int BOTTOMRIGHTY = 125;


        public volatile double redColorValue;
        public volatile double blueColorValue;
        public volatile double greenColorValue;
        Mat redcb = new Mat();
        Mat bluecb = new Mat();
        Mat greencb = new Mat();


        private void extractColor(Mat source) {
            // Imgproc.cvtColor(source, yCrb, Imgproc.COLOR_BGR2RGB);
            Mat rgbmat = new Mat();
            Imgproc.cvtColor(source, rgbmat, Imgproc.COLOR_RGBA2RGB);

            Core.extractChannel(rgbmat, redcb, 0);
            Core.extractChannel(rgbmat, bluecb, 2);
            Core.extractChannel(rgbmat, greencb, 1);
            redColorValue = Core.mean(redcb).val[0];
            blueColorValue = Core.mean(bluecb).val[0];
            greenColorValue = Core.mean(greencb).val[0];


        }


        @Override
        public void init(Mat mat) {

            super.init(mat);

        }

        @Override
        public Mat processFrame(Mat input) {

            Point topLeft = new Point(

                    TOPLEFTX,
                    TOPLEFTY);

            Point bottomRight = new Point(
                    BOTTOMRIGHTX,
                    BOTTOMRIGHTY);
            Imgproc.rectangle(
                    input,
                    topLeft, bottomRight, new Scalar(0, 255, 0), 4);


            Mat signalMat = input.submat(new Rect(topLeft, bottomRight));


            extractColor(signalMat);

            return input;
        }
    }
    private void driveLeft(Double distance, Double tolerance, Double power) {
        double leftFrontPower=-power * 1.21;
        double rightFrontPower =power * 1.25;
        double leftRearPower =power;
        double rightRearPower =-power;
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Reset Encoder");
        telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
        telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
        telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());

        telemetry.update();
        while(Math.abs(frontEncoder.getCurrentPosition())< distance*8192*.05)
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

        while(Math.abs(frontEncoder.getCurrentPosition())< distance*8192*.05)
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

        while((leftEncoder.getCurrentPosition())< distance*8192*4/22.25  )
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

}
