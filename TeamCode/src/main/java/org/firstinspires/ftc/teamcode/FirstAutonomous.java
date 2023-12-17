package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;


@Autonomous(name = "PowerPlayAutonomous1", group = "Autonomous")
public class FirstAutonomous extends LinearOpMode {
    OpenCvCamera webcam;
    int zoneNumber = 0;

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
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        lowerArm.setDirection(DcMotor.Direction.FORWARD);
        upperArm.setDirection(DcMotor.Direction.FORWARD);

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

}
