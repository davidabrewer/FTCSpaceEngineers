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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.*;

import java.util.ArrayList;


@Autonomous(name = "CameraTest", group = "Autonomous")
public class CameraTest extends LinearOpMode {
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
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;



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

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

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


        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive())
        {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if(detections != null)
        {
            telemetry.addData("FPS", webcam.getFps());
            telemetry.addData("Overhead ms", webcam.getOverheadTimeMs());
            telemetry.addData("Pipeline ms", webcam.getPipelineTimeMs());

            // If we don't see any tags
            if(detections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else
            {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                for(AprilTagDetection detection : detections)
                {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                    telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                    telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                    telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                    telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                }
            }

            telemetry.update();
        }}


        sleep(10000);
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
