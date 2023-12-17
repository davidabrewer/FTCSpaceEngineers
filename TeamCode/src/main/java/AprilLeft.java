import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "AprilLeftAutonomous", group = "Autonomous")
public class AprilLeft extends LinearOpMode {
    OpenCvCamera webcam;
    int zoneNumber = 0;
    public static int OVERRIDEZONE=0;

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
        while (opModeIsActive() && zoneNumber==0)
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
                        switch(detection.id)
                        {
                            case 24:
                                zoneNumber=1;
                                break;
                            case 1:
                                zoneNumber=2;
                                break;
                            case 0:
                                zoneNumber=3;
                                break;
                        }
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


        //lower the claw and reset encoder
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setTargetPosition(2300);
        upperArm.setPower(.75);
        upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //raise the arm
        runtime.reset();
        while (runtime.milliseconds() < 1000) {
            armRotator.setPower(.90);
        }
        armRotator.setPower(0);
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(OVERRIDEZONE!=0)
        {
            zoneNumber=OVERRIDEZONE;
        }
        int upperarmtarget=-4934;
        int lowerarmtarget=-4925;

        //raise the claw

        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setTargetPosition(-4934);
        upperArm.setPower(.75);
        upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerArm.setTargetPosition(-4925);
        lowerArm.setPower(.75);
        lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        //drive forward

        driveForward(26.5, 50.0, .5);

        telemetry.addLine("Finished driving");
        telemetry.update();
        //release the cone

        clawServo.setPosition(0);
        sleep(500);

        //angle the arm
        armServo.setPosition(0.6);
        //drive to parking

        switch (zoneNumber)
        {
            case 3:
                driveRight(52.0, 50.0, .5);
                driveForward(2.0, 50.0, .5);
                break;
            case 2:
                //do nothing

                break;
            case 1:
                driveBackwards(1.0, 50.0, .5);
                driveLeft(60.0, 50.0, .75);
                driveForward(4.0, 50.0, .5);
                break;
        }

        telemetry.addLine("Lowering arm");
        telemetry.update();

        //lower the claw to 0
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setTargetPosition(0);
        upperArm.setPower(.75);
        upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lowerArm.setTargetPosition(0);
        lowerArm.setPower(.75);
        lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2500);
        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
    private void driveBackwards(Double distance, Double tolerance, Double power) {
        ElapsedTime timer = new ElapsedTime();
        double leftFrontPower=-power;
        double rightFrontPower = -power;
        double leftRearPower =-power;
        double rightRearPower =-power;
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addLine("Reset Encoder");
        telemetry.addData("leftencoder", leftEncoder.getCurrentPosition());
        telemetry.addData("frontencoder", frontEncoder.getCurrentPosition());
        telemetry.addData("rightencoder", rightEncoder.getCurrentPosition());

        telemetry.update();
        timer.reset();

        while(Math.abs(leftEncoder.getCurrentPosition())< distance*8192*4/22.25  )
        {
            double diff = Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition());

        /*    if(Math.abs(diff) > tolerance)
            {
                rightFrontPower = power * (1 + diff/Math.abs(leftEncoder.getCurrentPosition()));
                rightRearPower = power * (1 + diff/Math.abs(leftEncoder.getCurrentPosition()));
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
}

