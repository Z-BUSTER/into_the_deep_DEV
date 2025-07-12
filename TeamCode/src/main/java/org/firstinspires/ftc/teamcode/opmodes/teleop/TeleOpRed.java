package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subss.Arm;
import org.firstinspires.ftc.teamcode.subss.Camera;
import org.firstinspires.ftc.teamcode.subss.Claw;
import org.firstinspires.ftc.teamcode.subss.Slides;
import org.firstinspires.ftc.teamcode.subss.Wrist;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RED")
@Config

public class TeleOpRed extends OpMode {

    //Subsystems
    Arm arm = new Arm(this, 1);
    Slides slides = new Slides(this);
    Wrist wrist = new Wrist(this);
    Claw claw = new Claw(this);
    Camera camera = new Camera(this);
    private OpenCvCamera webcam;

    public boolean switchMode = true;

    public double speed = 1.0;

    public double angle;
    public Point center;

    // Motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU
    public IMU imu;
    public YawPitchRollAngles robotOrientation;
    public double robotYaw;


    // Gamepad
    public double leftStickY, leftStickX, rightStickX;


    //Robot Yaw
    public double targetYaw;


    //Timer
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {

        arm.init();
        slides.init();
        wrist.init();
        claw.init();
//        camera.init();


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        // Initialize the pipeline
        webcam.setPipeline(camera);
        camera.init();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Status", "Camera started");
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open with error code: " + errorCode);
                telemetry.update();
            }
        });



    }
    @Override
    public void loop() {

        //Gamepad JoyStick
        leftStickY = -gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickX = gamepad1.right_stick_x;
        double zAccel = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        robotOrientation = imu.getRobotYawPitchRollAngles();
        robotYaw = robotOrientation.getYaw(AngleUnit.RADIANS);
        if (gamepad1.x) {
            imu.resetYaw();
        }
        // Field Centric Calculations
        double rotX = leftStickX * Math.cos(-robotYaw) - leftStickY * Math.sin(-robotYaw);
        double rotY = leftStickX * Math.sin(-robotYaw) + leftStickY * Math.cos(-robotYaw);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);

        if (gamepad1.right_trigger > 0.8) {
            speed = 0.25;
        } else {
            speed = 1.0;
        }

        telemetry.addData("Angle", camera.realAngle());
        telemetry.addData("Center X", camera.realX());
        telemetry.addData("Center Y", camera.realY());

        if(gamepad1.b){

            if(camera.realX() < -0.5 || camera.realX() > 0.5){
                leftStickX = camera.realX();
                rotX = 0;
                rotY = 0;
            } else if(camera.realY() < -0.5&& camera.realY() > 0.5){
                leftStickY = camera.realY();
                rotX = 0;
                rotY = 0;
            } else if(camera.realAngle() > 67.5 || camera.realAngle() <  22.5){
                wrist.PickUp90();
            } else{
                wrist.PickUp0();
            }
        }

        frontLeft.setPower(((rotY + rotX + rightStickX) / denominator)*speed);
        frontRight.setPower(((rotY - rotX - rightStickX) / denominator)*speed);
        backLeft.setPower(((rotY - rotX + rightStickX) / denominator)*speed);
        backRight.setPower(((rotY + rotX - rightStickX) / denominator)*speed);

        //Switch Mode
        if (gamepad2.back) {
            switchMode = !switchMode;
        }


        //Gamepad Controll
        if(gamepad2.dpad_up && switchMode) {
            placeSample();
        } else if (gamepad2.dpad_up) {
            placeSpecimen();
        }

        if (gamepad2.dpad_down) {
            resetAction();
        }

        if (gamepad2.dpad_right) {
            if (wrist.isUp()) {
                wrist.Down();
            } else {
                wrist.Up();
            }
            pickupSample();
        }

        if (gamepad2.dpad_left) {
            resetSlides();
        }

        if (gamepad2.right_stick_button) {
            //arm.pickUpSpecimen();
            wrist.PickUpSpecimen();
        }

        if(gamepad2.right_bumper) {
            claw.openClaw();
        }

        if(gamepad2.left_bumper) {
            claw.closeClaw();
        }

        if(gamepad2.y) {
            wrist.PickUp0();
        }

        if(gamepad2.x) {
            placeSampleLow();
        }

        if(gamepad2.a) {
            wrist.PickUp90();
        }

        if(gamepad2.b) {
            wrist.Up();
        }

        if(gamepad2.left_trigger > 0.3) {
            armHang();
        }
        if(gamepad2.right_trigger > 0.3) {
            slides.hangExtend();
        }

        if(switchMode) {
            telemetry.addData("MODE", "SAMPLE");
        } else {
            telemetry.addData("MODE", "SPECIMEN");
        }

        telemetry.addData("Yaw: ", Math.toDegrees(robotYaw));
        telemetry.addData("Yaw Acceleration", zAccel);
        telemetry.addData("LeftSlide", slides.leftGetCurrentPosition());
        telemetry.addData("RightSlide", slides.rightGetCurrentPosition());
        telemetry.addData("Arm", arm.getCurrentPosition());
        telemetry.update();
    }
    public boolean slidesReachedTarget(int targetSlides, int threshold) {
        return Math.abs(slides.rightGetCurrentPosition() - targetSlides) < threshold && Math.abs(slides.rightGetCurrentPosition() - targetSlides) < threshold;
    }
    public boolean armReachedTarget(int targetArm, int threshold) {
        return Math.abs(arm.getCurrentPosition() - targetArm) < threshold;
    }

    public void armHang() {
        arm.readyForHang();
        if (armReachedTarget(500, 50)) {
            slides.hangExtend();
        }
        if (slidesReachedTarget(1100, 50)) {
            arm.moveUp();
        }
    }

    public void placeSample() {
        wrist.PlaceSample();
        arm.moveUp();
        if(armReachedTarget(1650, 500)) {
            slides.placeSample();
        }
    }

    public void placeSampleLow() {
        wrist.PlaceSample();
        arm.moveUp();
        if(armReachedTarget(1650, 500)) {
            slides.placeSampleLow();
        }
    }
    public void placeSpecimen() {
        wrist.Up();
        arm.moveUp();
        if(armReachedTarget(1650, 100)) {
            slides.getReadyPlaceSpecimen();
        }
    }
    public void pickupSample() {
        if (armReachedTarget(1650, 100)) {
            slides.placeSpecimen();
        } else {
            claw.openClaw();
            slides.pickupSample();
        }
    }

    public void resetSlides() {
        wrist.Up();
        slides.moveToResetPos();
    }


    public void resetAction() {
        wrist.Down();
        slides.moveToResetPos();
        if(slidesReachedTarget(0, 300)) {
            arm.moveDown();
        }
    }



}

 */