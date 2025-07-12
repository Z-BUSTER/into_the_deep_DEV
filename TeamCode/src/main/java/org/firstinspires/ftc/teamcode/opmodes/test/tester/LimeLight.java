package org.firstinspires.ftc.teamcode.opmodes.test.tester;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.vision.VisionPortal;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DevStuff")
@Config

public class LimeLight extends OpMode {

    public Servo base, arms, elbow, wrist, claws;
    public static double basePos = 0.7, elbowPos =1, wristPos = 0.538,clawsPos =0.5 , armsPos= 0;
    double L1 = 9.0;
    double L2 = 6.5;

    boolean auto = false;

    double actualX = 10;
    double actualY = 10;
    double actualZ = 0;

    double cosZ = Math.cos(-actualZ);
    double sinZ = Math.sin(-actualZ);
    double targetX = actualX * cosZ - actualY * sinZ;
    double targetY = targetX * sinZ + actualY * cosZ;


    double visionX = -1;
    double visionY = -1;

    Thread udpThread;
    DatagramSocket socket;
    boolean udpRunning = false;




    final double ARM_MIN = 0.0333;
    final double ARM_MAX = 0.139;
    final double ELBOW_STRAIGHT = 0.625;
    final double ELBOW_RANGE = 112.5;


    @Override
    public void init(){
        base = hardwareMap.get(Servo.class, "base");
        arms= hardwareMap.get(Servo.class, "arm");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claws = hardwareMap.get(Servo.class, "claw");
        arms.setPosition(armsPos);
        elbow.setPosition(elbowPos);
        base.setPosition(basePos);
        wrist.setPosition(wristPos);

        if (!udpRunning) {
            udpRunning = true;
            udpThread = new Thread(() -> {
                try {
                    socket = new DatagramSocket(5801);
                    socket.setSoTimeout(1000); // 1s timeout to prevent indefinite block
                    byte[] buf = new byte[1024];
                    while (!Thread.interrupted()) {
                        try {
                            DatagramPacket packet = new DatagramPacket(buf, buf.length);
                            socket.receive(packet);
                            String message = new String(packet.getData(), 0, packet.getLength());
                            String[] parts = message.split(",");
                            if (parts.length >= 2) {
                                visionX = Double.parseDouble(parts[0]);
                                visionY = Double.parseDouble(parts[1]);
                            }
                        } catch (java.net.SocketTimeoutException e) {
                            // Timeout, continue loop
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                } catch (SocketException e) {
                    telemetry.addLine("Socket error: " + e.getMessage());
                    telemetry.update();
                    e.printStackTrace();
                }  finally {
                    if (socket != null && !socket.isClosed()) {
                        socket.close();
                    }
                }
            });
            udpThread.start();
        }


    }


    @Override
    public void loop(){

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        if(gamepad1.a) {
            clawsPos = 0;
            claws.setPosition(clawsPos);
        } if(gamepad1.b){
            clawsPos = 1;
            claws.setPosition(clawsPos);
        }


        /*
        if(gamepad1.right_stick_x > 0.1 && !auto){
            basePos += 0.0025;
            basePos = Math.min(basePos, 1);

            base.setPosition(basePos);
        }

        if(gamepad1.right_stick_x < -0.1 && !auto){
            basePos -= 0.0025;
            basePos = Math.max(basePos, 0);
            base.setPosition(basePos);
        }

        if(gamepad1.y){
            auto = true;
        }

        if(auto){
            if(visionX > 380){
                basePos += 0.0005;
            }else if(visionX < 260){
                basePos -= 0.0005;
            }else{
                auto = false;
            }
            base.setPosition(basePos);


        }



        if(gamepad1.left_stick_y > 0.1){
            armsPos -= 0.0003;
            armsPos = Math.min(armsPos, 0.2);
            arms.setPosition(armsPos);
        }

        if(gamepad1.left_stick_y < -0.1){
            armsPos += 0.0003;
            armsPos = Math.max(armsPos, 0);
            arms.setPosition(armsPos);
        }

        arms.setPosition(armsPos);

        if(gamepad1.dpad_up ){
            elbowPos -= 0.001;
            elbowPos = Math.max(elbowPos, 0);
            elbow.setPosition(elbowPos);
        }
        if(gamepad1.dpad_down ){
            elbowPos += 0.001;
            elbowPos = Math.min(elbowPos, 1);

        }



        elbow.setPosition(elbowPos);
        */






        double delta = 0.025;
        double deltaZ = 0.01;

        if(Math.abs(gamepad1.left_stick_x) > 0.05)
            actualX += gamepad1.left_stick_x * delta;
        if(Math.abs(gamepad1.left_stick_y) > 0.05)
            actualY -= gamepad1.left_stick_y * delta;
        if (Math.abs(gamepad1.right_stick_x) > 0.05)
            actualZ += gamepad1.right_stick_x * deltaZ;





        targetY = Math.min(Math.max(actualY, -6), 15.5);
        actualZ = Math.max(-Math.PI / 2, Math.min(5 * Math.PI / 12, actualZ));


        double minAngle = -Math.PI / 2;
        double maxAngle = 5 * Math.PI / 12;
        double minServo = 0.365;
        double maxServo = 1.0;


        double normalizedZ = (actualZ - minAngle) / (maxAngle - minAngle);
        basePos = minServo + normalizedZ * (maxServo - minServo);
        base.setPosition(basePos);

        targetX = actualX/Math.cos(actualZ);
        targetX = Math.min(Math.max(targetX, 0), 15.5);

        double dist = Math.sqrt(targetX * targetX + targetY * targetY);


        double reach = L1 + L2;
        if(dist > reach) {
            double angle = Math.atan2(targetY, targetX);
            targetX = reach * Math.cos(angle);
            targetY = reach * Math.sin(angle);
            dist = reach;
        }

        double minDist = Math.abs(L1 - L2);
        if(dist < minDist) {
            double angle = Math.atan2(targetY, targetX);
            targetX = minDist * Math.cos(angle);
            targetY = minDist * Math.sin(angle);
            dist = minDist;
        }



        double angle2 = Math.acos(( (L1*L1 + L2*L2)- dist*dist ) / (2 * L1 * L2));


        double angle1 = Math.atan2(targetY, targetX)
                + Math.acos((L1*L1 + dist*dist - L2*L2) / (2 * L1 * dist));


        double armAngleDeg = Math.toDegrees(angle1);
        telemetry.addData("armsAngle", armAngleDeg);
        double armServoPos = ARM_MAX - (ARM_MAX - ARM_MIN) * (armAngleDeg / 90.0);


        double elbowAngleDeg = Math.toDegrees(angle2);
        elbowAngleDeg = Math.max(75, elbowAngleDeg);

        double elbowServoPos = (280.0 - elbowAngleDeg) / 205.0;
        elbowServoPos = Math.max(0.0, Math.min(1.0, elbowServoPos));


        armsPos = Math.max(ARM_MIN, Math.min(0.25, armServoPos));
        elbowPos = Math.max(0, Math.min(1, elbowServoPos));

        arms.setPosition(armsPos);
        elbow.setPosition(elbowPos);















        if(gamepad1.dpad_right){
            wristPos -= 0.001;
            wristPos = Math.min(wristPos, 1);
            wrist.setPosition(wristPos);
        }

        if(gamepad1.dpad_left){
            wristPos += 0.001;
            wristPos = Math.max(wristPos, 0);
            wrist.setPosition(wristPos);
        }


        telemetry.addData("basePos", basePos);
        telemetry.addData("elbowPos", elbowPos);
        telemetry.addData("armsPos", armsPos);
        telemetry.addData("wristPos", wristPos);
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetY", targetY);
        telemetry.addData("visionX", visionX);
        telemetry.addData("visionY", visionY);
        telemetry.addData("AcutalX", actualX);
        telemetry.addData("AcutalY", actualY);
        telemetry.addData("ActualZ", actualZ);
        telemetry.addData("Auto", auto);





    }

    @Override
    public void stop() {
        if (udpThread != null && udpThread.isAlive()) {
            udpThread.interrupt();
        }

        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }


}
