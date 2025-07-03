package org.firstinspires.ftc.teamcode.opmodes.test.tester;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DevStuff")
@Config

public class LimeLight extends OpMode {

    public Servo base, arms, elbow, wrist, claws;
    double basePos = 0.7, elbowPos =1, wristPos,clawsPos =0.5 , armsPos= 0;
    double L1 = 9.0;
    double L2 = 6.5;




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

        if(gamepad1.right_stick_x > 0.1){
            basePos += 0.001;
            elbowPos = Math.min(basePos, 1);

            base.setPosition(basePos);
        }

        if(gamepad1.right_stick_x < -0.1){
            basePos -= 0.001;
            basePos = Math.max(basePos, 0);
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

        if(gamepad1.right_stick_y > 0.1){
            elbowPos += 0.001;
            elbowPos = Math.min(elbowPos, 1);
            elbow.setPosition(elbowPos);
        }

        if(gamepad1.right_stick_y < -0.1){
            elbowPos -= 0.001;
            elbowPos = Math.max(elbowPos, 0);
            elbow.setPosition(elbowPos);
        }

        if(gamepad1.left_stick_x > 0.1){
            wristPos += 0.001;
            wristPos = Math.min(wristPos, 1);
            wrist.setPosition(wristPos);
        }

        if(gamepad1.left_stick_x < -0.1){
            wristPos -= 0.001;
            wristPos = Math.max(wristPos, 0);
            wrist.setPosition(wristPos);
        }


        telemetry.addData("basePos", basePos);
        telemetry.addData("elbowPos", elbowPos);
        telemetry.addData("armsPos", armsPos);




    }


}
