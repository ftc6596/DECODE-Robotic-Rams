package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class LEDs extends LinearOpMode {
    private  ledcomputer;
    private ColorSensor Sensor;
    private ArrayList<String> Ballslots = new ArrayList<String>(3);

    public void runOpMode() throws InterruptedException{
        ledcomputer = hardwareMap.get(Servo.class,"indicatorLED");

        ledcomputer.setPosition();

//        if (Sensor.green() >= 55){
//            Ballslots.set(1, "green");
//        }
    }
}
