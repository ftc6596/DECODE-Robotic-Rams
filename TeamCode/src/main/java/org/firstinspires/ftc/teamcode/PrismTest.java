package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.util.ArrayList;

@TeleOp(name="PrismTest", group="Linear OpMode")
public class PrismTest extends LinearOpMode {
    private GoBildaPrismDriver prismDriver;
    private RevColorSensorV3 colorsensor;

    PrismAnimations.Solid slot0 = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid slot1 = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.Solid slot2 = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.Solid slot3 = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid slot4 = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.Solid slot5 = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.Solid slot6 = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid slot7 = new PrismAnimations.Solid(Color.PURPLE);
    PrismAnimations.Solid slot8 = new PrismAnimations.Solid(Color.PURPLE);

    @Override
    public void runOpMode() throws InterruptedException {
        prismDriver = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");

        setupleds(slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);

        waitForStart();
        ArrayList<String> currentartifacts = new ArrayList<String>(3);
        int currentslot = 0;
        currentartifacts.add("green");
        currentartifacts.add("purple");
        currentartifacts.add("purple");
        updateleds(prismDriver, currentartifacts.get(0), currentartifacts.get(1), currentartifacts.get(2), slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
        while (opModeIsActive())
        {
            String color = "None";
            int r = colorsensor.red();
            int g = colorsensor.green();
            int b = colorsensor.blue();



            if(g >= 3100 || g <= 100 && g >= 50 && r <= 32 && b < 60){
                color = "green";
                currentslot++;
            }
            else if (b >= 3000 || b >= 50 && b <= 100 && g <= 60 && r < 60) {
                color = "purple";
                currentslot++;
            }
            if (currentslot > 2) {
                currentslot = 0;
            }
            telemetry.addData("slot0: ", currentartifacts.get(0));
            telemetry.addData("slot1: ", currentartifacts.get(1));
            telemetry.addData("slot2: ", currentartifacts.get(2));
            telemetry.addData("color: ", color);
            telemetry.update();
            currentartifacts.set(currentslot, color);
        }
    }
    public static void setupleds(PrismAnimations.Solid slot0, PrismAnimations.Solid slot1, PrismAnimations.Solid slot2, PrismAnimations.Solid slot3, PrismAnimations.Solid slot4, PrismAnimations.Solid slot5, PrismAnimations.Solid slot6, PrismAnimations.Solid slot7, PrismAnimations.Solid slot8)
    {
        slot0.setBrightness(50);
        slot0.setStartIndex(0);
        slot0.setStopIndex(1);
        slot1.setBrightness(50);
        slot1.setStartIndex(2);
        slot1.setStopIndex(3);
        slot2.setBrightness(50);
        slot2.setStartIndex(4);
        slot2.setStopIndex(5);
        slot3.setBrightness(50);
        slot3.setStartIndex(6);
        slot3.setStopIndex(7);
        slot4.setBrightness(50);
        slot4.setStartIndex(8);
        slot4.setStopIndex(9);
        slot5.setBrightness(50);
        slot5.setStartIndex(10);
        slot5.setStopIndex(11);
        slot6.setBrightness(50);
        slot6.setStartIndex(12);
        slot6.setStopIndex(13);
        slot7.setBrightness(50);
        slot7.setStartIndex(14);
        slot7.setStopIndex(15);
        slot8.setBrightness(50);
        slot8.setStartIndex(16);
        slot8.setStopIndex(17);
    }
    public static void updateleds(GoBildaPrismDriver prismDriver, String slot0Color, String slot1Color, String slot2Color, PrismAnimations.Solid slot0, PrismAnimations.Solid slot1, PrismAnimations.Solid slot2, PrismAnimations.Solid slot3, PrismAnimations.Solid slot4, PrismAnimations.Solid slot5, PrismAnimations.Solid slot6, PrismAnimations.Solid slot7, PrismAnimations.Solid slot8)
    {
        if(slot0Color.equals("green"))
        {
            slot0.setBrightness(50);
            slot3.setBrightness(50);
            slot6.setBrightness(50);
            slot0.setPrimaryColor(Color.GREEN);
            slot3.setPrimaryColor(Color.GREEN);
            slot6.setPrimaryColor(Color.GREEN);
        } else if (slot0Color.equals("purple")) {
            slot0.setBrightness(50);
            slot3.setBrightness(50);
            slot6.setBrightness(50);
            slot0.setPrimaryColor(Color.PURPLE);
            slot3.setPrimaryColor(Color.PURPLE);
            slot6.setPrimaryColor(Color.PURPLE);
        }
        else {
            slot0.setBrightness(0);
            slot3.setBrightness(0);
            slot6.setBrightness(0);
        }

        if(slot1Color.equals("green"))
        {
            slot1.setBrightness(50);
            slot4.setBrightness(50);
            slot7.setBrightness(50);
            slot1.setPrimaryColor(Color.GREEN);
            slot4.setPrimaryColor(Color.GREEN);
            slot7.setPrimaryColor(Color.GREEN);
        } else if (slot1Color.equals("purple")) {
            slot1.setBrightness(50);
            slot4.setBrightness(50);
            slot7.setBrightness(50);
            slot1.setPrimaryColor(Color.PURPLE);
            slot4.setPrimaryColor(Color.PURPLE);
            slot7.setPrimaryColor(Color.PURPLE);
        }
        else {
            slot1.setBrightness(0);
            slot4.setBrightness(0);
            slot7.setBrightness(0);
        }
        if(slot2Color.equals("green"))
        {
            slot2.setBrightness(50);
            slot5.setBrightness(50);
            slot8.setBrightness(50);
            slot2.setPrimaryColor(Color.GREEN);
            slot5.setPrimaryColor(Color.GREEN);
            slot8.setPrimaryColor(Color.GREEN);
        } else if (slot2Color.equals("purple")) {
            slot2.setBrightness(50);
            slot5.setBrightness(50);
            slot8.setBrightness(50);
            slot2.setPrimaryColor(Color.PURPLE);
            slot5.setPrimaryColor(Color.PURPLE);
            slot8.setPrimaryColor(Color.PURPLE);
        }
        else {
            slot2.setBrightness(0);
            slot5.setBrightness(0);
            slot8.setBrightness(0);
        }

        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, slot0);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, slot1);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, slot2);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, slot3);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, slot4);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, slot5);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_6, slot6);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_7, slot7);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_8, slot8);
    }
}
