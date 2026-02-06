package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.lang.ref.SoftReference;
import java.util.ArrayList;
@Disabled
@TeleOp(name="PrismTest", group="Linear OpMode")
public class PrismTest extends LinearOpMode {

    private GoBildaPrismDriver prismDriver;
    private RevColorSensorV3 colorsensor;

    PrismAnimations.Solid slot0 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot1 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot2 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot3 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot4 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot5 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot6 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot7 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot8 = new PrismAnimations.Solid(Color.YELLOW);

    @Override
    public void runOpMode() throws InterruptedException {
        prismDriver = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");

        setupleds("Shooting", slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
        waitForStart();
        ArrayList<String> currentartifacts = new ArrayList<String>(3);
        int currentslot = 0;
        boolean abletochange = true;
        boolean canReadBall = true;
        String SorterMode = "Shooting";
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
            if (SorterMode.equals("Intake"))
            {
                if(canReadBall)
                {
                    if(g >= 3100 || g <= 100 && g >= 50 && r <= 32 && b < 60){
                        color = "green";
                        canReadBall = false;
                        currentartifacts.set(currentslot, "green");
                        UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    }
                    else if (b >= 3000 || b >= 50 && b <= 100 && g <= 60 && r < 60) {
                        color = "purple";
                        canReadBall = false;
                        currentartifacts.set(currentslot, "purple");
                        UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    }
                }
            }
            else
            {
                if(gamepad2.right_trigger > 0)
                {
                    currentartifacts.set(currentslot, "none");
                    UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                }
            }


            if (gamepad2.dpad_right && abletochange) {
                currentslot ++;
                abletochange = false;
                canReadBall = true;
            }
            else if (gamepad2.dpad_left && abletochange) {
                currentslot --;
                abletochange = false;
                canReadBall = true;
            }
            if (!gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.y) {
                abletochange = true;
            }
            if (gamepad2.y && abletochange){
                abletochange = false;

                currentslot--;

                if (SorterMode.equals("Intake"))
                {
                    SorterMode = "Shooting";
                } else {
                    SorterMode = "Intake";
                    canReadBall = true;
                }

                setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
            }
            if (currentslot > 2) {
                currentslot = 0;
            }
            if (currentslot < 0) {
                currentslot = 2;
            }
            if (!abletochange)
            {
                UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
            }
            telemetry.addData("slot0: ", currentartifacts.get(0));
            telemetry.addData("slot1: ", currentartifacts.get(1));
            telemetry.addData("slot2: ", currentartifacts.get(2));
            telemetry.addData("color: ", color);
            telemetry.addData("Current Slot: ", currentslot);
            telemetry.addData("Sorter Mode: ", SorterMode);
            telemetry.update();
        }
    }
    public static void setupleds(String SorterMode, PrismAnimations.Solid slot0, PrismAnimations.Solid slot1, PrismAnimations.Solid slot2, PrismAnimations.Solid slot3, PrismAnimations.Solid slot4, PrismAnimations.Solid slot5, PrismAnimations.Solid slot6, PrismAnimations.Solid slot7, PrismAnimations.Solid slot8)
    {

        if (SorterMode.equals("Intake")) {
            slot0.setBrightness(50);
            slot0.setStartIndex(9);
            slot0.setStopIndex(11);
            slot3.setBrightness(50);
            slot3.setStartIndex(9);
            slot3.setStopIndex(11);
            slot6.setBrightness(50);
            slot6.setStartIndex(12);
            slot6.setStopIndex(14);

            slot1.setBrightness(50);
            slot1.setStartIndex(0);
            slot1.setStopIndex(2);
            slot4.setBrightness(50);
            slot4.setStartIndex(0);
            slot4.setStopIndex(2);
            slot7.setBrightness(50);
            slot7.setStartIndex(15);
            slot7.setStopIndex(17);

            slot2.setBrightness(50);
            slot2.setStartIndex(3);
            slot2.setStopIndex(5);
            slot5.setBrightness(50);
            slot5.setStartIndex(3);
            slot5.setStopIndex(5);
            slot8.setBrightness(50);
            slot8.setStartIndex(6);
            slot8.setStopIndex(8);
        } else {
            slot0.setStartIndex(0);
            slot0.setStopIndex(1);
            slot3.setBrightness(50);
            slot3.setStartIndex(2);
            slot3.setStopIndex(3);
            slot6.setBrightness(50);
            slot6.setStartIndex(4);
            slot6.setStopIndex(5);

            slot1.setBrightness(50);
            slot1.setStartIndex(6);
            slot1.setStopIndex(7);
            slot4.setBrightness(50);
            slot4.setStartIndex(8);
            slot4.setStopIndex(9);
            slot7.setBrightness(50);
            slot7.setStartIndex(10);
            slot7.setStopIndex(11);

            slot2.setBrightness(50);
            slot2.setStartIndex(12);
            slot2.setStopIndex(13);
            slot5.setBrightness(50);
            slot5.setStartIndex(14);
            slot5.setStopIndex(15);
            slot8.setBrightness(50);
            slot8.setStartIndex(16);
            slot8.setStopIndex(17);
        }
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

    public static void UpdateLEDs(ArrayList<String> currentartifacts, int currentslot, GoBildaPrismDriver prismDriver, PrismAnimations.Solid slot0, PrismAnimations.Solid slot1, PrismAnimations.Solid slot2, PrismAnimations.Solid slot3, PrismAnimations.Solid slot4, PrismAnimations.Solid slot5, PrismAnimations.Solid slot6, PrismAnimations.Solid slot7, PrismAnimations.Solid slot8)
    {
        int currentslotPlus1 = (currentslot+1)%3;
        int currentslotPlus2 = (currentslot+2)%3;
        updateleds(prismDriver, currentartifacts.get(currentslot), currentartifacts.get(currentslotPlus1), currentartifacts.get(currentslotPlus2), slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
    }
}
