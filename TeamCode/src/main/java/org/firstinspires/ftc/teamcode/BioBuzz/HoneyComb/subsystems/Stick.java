package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

public class Stick implements Subsystem {
    public boolean isPushing = false;
    public Servo servo;
    private final double pushPos = 1;
    private final double nonPushPos = 0;

    private final Telemetry telemetry;

    @Override
    public void update() {
        telemetry.addData("Pushing?", isPushing);
        if (isPushing && servo.getPosition() != pushPos) {
            servo.setPosition(pushPos);
        } else if (!isPushing && servo.getPosition() != nonPushPos){
            servo.setPosition(nonPushPos);
        }
    }


    public Stick (HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class,"stick");
        this.telemetry = telemetry;
    }



    public void push() {
        this.isPushing = true;
    }

    public void stopPushing() {
        this.isPushing = false;
    }

}
