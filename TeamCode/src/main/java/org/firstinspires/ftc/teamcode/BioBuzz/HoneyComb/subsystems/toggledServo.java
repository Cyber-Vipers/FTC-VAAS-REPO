package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;

public class toggledServo implements Subsystem {


    private boolean isUp = false;
    private final Servo servo;
    private final double upPos;
    private final double downPos;
    private final HardwareMap hardwareMap;

    private final Telemetry telemetry;

    @Override
    public void update() {
        telemetry.addData("Are we up? :", isUp ? "Yes" : "No");
    }
    public toggledServo(HardwareMap hardwareMap, Telemetry telemetry, double upPos, double downPos,String name,boolean isUp,boolean reversed) {
        this.hardwareMap = hardwareMap;
        servo = hardwareMap.get(Servo.class, name);
        this.telemetry = telemetry;
        this.isUp = isUp;
        this.upPos = upPos;
        this.downPos = downPos;
        if (reversed) {
            servo.setDirection(Servo.Direction.REVERSE);
        }
    }

    public void moveDown() {
        isUp = false;
        servo.setPosition(downPos);
    }

    public void nudgeDown() {
        servo.setPosition(downPos + 0.1);
    }

    public void nudgeUp() {
        servo.setPosition(upPos - 0.1);
    }

    public void moveUp() {
        isUp = true;
        servo.setPosition(upPos);
    }


    public boolean isUp() {
        return servo.getPosition() == upPos || servo.getPosition() == upPos + 0.1;
    }
    public void toggle() {
        if (isUp) {
            moveDown();
        } else {
            moveUp();
        }
    }




}
