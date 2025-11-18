package org.firstinspires.ftc.teamcode.bunsBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

public class BunsBot {
    Telemetry telemetry;
    HardwareMap hwMap;
    LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();

    public Drivetrain drivetrain;

    //Outtake
    public DcMotorEx outtake1;
    public DcMotorEx outtake2;

    //Intake
    DcMotorEx intake;

    //Park
    DcMotorEx park;

    public enum States  {
        DOWN,
        UP,
        HOLD,
        OFF
    }


    private double INTAKE_IN_POWER = -0.85;
    private double INTAKE_OUT_POWER = 0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private double FOOT_UP_POWER = 1.0;
    private double FOOT_DOWN_POWER = -0.85;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    public States outtakeState = States.HOLD;
    public States footState = States.HOLD;

    public static ElapsedTime pivotDownTime = new ElapsedTime();


    public BunsBot(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

        drivetrain = new Drivetrain(hwMap, telemetry);

        outtake1 = hwMap.get(DcMotorEx.class, "o1");
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2 = hwMap.get(DcMotorEx.class, "o2");
        intake = hwMap.get(DcMotorEx.class, "i");
        park = hwMap.get(DcMotorEx.class, "park");

        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        park.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void reset() {
        runtime.reset();
        catatime.reset();
    }

    public void update() {
        updateOuttake();
        updateFoot();
        drivetrain.update();
    }

    public void intake() {
        intake.setPower(INTAKE_IN_POWER);
    }
    public void outtakeOnTheIntake() {
        intake.setPower(INTAKE_OUT_POWER);
    }
    public void turnOffIntake() {
        intake.setPower(INTAKE_OFF_POWER);
    }

    public void resetTimer() {
        pivotDownTime.reset();
    }

    public void updateFoot() {
        switch (footState) {
            case DOWN:
                footPower = FOOT_DOWN_POWER;
                break;
            case UP:
                footPower = FOOT_UP_POWER;
                break;
            default:
                park.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                footPower = FOOT_OFF_POWER;
                break;
        }
        park.setPower(footPower);

    }

    double IN = 90;
    public void updateOuttake() {
        switch(outtakeState){
            case UP:
                outtake2.setPower(-1);
                outtake1.setPower(-1);
                pivotDownTime.reset();
                break;
            case OFF:
                outtake2.setPower(0);
                outtake1.setPower(0);
                break;
            case DOWN:
                if (pivotDownTime.time() < 0.4) {
                    outtake2.setPower(1);
                    outtake1.setPower(1);
                } else {
                    outtakeState = States.HOLD;
                    outtake2.setPower(0.1);
                    outtake1.setPower(0.1);
                }
                break;
            case HOLD:
                outtake2.setPower(0.1);
                outtake1.setPower(0.1);
                break;
            default:
                if (outtake1.getCurrentPosition() >= IN) {
                    outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    outtake2.setPower(0.1);
                    outtake1.setPower(0.1);
                    outtakeState = States.HOLD;
                } else {
                    outtake2.setPower(1);
                    outtake1.setPower(1);
                }
                break;

        }

    }
    public void resetOuttake() {
        outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void backwards() {
        drivetrain.setDrivePower(-1, -1, -1, -1);

    }
    public void strafeLeft() {
        drivetrain.setDrivePower(-1, 1, 1, -1);
    }
    public void strafeRight() {
        drivetrain.setDrivePower(1, -1, -1, 1);
    }

    public void goToPoint(Pose2D pose, boolean through, double maxSpeed) {
        drivetrain.goToPoint(pose, through, maxSpeed);
        while (!opMode.isStopRequested() && drivetrain.isBusy() ) {
            update();
            telemetry.update();
        }
    }

    public void sleep(double ms) {
        double time = System.currentTimeMillis();
        while (!opMode.isStopRequested() &&  System.currentTimeMillis() - time < ms) {
            update();
        }
    }

}
