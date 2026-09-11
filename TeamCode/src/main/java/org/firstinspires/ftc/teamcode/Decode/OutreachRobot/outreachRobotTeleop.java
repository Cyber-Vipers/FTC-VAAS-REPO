package org.firstinspires.ftc.teamcode.Decode.OutreachRobot;

//Driving + Arm Control


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name ="Outreach Robot Teleop")
public class outreachRobotTeleop extends OpMode {
    /*
    VARIABLES
     */
    /**
     * Motors
     */
    public static DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public static Servo extension, extension2;
    /**
     * Custom ArmPose Class
     */
    public ArmPose currentPose = baseArmPose;
    public ArmPose savedPos = baseArmPose;
    public static Servo claw;
    public static Servo baseRotator;
    public static Servo armRotator;
    public static Servo clawRotator;


    public static boolean extensionPosLock = false;
    public static double clawOpenPosition = 0.5;
    public long lastLockTime = 0;
    public double lockCooldown = 0.1;
    public static double extensionMoveScale = 1;
    public static double armRotaterMultiplier = 0.02;
    public static double baseRotatorMultiplier = 0.02;
    public static double clawRotatorMultiplier = 0.02;
    public static double armRotaterPosition = 0;
    public static double baseRotatorPosition = 0;
    public static double clawRotationPosition = 0;
    public static double clawClosePos = 0;

    public static double extensionPosition = 0;
    public static double pExtension = 1;

    public static PIDFCoefficients extensionCoefficients = new PIDFCoefficients(1,0,1,1);

    public static PIDFCoefficients changedExtensionCoefficients = new PIDFCoefficients(1,0,1,1);

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
        extension = hardwareMap.get(Servo.class, "extension");
        extension2 = hardwareMap.get(Servo.class,"extension2");
        baseRotator = hardwareMap.get(Servo.class, "baseRotator");
        armRotator = hardwareMap.get(Servo.class, "armRotator");
        clawRotator = hardwareMap.get(Servo.class, "clawRotator");
        frontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        backRight = hardwareMap.get(DcMotorEx.class,"backRight");

//        extension.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,extensionCoefficients);
//        extension.setPositionPIDFCoefficients(pExtension);
        logArmPositions();
    }
    @Override
    public void loop() {
        preventExtensionOverheat();
        drive();
        controls();
        logArmPositions();
        moveToGiven();
        telemetry.update();

    }


    public void moveToGiven() {
        if (armRotator.getPosition() != armRotaterPosition) {
            armRotator.setPosition(armRotaterPosition);
        }
        if (baseRotatorPosition != baseRotator.getPosition()) {
            baseRotator.setPosition(baseRotatorPosition);
        }
        if (clawRotationPosition != clawRotator.getPosition()) {
            clawRotator.setPosition(clawRotationPosition);
        }
        if (extensionPosition != extension.getPosition()) {
            extension.setPosition((int) extensionPosition);
            extension2.setPosition(extensionPosition);
        }
//        if (!extensionCoefficients.equals(changedExtensionCoefficients)) {
////            extension.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,changedExtensionCoefficients);
//            telemetry.addLine("changing PIDF");
//            telemetry.update();
//        }



    }
    public void logArmPositions() {
        telemetry.addLine("ARM SERVO POSITIONS");
        telemetry.addData("Claw", claw.getPosition());
        telemetry.addData("Extension", extension.getPosition());
        telemetry.addData("baseRotator", baseRotator.getPosition());
        telemetry.addData("armRotator", armRotator.getPosition());
        telemetry.addData("clawRotator", clawRotator.getPosition());
        telemetry.addData("ExtensionPower", extensionPosition);
        telemetry.update();
    }
    public void resetZeroPositions() {
        baseRotator.setPosition(0);
        armRotator.setPosition(0);
        extension.setPosition(0);
        extension2.setPosition(0);
        claw.setPosition(0);
        clawRotator.setPosition(0);
    }
    public boolean isClawOpen() {
        return claw.getPosition() == clawOpenPosition;
    }
    public void preventExtensionOverheat() {
//        if (Math.abs(extension.getTargetPosition() - extension.getCurrentPosition())  < 10) {
//            extension.setTargetPosition(extension.getCurrentPosition());
//            extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            extension.setPower(0);
//        }
    }
    public void moveClaw() {
        if (isClawOpen()) {
            claw.setPosition(clawClosePos);
        } else {
            claw.setPosition(clawOpenPosition);
        }
    }
    /**
     * Controls for the Robot
     * Driver 1
     *  A = moveClaw
     *  B = move to current ArmPose
     *  Y = Reset pose to BasePose and then Move to pose
     *  X = reset to Zero Positions
     *  Left JoyStick = Move
     *  Right JoyStick = Rotate
     *  Left Bumper = Reset Pose to last saved pose and then Move to pos
     *  Driver 2
     *  DPAD Up/Down = Move Life
     *  Dpad Left/Right = Rotate Claw
     *  A = Move Claw
     *  Right Bumper = Save Current Arm Position
     *  Left Bumper = Lock Extension
     *  Right Joystick Y = Move Extension
     *  Right Joystick Button + Left Joystick Button = Reset Extension Zero
     */
    public void controls() {
//        extension.setPositionPIDFCoefficients(pExtension);
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            moveClaw();
        }
        if (gamepad1.xWasPressed()) {
            resetZeroPositions();
        }
        if (gamepad1.bWasPressed()) {
            currentPose.moveToPose();
        }
        if (gamepad1.yWasPressed()) {
            currentPose = baseArmPose;
            currentPose.moveToPose();
        }
        if (Math.abs(gamepad2.left_stick_x) > 0.1) {
            baseRotator.setPosition(baseRotator.getPosition() + (gamepad2.left_stick_x * baseRotatorMultiplier));
            baseRotatorPosition = baseRotator.getPosition() + (gamepad2.left_stick_x * baseRotatorMultiplier);
        }
        if (gamepad2.dpad_up || gamepad2.dpad_down) {
            double value = toGamepadInput(gamepad2.dpad_up, gamepad2.dpad_down);
            armRotator.setPosition(armRotator.getPosition() + (value * armRotaterMultiplier));
            armRotaterPosition = armRotator.getPosition() + (value * armRotaterMultiplier);
        }
        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            double value = toGamepadInput(gamepad2.dpad_left, gamepad2.dpad_right);
            clawRotator.setPosition(clawRotator.getPosition() + (value * clawRotatorMultiplier));
            clawRotationPosition = clawRotator.getPosition() + (value * clawRotatorMultiplier);
        }
        if (gamepad2.left_bumper) {
            if (lastLockTime + lockCooldown * 1000 <= System.currentTimeMillis()) {
                extensionPosLock = !extensionPosLock;
                lastLockTime = System.currentTimeMillis();
            }
        }
        if (gamepad2.rightBumperWasPressed()) {
            //LET'S SAVE A POS
            savedPos = new ArmPose((float) baseRotator.getPosition(), (float) armRotator.getPosition(), (float) clawRotator.getPosition(), (float) extension.getPosition(),isClawOpen());
            currentPose = savedPos;
        }
        if (gamepad1.leftBumperWasPressed()) {
            currentPose = savedPos;
            savedPos.moveToPose();
        }
        if (Math.abs(gamepad2.right_stick_y) > 0.1 && !extensionPosLock) {
            extensionPosition = (float) (extension.getPosition() + (gamepad2.right_stick_y / extensionMoveScale));
        }
    }

    public static ArmPose baseArmPose = new ArmPose(0,0,0,0,false);
    public static class ArmPose {
        float basePositionValue,armRotationValue,clawRotationValue,extensionValue;
        boolean clawOpen;

        public ArmPose(float basePositionValue, float armRotatorValue,float clawRotationValue, float extensionValue,boolean clawOpen) {
            this.basePositionValue = basePositionValue;
            this.armRotationValue = armRotatorValue;
            this.clawRotationValue = clawRotationValue;
            this.extensionValue = extensionValue;
            this.clawOpen = clawOpen;
        }

        public void moveToPose() {
            baseRotator.setPosition(basePositionValue);
            armRotator.setPosition(armRotationValue);
            clawRotator.setPosition(clawRotationValue);
            if (clawOpen) {
                claw.setPosition(1);
            } else {
                claw.setPosition(0);
            }
            extensionPosition = extensionValue;
        }
    }

    public double toGamepadInput(boolean negative, boolean positive) {
        double neg = 0;
        if (negative) {
            neg = -1;
        }
        double pos = boolToDouble(positive);
        return neg + pos;
    }

    public double boolToDouble(boolean bool) {
        if (bool) {
            return 1;
        }
        return 0;
    }
    /**
     * Controls the Drive Train
     * THANK YOU GM0
     */
    public void drive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}