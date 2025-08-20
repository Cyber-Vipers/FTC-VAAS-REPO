package org.firstinspires.ftc.teamcode.runt_2025_2026;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RUNT-TELEOP-V1")
public class teleop_v1 extends OpMode {

    public enum Calculating_Mode_Type {
        none
    }
    public Calculating_Mode_Type Calc_Mode = Calculating_Mode_Type.none;

    //Defining Devices
    public DcMotor back_left,front_left,front_right,back_right;
    public Limelight3A limelight;
    // Default Pipeline that limelight runs on
    public int default_pipeline = 0;
    //Sets intended pipeline
    public int active_pipeline = 0;
    //Check for last pipeline switch
    public long last_pipeline_switch = 0;
    //Limit for how fast that it set's pipeline if it's wrong (seconds)
    public double pipeline_delay = 0.5;

    //Initial function runs when "play" is pressed
    //DO NOT MOVE MOTORS/SERVOS IN THIS FUNCTION
    @Override
    public void init() {
        //Registering Motors & Servos & Peripherals
        register();
    }

    public void register(){
        //Assigning motors to their name inside of the Brain
        back_left = hardwareMap.get(DcMotor.class,"back_left");
        front_left = hardwareMap.get(DcMotor.class,"front_left");
        front_right = hardwareMap.get(DcMotor.class,"front_right");
        back_right = hardwareMap.get(DcMotor.class,"back_right");
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        //Setting pipeline to default pipeline
        limelight.pipelineSwitch(default_pipeline);
    }

    @Override
    public void loop() {
        //Getting Limelight result and doing [insert thing here] with it
        pokemon_go_to_the_polls(limelight.getLatestResult());
    }

    //Getting Limelight result and doing [insert thing here] with it
    public void pokemon_go_to_the_polls(LLResult result) {
        if (!pipeline_correct()) {
            set_pipeline(active_pipeline);
        }
        //Depending on Calc_Mode run different ways of determining things seen by the camera
        switch (Calc_Mode) {
            case none:
                break;
        }
    }


    //Sets pipeline and checks if it is correct
    public boolean pipeline_correct() {
        return limelight.getLatestResult().getPipelineIndex() == active_pipeline;
    }
    //Handling setting pipeline to ensure no reruns
    public void set_pipeline(int set) {
        if (!(last_pipeline_switch + (pipeline_delay * 1000) <= System.currentTimeMillis()))  { return;}
        limelight.pipelineSwitch(set);
        last_pipeline_switch = System.currentTimeMillis();
    }
}
