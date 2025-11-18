package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2D;

import java.util.List;

public class Limelight {
    private Telemetry telemetry;

    private Limelight3A limelight;
    private LLResultTypes.FiducialResult result;


    public enum CameraState {
        OFF,
        MOTIF,
        TARGET,
    }

    public CameraState cameraState = CameraState.OFF;

    private double[] inputs = new double[8];

    public Limelight(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        limelight = hwMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.start();
    }

    private double targetX = 0;

    public void update() {
        inputs[0] = cameraState.ordinal();
        limelight.updatePythonInputs(inputs);
        result = aprilTag();
//        LLFieldMap map = new LLFieldMap("{\"fiducials\":[{\"transform\":[0.5877852522924731,-0.8090169943749475,0,-1.482,0.8090169943749475,0.5877852522924731,0,-1.413,0,0,1,0.749,0,0,0,1],\"size\":165.1,\"id\":20,\"family\":\"apriltag3_36h11_classic\",\"unique\":1},{\"transform\":[0.5877852522924731,0.8090169943749475,0,-1.482,-0.8090169943749475,0.5877852522924731,0,1.413,0,0,1,0.749,0,0,0,1],\"size\":165.1,\"id\":24,\"family\":\"apriltag3_36h11_classic\",\"unique\":1}],\"type\":\"ftc\",\"fieldlength\":17.5482504,\"fieldwidth\":8.0519016}\n");

        switch (cameraState){
            case OFF:
                break;
            case MOTIF:
                if (result != null) {
                    Globals.MOTIF = result.getFiducialId() - 20;
//                    cameraState = CameraState.OFF;
                    telemetry.addData("MOTIF: ", Globals.MOTIF);
                } else {
                    telemetry.addLine("Can't find apriltag!");
                }
                break;
            case TARGET:
                int id = result.getFiducialId();
                if ((Globals.RED && id == 24) || (!Globals.RED && id == 20)) {
                    targetX =  aprilTag().getTargetXDegrees();
                    break;
                }
                targetX = 0;
                break;
        }
    }


    public Pose2D getRobotPos() {
        double x = result.getRobotPoseFieldSpace().getPosition().x;
        double y = result.getRobotPoseFieldSpace().getPosition().y;
        double h = result.getRobotPoseFieldSpace().getOrientation().getYaw();
        return new Pose2D(x, y, h);
    }

    public double getTargetX() { return targetX; }

    public LLResultTypes.FiducialResult aprilTag() {
        limelight.pipelineSwitch(1);
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            return result.getFiducialResults().get(0);
        }
        return null;
    }

}
