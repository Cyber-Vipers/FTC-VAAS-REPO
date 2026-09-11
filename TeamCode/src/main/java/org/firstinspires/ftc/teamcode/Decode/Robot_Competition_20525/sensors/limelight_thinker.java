package org.firstinspires.ftc.teamcode.Decode.Robot_Competition_20525.sensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Decode.Tools.Detect_Type;
import org.firstinspires.ftc.teamcode.Decode.Tools.Detection;
import org.firstinspires.ftc.teamcode.tools.Vector3;

import java.util.ArrayList;
import java.util.List;

public class limelight_thinker {
    public Limelight3A limelight;
    public limelight_thinker(Limelight3A lime) {
        limelight = lime;
    }
    public List<Detection> search() {
        LLResult result = limelight.getLatestResult();
        List<Detection> detections = new ArrayList<>();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            for(LLResultTypes.FiducialResult april : result.getFiducialResults()) {
                switch (april.getFiducialId()) {
                    case 21:
                        detections.add(new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),21));
                    case 22:
                        detections.add(new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),22));
                    case 33:
                        detections.add(new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),33));
                    case 20:
                        detections.add(new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),20));
                    case 24:
                        detections.add(new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),24));



//read the code, this ish ow we interface with the limelight
                }
            }

        } else {
        }
        return detections;
    }

}
