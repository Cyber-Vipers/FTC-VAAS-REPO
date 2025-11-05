package org.firstinspires.ftc.teamcode.Competition.sensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.tools.Detect_Type;
import org.firstinspires.ftc.teamcode.tools.Detection;
import org.firstinspires.ftc.teamcode.tools.Vector3;

public class limelight_thinker {
    public Limelight3A limelight;
    public limelight_thinker(Limelight3A lime) {
        limelight = lime;
    }

    public Detection search() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            for(LLResultTypes.FiducialResult april : result.getFiducialResults()) {
                switch (april.getFiducialId()) {
                    case 21:
                        return new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),21);
                    case 22:
                        return new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),22);
                    case 33:
                        return new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),33);
                    case 20:
                        return new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),20);
                    case 24:
                        return new Detection(Detect_Type.April_Tag,new Vector3(tx,ty,ta),24);



//read the code, this ish ow we interface with the limelight
                }
            }

        } else {
        }
        return new Detection(Detect_Type.Purple, new Vector3(0,0,0),0);
    }

}
