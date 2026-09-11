package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;
import org.firstinspires.ftc.teamcode.neuronpathing.Line;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

public class LinearRegression implements Subsystem {

    public Flywheel flywheel;
    public Function<Double,Double> equation;
    public Object localizer;
    public boolean isNeuron;
    public Object goalPos;

    @Override
    public void update() {
        if (isNeuron) {
            NeuronLocalizer localizer = (NeuronLocalizer) this.localizer;
            Pose2D currentPosition = localizer.getPose();
            double distance = currentPosition.distanceToPoint((Pose2D) goalPos);
            Double speed = equation.apply(distance);
            flywheel.setSpeed((int) Math.round(speed));
        } else {
            Follower follower = (Follower) this.localizer;
            Pose pose = follower.localizer.pose();
            double distance = pose.distance((Pose) goalPos);
            Double speed = equation.apply(distance);
            flywheel.setSpeed((int) Math.round(speed));
         }
    }

    public void setGoalPos(Object object) {
        if (isNeuron) {
            if (!(object instanceof Pose2D)) {
                throw new RuntimeException();
            }
        } else {
            if (!(object instanceof Pose)) {
                throw new RuntimeException();
            }
        }
        goalPos = object;
    }

    public LinearRegression(Flywheel flywheel, Function<Double,Double> equation, NeuronLocalizer localizer, Pose2D goalPos) {
        this.equation = equation;
        this.flywheel = flywheel;
        this.localizer = localizer;
        isNeuron = true;
        this.goalPos = goalPos;
    }

    public LinearRegression(Flywheel flywheel, Function<Double,Double> equation, Follower localizer, Pose goalPos) {
        this.equation = equation;
        this.flywheel = flywheel;
        this.localizer = localizer;
        isNeuron = false;
        this.goalPos = goalPos;
    }
}
