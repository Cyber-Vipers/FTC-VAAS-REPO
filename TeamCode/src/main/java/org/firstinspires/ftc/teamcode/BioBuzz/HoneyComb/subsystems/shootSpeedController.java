package org.firstinspires.ftc.teamcode.BioBuzz.HoneyComb.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SubsytemOrganizer.Subsystem;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

import java.util.function.DoubleUnaryOperator;

public class shootSpeedController implements Subsystem {

    public Flywheel flywheel;
    public DoubleUnaryOperator equation;
    public Object localizer;
    private final DistanceSource distanceSource;
    @Override
    public void update() {
        double distance = distanceSource.distanceToGoal();
        double speed = equation.applyAsDouble(distance);
        flywheel.setSpeed((int) speed);
    }

    public void setGoalPos(Object object) {
       distanceSource.setGoal(object);
    }

    public shootSpeedController(Flywheel flywheel, DoubleUnaryOperator equation, NeuronLocalizer localizer, Pose2D goalPos) {
        this.equation = equation;
        this.flywheel = flywheel;
        distanceSource = new NeuronDistanceSource(localizer,goalPos);
    }

    public shootSpeedController(Flywheel flywheel, DoubleUnaryOperator equation, Follower localizer, Pose goalPos) {
        this.equation = equation;
        this.flywheel = flywheel;
        distanceSource = new PedroDistanceSource(localizer,goalPos);
    }
    public interface DistanceSource {
        double distanceToGoal();
        void setGoal(Object object);
    }
    public class NeuronDistanceSource implements DistanceSource {
        private final NeuronLocalizer localizer;
        private Pose2D goal;
        public NeuronDistanceSource(NeuronLocalizer localizer, Pose2D goal) {
            this.localizer = localizer;
            this.goal = goal;
        }
        public double distanceToGoal() {
            return localizer.getPose().distanceToPoint(goal);
        }

        public void setGoal(Object goal) {
            if (goal instanceof Pose2D pose2D) {
                this.goal = pose2D;
                return;
            }
            throw new RuntimeException();
        }
    }
    public class PedroDistanceSource implements DistanceSource {
        private final Follower follower;
        private Pose goal;
        public PedroDistanceSource(Follower follower, Pose goal) {
            this.follower = follower;
            this.goal = goal;
        }
        public double distanceToGoal() {
            return follower.localizer.pose().distance(goal);
        }
        public void setGoal(Object goal) {
            if (goal instanceof Pose pose) {
                this.goal = pose;
                return;
            }
            throw new RuntimeException();
        }
    }
}