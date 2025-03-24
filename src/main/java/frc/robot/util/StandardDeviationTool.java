package frc.robot.util;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class StandardDeviationTool {
    private int num_measurements;
    private double mean;
    private double sum;
    private ArrayList<Double> window;

    public StandardDeviationTool(int _num){
        this.num_measurements = _num;
        window = new ArrayList<>();
    }

    public void addSample(double sample){
        if(window.size() == num_measurements){
            sum -= window.remove(0);

        }
        window.add(sample);
        sum += sample;
        mean = sum/window.size();
    }

    public double getStdDeviation(){
        double _sum = 0;
        for(int i = 0; i < window.size(); i++){
            _sum += ((window.get(i) - mean) * (window.get(i) - mean));
        }
        _sum /= window.size();
        
        return Math.sqrt(Math.abs(_sum));
    }
}
