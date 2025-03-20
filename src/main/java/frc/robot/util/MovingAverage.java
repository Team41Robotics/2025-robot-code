package frc.robot.util;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {
	private int size;
	private double sum;

	private Queue<Double> window;

    public MovingAverage(int _size){
        this.size = _size;
        this.window = new LinkedList<Double>();
        this.sum = 0.0;
    }

    public void add(double value){
        if(window.size() == size){
            sum -= window.poll();
        }
        window.add(value);
        sum += value;
    }

    public double getAverage(){
        return sum / window.size();
    }

    public void empty(){
        window.clear();
    }

}
