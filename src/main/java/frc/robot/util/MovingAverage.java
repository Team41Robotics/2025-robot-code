import java.util.Arrays;

public class MovingAverage{
    private int size;
    private int sum;
    private int index = 0;

    private double[] arr;

    public MovingAverage(int _size){
        this.size = _size;
        arr = new double[size];
        Arrays.fill(arr, 0)
    }

    public void add(double in){
        arr[index] = 0;
        
        if(++index == size){
            index = 0;
        }
    }

    public void clear(){
        index = 0;
        sum = 0;
        Arrays.fill(arr, 0);
    }

}