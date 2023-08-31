package frc.robot.utils;

public class SwerveUtils {
     /**
     * Steps a value towards a target with a specified step size.
     * 
     * @param _current  The current or starting value. Can be positive or negative.
     * @param _target   The target value the algorithm will step towards. Can be
     *                  positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified
     *         step towards the specified target.
     */


    public static double StepToward(double _current, double _target, double _stepsize){
        if (Math.abs(_current - _target) <= _stepsize){
            return _target;
        }
        
        else if (_target < _current){
            return _current - _stepsize;
        }

        else{
            return _current + _stepsize;
        }
    }

    public static double AngleDistance(double _angleA, double _angleB){
        double difference = Math.abs(_angleA = _angleB);
        return difference > Math.PI ? (2 * Math.PI) - difference : difference;
    }














}
