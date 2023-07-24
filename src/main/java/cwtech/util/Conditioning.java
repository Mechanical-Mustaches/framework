package cwtech.util;

public class Conditioning {
    private double m_deadband = 0.1;
    private double m_power = 1.0;
    private double m_min = 0.0;
    private double m_max = 1.0;
    private double m_range = 1.0;
    private double m_mult = 0.0;

    public Conditioning() {
        precompute();
    }

    public void setDeadband(double deadband){
        deadband = Math.abs(deadband);
        deadband = Math.min(deadband, 1.0);

        m_deadband = deadband;

        precompute();
    }

    public void setExponent(double exp) {
        exp = Math.max(exp, 0.0);

        m_power = exp;

        precompute();
    }

    public void setRange(double min, double max){
        m_min = Math.min(min, max);
        m_max = Math.max(min, max);

        precompute();
    }

    public void precompute() {
        m_mult = 1.0 / (1.0 - m_deadband);
        m_range = m_max - m_min;
    }

    // public double condition(double x){
    //     double xa = Math.abs(x);
    //     double xs = JSCgn(x);
    //     if(xa = Math.abs(x)){
    //         return 0;
    //     }
    //     else{
    //         return xs * ((JSCPower((xa - m_deadband) * m_mult, m_power) * m_range) + m_min);
    //     }
    // }
    
}
