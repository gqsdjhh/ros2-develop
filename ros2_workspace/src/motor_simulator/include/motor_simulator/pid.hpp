#pragma once

namespace pid {

class Pid {
public:
    Pid(float maxOutput = 1000.0f, float minOutput = -1000.0f, float maxIntegral = 500.0f, float minIntegral = -500.0f)
        : _max_output(maxOutput), _min_output(minOutput),_max_integral(maxIntegral), _min_integral(minIntegral) {};
    ~Pid() = default;
    
    void SetPid(float p, float i, float d){
        _p = p;
        _i = i;
        _d = d;
    }

    void SetOutputLimits(float max, float min){
        _max_output = max;
        _min_output = min;
    }
    
    void SetIntegralLimits(float max, float min){
        _max_integral = max;
        _min_integral = min;
    }
    
    void SetTarget(float target){
        _target = target;
    }

    virtual float PidCal(float input) = 0;
    
    void reset(){
        _target = 0.0f;
        _now = 0.0f;
        _error[0] = _error[1] = _error[2] = 0.0f;
        _pout = _iout = _dout = 0.0f;
        _out = 0.0f;
    }

protected:
    float _target = 0.0f;
	float _now = 0.0f;
	float _error[3] = {0.0f, 0.0f, 0.0f};
	float _p = 0.0f, _i = 0.0f, _d = 0.0f;
	float _pout = 0.0f, _dout = 0.0f, _iout = 0.0f;
	float _out = 0.0f;
    float _max_output = 0.0f;    
    float _min_output = 0.0f;    
    float _max_integral = 0.0f;  
    float _min_integral = 0.0f;  

    void LimitIntegral() {
        if (_iout > _max_integral) {
            _iout = _max_integral;
        } else if (_iout < _min_integral) {
            _iout = _min_integral;
        }
    }
    
    void LimitOutput() {
        if (_out > _max_output) {
            _out = _max_output;
        } else if (_out < _min_output) {
            _out = _min_output;
        }
    }
    
    void UpdateError(float error) {
        _error[2] = _error[1];  
        _error[1] = _error[0];  
        _error[0] = error;  
    }
};

class PositionPid : public Pid {
public:
    using Pid::Pid;  
    
    float PidCal(float input) override {
        _now = input;
        float error = _target - _now;
        UpdateError(error);
        
        _pout = _p * _error[0];
        _iout += _i * _error[0];
        _dout = _d * (_error[0] - _error[1]);
        LimitIntegral();  // 积分限幅
        
        _out = _pout + _iout + _dout;
        LimitOutput();
        
        return _out;
    }
};

class DeltaPid : public Pid {
public:
    using Pid::Pid;
    
    float PidCal(float input) override {
        _now = input;
        float error = _target - _now;
        UpdateError(error);
        
        _pout = _p * (_error[0] - _error[1]);
        _iout = _i * _error[0];
        _dout = _d * (_error[0] - 2 * _error[1] + _error[2]);
        
        _out += _pout + _iout + _dout;
        
        LimitOutput();
               
        return _out;
    }
};

} // namespace pid