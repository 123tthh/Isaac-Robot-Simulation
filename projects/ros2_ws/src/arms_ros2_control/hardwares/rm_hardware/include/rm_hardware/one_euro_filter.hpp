#pragma once

#include <cmath>
#include <stdexcept>
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

// -----------------------------------------------------------------

class LowPassFilter {
    double y, a, s;
    bool initialized;

    void setAlpha(double alpha) {
        if (alpha <= 0.0 || alpha > 1.0)
            throw std::invalid_argument("alpha should be in (0.0, 1.0]");
        a = alpha;
    }

    public:
    LowPassFilter(double alpha, double initval = 0.0) {
        setAlpha(alpha);
        initialized = false;
        y = s = initval;
    }

    double filter(double value) {
        double result;
        if (initialized)
            result = a * value + (1.0 - a) * s;
        else {
            result = value;
            initialized = true;
        }
        y = value;
        s = result;
        return result;
    }

    double filterWithAlpha(double value, double alpha) {
        setAlpha(alpha);
        return filter(value);
    }

    bool hasLastRawValue(void) { return initialized; }

    double lastRawValue(void) { return y; }
};

// -----------------------------------------------------------------

class OneEuroFilter {
    double freq;
    double mincutoff;
    double beta_;
    double dcutoff;
    LowPassFilter* x;
    LowPassFilter* dx;
    double lasttime;

    double alpha(double cutoff) {
        double te = 1.0 / freq;
        double tau = 1.0 / (2 * M_PI * cutoff);
        return 1.0 / (1.0 + tau / te);
    }

    void setFrequency(double f) {
        if (f <= 0) throw std::invalid_argument("freq should be >0");
        freq = f;
    }

    void setMinCutoff(double mc) {
        if (mc <= 0) throw std::invalid_argument("mincutoff should be >0");
        mincutoff = mc;
    }

    void setBeta(double b) { beta_ = b; }

    void setDCutoff(double dc) {
        if (dc <= 0) throw std::invalid_argument("dcutoff should be >0");
        dcutoff = dc;
    }

    public:
    OneEuroFilter(double freq, double mincutoff = 1.0, double beta_ = 0.0,
        double dcutoff = 1.0) {
        setFrequency(freq);
        setMinCutoff(mincutoff);
        setBeta(beta_);
        setDCutoff(dcutoff);
        x = new LowPassFilter(alpha(mincutoff));
        dx = new LowPassFilter(alpha(dcutoff));
        lasttime = -1.0;
    }

    double filter(double value, double timestamp = -1.0) {
        // update the sampling frequency based on timestamps
        if (lasttime != -1.0 && timestamp != -1.0)
            freq = 1.0 / (timestamp - lasttime);
        lasttime = timestamp;
        // estimate the current variation per second
        double dvalue =
            x->hasLastRawValue() ? (value - x->lastRawValue()) * freq : 0.0;
        double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));
        // use it to update the cutoff frequency
        double cutoff = mincutoff + beta_ * std::abs(edvalue);
        // filter the given value
        return x->filterWithAlpha(value, alpha(cutoff));
    }

    ~OneEuroFilter(void) {
        delete x;
        delete dx;
    }
};
