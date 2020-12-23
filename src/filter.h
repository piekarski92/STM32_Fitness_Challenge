#ifndef FILTER_H_INCLUDED
#define FILTER_H_INCLUDED

class CanonFilter{
    private:
        const static double b[3];
        const static double a[3];
        double y0;
        double w0;
        double w1;
        double w2;
    public:
        double filterData(double x0);
        void resetStates();
};

#endif 