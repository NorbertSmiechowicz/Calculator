#include <iostream>
#include <math.h>

double derivative(double x, double (*f)(double x), unsigned char order) {
	if (order == 0) return f(x);
	if (order == 1) return 100000 * (f(x + 0.00001) - f(x));
	return 100000*(derivative(x+0.00001,f,order-1)- derivative(x, f, order - 1));
}

double integral(double x, double(*f)(double x), unsigned char order) {
	double t = 0;
	double s = 0;
	double dt = x / 500;
	if (order == 0) {
		return f(x) - f(0);
	}
	if (order == 1) {
		while (t < x) {
			s += dt * f(t);
			t += dt;
		}
		return s;
	}
	while (t < x) {
		s += dt * integral(t, f, order - 1);
		t += dt;
	}
	return s;
}

double integral_line(double t0, double t1, double (**f)(double* x), double (**x)(double x), int dim) {
	double dt = t1 - t0;
	dt *= 0.00001;
	double s = 0;
	double* p = new double[dim];
	while (t0 < t1) {
		for (int i = 0; i < dim; i++) {
			p[i] = x[i](t0);
		}		
		for (int i = 0; i < dim; i++) {
			s += dt * derivative(t0, x[i], 1) * f[i](p);
		}
		t0 += dt;
	}
	return s;
}

double F_zero(double* x) {
	return 0.0;
}

double F_x1(double* x) {
	return x[1];
}

double F_x0(double* x) {
	return x[0];
}

double f_zero(double x) {
	return 0.0;
}

int main()
{
	double z = 0;
	double (*param[3])(double x);
	param[0] = sin;
	param[1] = cos;
	param[2] = f_zero;

	double (*form[3])(double* x);
	form[0] = F_x1;
	form[1] = F_x0;
	form[2] = F_zero;

	std::cout << integral_line(0, 3.1425/4,form,param,3);
}
