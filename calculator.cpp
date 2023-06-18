#include <iostream>
#include <Windows.h>
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


unsigned int map_R3_R3embdPlane(double* x, double* p, double* y) {
	// plane eq: (p - p0) dot n = 0
	// p(arameters): 0,1,2 - point on the plane; 3,4,5 - normal vector; 6,7,8 - camera
	double d = (p[0] - x[0]) * p[3] + (p[1] - x[1]) * p[4] + (p[2] - x[2]) * p[5];
	y[0] = p[6] - x[0];
	y[1] = p[7] - x[1];
	y[2] = p[8] - x[2];
	d = d / (y[0] * p[3] + y[1] * p[4] + y[2] * p[5]);
	y[0] *= d;
	y[1] *= d;
	y[2] *= d;
	d = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
	y[0] += x[0];
	y[1] += x[1];
	y[2] += x[2];
	//y is now a point on the embedded plane in R3
	//d is distance squared to the porjective plane
	return (int)d;
}
void map_R3embdPlane_frameBuffer(double* x, double* p, unsigned int d, int* fb) {
	// p(arameters) : 0,1 - local origin in x,y; 2,3,4,5 - [du/dx]
	double u = p[2] * (x[0] - p[0]) + p[3] * (x[1] - p[1]);
	double v = p[4] * (x[0] - p[0]) + p[5] * (x[1] - p[1]);
	if (u < 0) return;
	if (v < 0) return;
	if (u > 1280.0) return;
	if (v > 720.0) return;
	int temp = (int)u + ((int)v * 1280.0);
	//if(d<fb[temp]) fb[temp] = d & 0xFFFFFF;
	fb[temp] = 0xFFFFFF;
}
void map_R2_R3(double* x, double (**f)(double* x), double* y) {
	y[0] = f[0](x);
	y[1] = f[1](x);
	y[2] = f[2](x);
}


/*
double f_torus_x(double* x) {
	return cos(x[0]);// *(3 + cos(x[1]));
}
double f_torus_y(double* x) {
	return sin(x[0]);// *(3 + cos(x[1]));
}
double f_torus_z(double* x) {
	return 0.0;
}
*/
double f_torus_x(double* x) {
	return cos(x[0])*cos(x[1]);
}
double f_torus_y(double* x) {
	return sin(x[0])*cos(x[1]);
}
double f_torus_z(double* x) {
	return sin(x[1]);
}

int main()
{
	double z = 0;
	double (*fn[3])(double* x);
	fn[0] = f_torus_x;
	fn[1] = f_torus_y;
	fn[2] = f_torus_z;

	double x[3] = { 0,0,2 };
	double projPlane[9] = { 0,0,-2,0,0,1,1,1,-1 };
	double y[3] = { 0,0,0 };

	double transfromPlane[6] = {-1.5,4,720/4,0,0,-720/4};
	unsigned int d = 0;

	HDC hdc = GetDC(GetConsoleWindow());
	HDC buf = CreateCompatibleDC(hdc);
	COLORREF* frameBuffer = (COLORREF*)calloc(1280*720, sizeof(COLORREF));
	HBITMAP hbitmap;
	
	for (double i = 0; i < 3.1425; i += 0.005) {
		for (double j = 0; j < 3.1425; j += 0.005) {
			y[0] = i;
			y[1] = j;
			map_R2_R3(y,fn,x);
			d = map_R3_R3embdPlane(x,projPlane,y);
			map_R3embdPlane_frameBuffer(y, transfromPlane, d, (int*)frameBuffer);
		}
	}

	hbitmap = CreateBitmap(1280, 720, 1, 32, (void*)frameBuffer);
	SelectObject(buf, hbitmap);
	BitBlt(hdc, 0, 0, 1280, 720, buf, 0, 0, SRCCOPY);
	while (1);
}
