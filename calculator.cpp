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

struct projectivePlane{
	// 1200 x 800 / => du * dx = 400
	double cdist;
	double cangle[2];	// 0 < cangle[0] < 2pi  -pi/2 < cangle[1] < pi/2
	double normal[3];
	double origin[3];
	double camera[3];
	double jacobian[6]; // { du/dx, du/dy, du/dz, dv/dx, dv/dy, dv/dz }
};

void projectivePlane_update(projectivePlane* p) {
	// degeneracies exist when either psi or phi are multiples of (pi/2)
	if (p->cangle[0] < 0.01) p->cangle[0] = 0.01;
	if ((p->cangle[0] > 1.57) && (p->cangle[0] < 1.58)) p->cangle[0] = 1.58;
	if ((p->cangle[0] > 3.14) && (p->cangle[0] < 3.15)) p->cangle[0] = 3.15;
	if ((p->cangle[0] > 4.71) && (p->cangle[0] < 4.72)) p->cangle[0] = 4.72;
	if (p->cangle[0] > 6.28) p->cangle[0] = 6.28;

	if (p->cangle[1] < -1.57) p->cangle[1] = -1.57;
	if ((p->cangle[1] > -0.01) && (p->cangle[1] < 0.01)) p->cangle[1] = 0.01;
	if (p->cangle[1] > 1.57) p->cangle[1] = 1.57;

	// temporarily, camera only looks at 0 
	// if updated, then coordinate functions change
	p->normal[0] = cos(p->cangle[0]) * cos(p->cangle[1]);
	p->normal[1] = sin(p->cangle[0]) * cos(p->cangle[1]);
	p->normal[2] = sin(p->cangle[1]);

	p->jacobian[0] = -sin(p->cangle[0]);
	p->jacobian[1] = cos(p->cangle[0]);
	p->jacobian[2] = 0;
	p->jacobian[3] = cos(p->cangle[0]) * sin(p->cangle[1]);
	p->jacobian[4] = sin(p->cangle[0]) * sin(p->cangle[1]);
	p->jacobian[5] = -cos(p->cangle[1]);

	// origin is 2 global units towards global origin starting from camera,
	// then 1.5 global units in direction of - du,
	// then 1 global units in the direction of -dv
	// assuming 3x2 global unit plane, camera is now in center
	p->camera[0] = p->cdist * p->normal[0];
	p->camera[1] = p->cdist * p->normal[1];
	p->camera[2] = p->cdist * p->normal[2];

	p->origin[0] = p->camera[0] - p->normal[0] - p->normal[0];
	p->origin[1] = p->camera[1] - p->normal[1] - p->normal[1];
	p->origin[2] = p->camera[2] - p->normal[2] - p->normal[2];

	p->origin[0] -= 1.5*p->jacobian[0];
	p->origin[1] -= 1.5*p->jacobian[1];

	p->origin[0] -= p->jacobian[3];
	p->origin[1] -= p->jacobian[4];
	p->origin[2] -= p->jacobian[5];
	
	p->jacobian[0] *= 400;
	p->jacobian[1] *= 400;
	p->jacobian[3] *= 400;
	p->jacobian[4] *= 400;
	p->jacobian[5] *= 400;
}

void map_R3_Screen(double* x, projectivePlane* pp,  int* frame) {

	double l[3];
	l[0] = pp->camera[0] - x[0];
	l[1] = pp->camera[1] - x[1];
	l[2] = pp->camera[2] - x[2];
	double d = (pp->origin[0] - x[0]) * pp->normal[0];
	d += (pp->origin[1] - x[1]) * pp->normal[1];
	d += (pp->origin[2] - x[2]) * pp->normal[2];
	d = d/( l[0] * pp->normal[0] + l[1] * pp->normal[1] + l[2] * pp->normal[2] );
	l[0] *= d;
	l[1] *= d;
	l[2] *= d;
	d = l[0] * l[0] + l[1] * l[1] + l[2] * l[2] + 0.1;
	d = 255 / d;
	l[0] += x[0];
	l[1] += x[1];
	l[2] += x[2];
	// l is a point on the projection plane
	// convert to a tangent vector on the projection plane d/dx
	// then vector components are 1 forms dx
	// since there is a trivial linear map from TpP to ToP
	// substitute dz as zp - zo and the structre is preserved;
	l[0] = l[0] - pp->origin[0];
	l[1] = l[1] - pp->origin[1];
	l[2] = l[2] - pp->origin[2];
	// du = du/dx dx + du/dy dy + du/dz dz
	x[0] = l[0] * pp->jacobian[0] + l[1] * pp->jacobian[1]; 
	x[1] = l[0] * pp->jacobian[3] + l[1] * pp->jacobian[4] + l[2] * pp->jacobian[5];
	if (x[0] > 1200) return;
	if (x[0] < 0) return;
	if (x[1] > 800) return;
	if (x[1] < 0) return;
	frame[(int)x[0]+ 1200 * (int)x[1]] = 0xFF0080;
}

int main()
{
	projectivePlane p;
	p.cangle[0] = 0;
	p.cangle[1] = 0;
	p.cdist = 5;
	
	HDC hdc = GetDC(GetConsoleWindow());
	HDC buf = CreateCompatibleDC(hdc);
	int* frameBuffer = (int*)calloc(1200*800, sizeof(COLORREF));
	
	double x[3];
	bool flag = 1;

	while (1) {
		if (flag) {
			HBITMAP hbitmap;
			for (int i = 0; i < 1200 * 800; i++) {
				frameBuffer[i] = 0;
			}
			projectivePlane_update(&p);
			flag = 0;
			for (double i = 0; i < 6.28; i += 0.02) {
				for (double j = 0; j < 6.28; j += 0.04) {
					x[0] = cos(j) * (2 + cos(i));
					x[1] = sin(j) * (2 + cos(i));
					x[2] = sin(i);
					map_R3_Screen(x, &p, frameBuffer);
				}
			}

			hbitmap = CreateBitmap(1200, 800, 1, 32, (void*)frameBuffer);
			SelectObject(buf, hbitmap);
			BitBlt(hdc, 0, 0, 1200, 800, buf, 0, 0, SRCCOPY);
			DeleteObject(hbitmap);
		}

		if (GetKeyState(VK_LEFT) & 0x8000) {
			p.cangle[0] += 0.03;
			flag = 1;
		}
		if (GetKeyState(VK_RIGHT) & 0x8000) {
			p.cangle[0] -= 0.03;
			flag = 1;
		}
		if (GetKeyState(VK_UP) & 0x8000) {
			p.cangle[1] -= 0.03;
			flag = 1;
		}
		if (GetKeyState(VK_DOWN) & 0x8000) {
			p.cangle[1] += 0.03;
			flag = 1;
		}
		if (GetKeyState('W') & 0x8000) {
			if (p.cdist > 0.05) {
				p.cdist -= 0.05;
				flag = 1;
			}
		}
		if (GetKeyState('S') & 0x8000) {
			p.cdist += 0.05;
			flag = 1;
		}
	};
}
