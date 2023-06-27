#include <iostream>
#include <windows.h>
#include <chrono>
#include <thread>

struct projectivePlane {
	// 1024 x 683
	double cdist;
	double cangle[2];	// 0 < cangle[0] < 2pi  -pi/2 < cangle[1] < pi/2
	double normal[3];
	double origin[3];
	double camera[3];
	double jacobian[6]; // { du/dx, du/dy, du/dz, dv/dx, dv/dy, dv/dz }
	double globalOffset[3];
};

double robot_body[24] = {
	-.15,-.15,-1, .15,-.15,-1, -.15,.15,-1, -.15,-.15,1,
	-.15,.15,1, .15,-.15,1, .15,.15,-1, .15,.15,1
};
double robot_arm[24] = {
	-.1,-.1,-.1, 1.4,-.1,-.1, -.1,.1,-.1, -.1,-.1,.1,
	-.1,.1,.1, 1.4,-.1,.1, 1.4,.1,-.1, 1.4,.1,.1
};
double robot_hand[24] = {
	1.2,-.06,-.3, 1.32,-.06,-.3, 1.2,.06,-.3, 1.2,-.06,-.1,
	1.2,.06,-.1, 1.32,-.06,-.1, 1.32,.06,-.3, 1.32,.06,-.1
};
double robot_base[24] = {
	-.3, -.3, -1.2,  .3, -.3, -1.2,  -.3, .3,-1.2, -.3,-.3,-1,
	-.3,.3,-1, .3,-.3,-1, .3,.3,-1.2, .3,.3,-1
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

	p->origin[0] -= 1.5 * p->jacobian[0];
	p->origin[1] -= 1.5 * p->jacobian[1];

	p->origin[0] -= p->jacobian[3];
	p->origin[1] -= p->jacobian[4];
	p->origin[2] -= p->jacobian[5];

	p->jacobian[0] *= 341;
	p->jacobian[1] *= 341;
	p->jacobian[3] *= 341;
	p->jacobian[4] *= 341;
	p->jacobian[5] *= 341;
}

bool projectivePlane_move(projectivePlane* p) {
	bool flag = 0;
	if (GetKeyState(VK_LEFT) & 0x8000) {
		p->cangle[0] += 0.03;
		flag = 1;
	}
	if (GetKeyState(VK_RIGHT) & 0x8000) {
		p->cangle[0] -= 0.03;
		flag = 1;
	}
	if (GetKeyState(VK_UP) & 0x8000) {
		p->cangle[1] += 0.03;
		flag = 1;
	}
	if (GetKeyState(VK_DOWN) & 0x8000) {
		p->cangle[1] -= 0.03;
		flag = 1;
	}
	if (GetKeyState('W') & 0x8000) {
		if (p->cdist > 0.05) {
			p->cdist -= 0.05;
			flag = 1;
		}
	}
	if (GetKeyState('S') & 0x8000) {
		p->cdist += 0.05;
		flag = 1;
	}
	/*
	if (GetKeyState('E') & 0x8000) {
		p->globalOffset[0] += 0.05;
		flag = 1;
	}
	if (GetKeyState('R') & 0x8000) {
		p->globalOffset[0] -= 0.05;
		flag = 1;
	}
	if (GetKeyState('D') & 0x8000) {
		p->globalOffset[1] += 0.05;
		flag = 1;
	}
	if (GetKeyState('F') & 0x8000) {
		p->globalOffset[1] -= 0.05;
		flag = 1;
	}
	if (GetKeyState('X') & 0x8000) {
		p->globalOffset[2] += 0.05;
		flag = 1;
	}
	if (GetKeyState('C') & 0x8000) {
		p->globalOffset[2] -= 0.05;
		flag = 1;
	}
	if (GetKeyState('Q') & 0x8000) {
		p->globalOffset[0] = 0;
		p->globalOffset[1] = 0;
		p->globalOffset[2] = 0;
		flag = 1;
	}
	*/
	if (GetKeyState('A') & 0x8000) {
		p->cangle[0] = 0;
		p->cangle[1] = 0;
		p->cdist = 5;
		flag = 1;
	}
	return flag;
}

void projectivePlane_map_fromR3(double* x, projectivePlane* pp, double* y) {
	double l[3];
	l[0] = pp->camera[0] - x[0];
	l[1] = pp->camera[1] - x[1];
	l[2] = pp->camera[2] - x[2];
	double d = (pp->origin[0] - x[0]) * pp->normal[0];
	d += (pp->origin[1] - x[1]) * pp->normal[1];
	d += (pp->origin[2] - x[2]) * pp->normal[2];
	d = d / (l[0] * pp->normal[0] + l[1] * pp->normal[1] + l[2] * pp->normal[2]);
	l[0] *= d;
	l[1] *= d;
	l[2] *= d;
	//d = l[0] * l[0] + l[1] * l[1] + l[2] * l[2] + 0.1;
	//d = 255 / sqrt(d+1);
	l[0] += x[0];
	l[1] += x[1];
	l[2] += x[2];
	// l is a point on the projection plane
	// convert to a tangent vector on the projection plane d/dx
	// then vector components are 1 forms
	// since there is a trivial map from TpP to ToP
	// substitute dz as zp - zo
	l[0] = l[0] - pp->origin[0];
	l[1] = l[1] - pp->origin[1];
	l[2] = l[2] - pp->origin[2];
	// du = du/dx dx + du/dy dy + du/dz dz
	y[0] = l[0] * pp->jacobian[0] + l[1] * pp->jacobian[1];
	y[1] = l[0] * pp->jacobian[3] + l[1] * pp->jacobian[4] + l[2] * pp->jacobian[5];
	return;
}

void screen_drawLine(double* a, double* b, int* frame) {
	int x = a[0];
	int y = a[1];
	x = x << 18;
	y = y << 18;
	double temp1 = (b[0] - a[0]);
	double temp2 = (b[1] - a[1]);
	double temp = temp1 * temp1 + temp2 * temp2;
	temp = sqrt(temp);
	temp1 = 0x40000 / temp;
	int v1 = (int)((b[0] - a[0]) * temp1);
	int v2 = (int)(temp2 * temp1);
	double i = 0;
	while ((i < temp)) {
		if ((x < 0x10000000) && (y < 179044352) && (x > 0) && (y > 0)) {
			frame[(x >> 18) + ((y >> 8) & 0xFFFFFC00)] = 0xFFFFFF; // >>18 then * screen width
		}
		x += v1;
		y += v2;
		i++;
	}
}

void hex_draw(double* a, int* frame) {

	//		  8		14
	//		6	  10
	// 
	//		  4		12
	//		0     2
	//

	screen_drawLine(a, a + 2, frame);
	screen_drawLine(a, a + 4, frame);
	screen_drawLine(a + 2, a + 12, frame);
	screen_drawLine(a + 4, a + 12, frame);

	screen_drawLine(a + 6, a + 8, frame);
	screen_drawLine(a + 6, a + 10, frame);
	screen_drawLine(a + 8, a + 14, frame);
	screen_drawLine(a + 10, a + 14, frame);

	screen_drawLine(a, a + 6, frame);
	screen_drawLine(a + 2, a + 10, frame);
	screen_drawLine(a + 4, a + 8, frame);
	screen_drawLine(a + 12, a + 14, frame);
}

void hex_project(double* ver3d, projectivePlane* p, double* ver2d) {
	projectivePlane_map_fromR3(ver3d, p, ver2d);
	projectivePlane_map_fromR3(ver3d + 3, p, ver2d + 2);
	projectivePlane_map_fromR3(ver3d + 6, p, ver2d + 4);
	projectivePlane_map_fromR3(ver3d + 9, p, ver2d + 6);
	projectivePlane_map_fromR3(ver3d + 12, p, ver2d + 8);
	projectivePlane_map_fromR3(ver3d + 15, p, ver2d + 10);
	projectivePlane_map_fromR3(ver3d + 18, p, ver2d + 12);
	projectivePlane_map_fromR3(ver3d + 21, p, ver2d + 14);
}

void robot_update_pos() {

}

int main()
{
	projectivePlane p;
	p.cangle[0] = 0;
	p.cangle[1] = 0;
	p.cdist = 5;
	p.globalOffset[0] = 0;
	p.globalOffset[1] = 0;
	p.globalOffset[2] = 0;

	HDC hdc = GetDC(GetConsoleWindow());
	HDC buf = CreateCompatibleDC(hdc);

	/*
	double** points;
	points = (double**)calloc(157 * 315, sizeof(double*));
	for (int i = 0; i < 157 * 315; i++) {
		points[i] = (double*)calloc(3, sizeof(double));
	}

	bool flag0 = 1;
	bool flag1 = 1;
	*/

	std::chrono::high_resolution_clock::time_point t1;
	std::chrono::high_resolution_clock::time_point t2;

	int* frameBuffer = (int*)calloc(1024 * 683, 4);
	HBITMAP hbitmap;

	p.cangle[0] = 0.5;
	p.cangle[1] = 0.4;
	p.cdist = 3;
	projectivePlane_update(&p);


	double hexah[16];

	bool flag1 = 1;
	
	while (1) {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

		if (flag1) {


			int* frameBuffer = (int*)calloc(1024 * 683, 4);
			HBITMAP hbitmap;

			projectivePlane_update(&p);
			flag1 = 0;

			hex_project(robot_body,&p,hexah);
			hex_draw(hexah,frameBuffer);
			hex_project(robot_arm, &p, hexah);
			hex_draw(hexah, frameBuffer);
			hex_project(robot_hand, &p, hexah);
			hex_draw(hexah, frameBuffer);
			hex_project(robot_base, &p, hexah);
			hex_draw(hexah, frameBuffer);

			hbitmap = CreateBitmap(1024, 683, 1, 32, (void*)frameBuffer);
			SelectObject(buf, hbitmap);
			BitBlt(hdc, 0, 0, 1024, 683, buf, 0, 0, SRCCOPY);
			DeleteObject(hbitmap);
			free(frameBuffer);
		}
		flag1 = projectivePlane_move(&p);

		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::microseconds(16666) - (t2 - t1));
	}
}