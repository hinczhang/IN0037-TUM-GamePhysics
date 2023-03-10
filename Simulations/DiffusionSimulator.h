#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

//impement your own grid class for saving grid data
class Grid {
public:
	// Construtors
	Grid() : Grid(16, 16) {};
	Grid(int rows, int cols) : rows{ rows }, cols{ cols } {

		data = new float* [rows];

		for (int i = 0; i < rows; i++)
			data[i] = new float[cols](); // zero-initialized
	}

	~Grid() {
		for (int i = 0; i < rows; i++) 
			delete[] data[i];
		delete[] data;
	}

	int rows, cols;
	float** data;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();
	~DiffusionSimulator() {
		delete T;
	}
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	Grid* diffuseTemperatureExplicit(float timeStep);
	void diffuseTemperatureImplicit(float timeStep);
	void resetT();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step
	unsigned int m_rows, m_cols;
	unsigned int nb_rand_points = 3;
	//float delta_x, delta_y;
	//double diffusion_coefficient = 9.4 * pow(10, -16);
	double diffusion_constant = 3; //combination of multiple values, like diffusion coeff, delta_x, delta_y
};

#endif