#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include <cstdlib>
#include <ctime>
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	
	m_rows = 16;
	m_cols = 16;

	T = new Grid(m_rows, m_cols);

	//delta_x = 1.0 / T->cols;
	//delta_y = 1.0 / T->rows;

	srand(time(NULL));
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Explicit - middle, Explicit - random, Explicit - unstable, "
		"Implicit_solver, Implicit - middle, Implicit - random, Implicit - stable";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	//TwAddVarRW(this->DUC->g_pTweakBar, "Grid size", TW_TYPE_INT32, &m_rows, "step=1 min=5 max=100");
	TwAddVarRW(this->DUC->g_pTweakBar, "Rows", TW_TYPE_INT32, &m_rows, "step=1 min=5 max=100");
	TwAddVarRW(this->DUC->g_pTweakBar, "Columns", TW_TYPE_INT32, &m_cols, "step=1 min=5 max=100");

	if (m_iTestCase == 2 || m_iTestCase == 6) {
		TwAddVarRW(this->DUC->g_pTweakBar, "Random points", TW_TYPE_INT32, &nb_rand_points, "step=1 min=1 max=10");
	}
}

void printT(Grid* T) {
	for (int i = 0; i < T->rows; i++) {
		for (int j = 0; j < T->cols; j++) {
			cout << T->data[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

void fillTRand(Grid* T, int n) {
	for (int point = 0; point < n; point++) {
		//generate a position randomly
		// I want the positions to be between 1 -> rows/cols - 2 (so I don't put points on the border)
		int row = (rand() % (T->rows - 2)) + 1;
		int col = (rand() % (T->cols - 2)) + 1;

		// generate value
		// I want this value to be between -1 and 1
		int sign = pow(-1, rand() % 2);
		float val = sign * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

		T->data[row][col] = val;
	}
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	
	delete T;
	//T = new Grid(m_rows, m_rows);
	T = new Grid(m_rows, m_cols);

	diffusion_constant = 3;
	
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		cout << "Simple showcase of the explicit solver, with a hot spot in the upper left corner\n\n";
		
		T->data[1][1] = 1;
		break;
	case 1:
		cout << "Explicit - middle\n";
		cout << "Simple showcase of the explicit solver, with a hot spot in the middle\n\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		break;
	case 2:
		cout << "Explicit - random\n";
		cout << "Generate a number of points randomly on the grid\n";
		cout << "WHITE - positive values, RED - negative values\n\n";
		fillTRand(T, nb_rand_points);
		break;
	case 3:
		cout << "Explicit - unstable\n";
		cout << "Explicit solver with a large diffusion constant (will be unstable)\n";
		cout << "The purpose of this is to compare it with the implicit solver (Implicit - stable)\n\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		diffusion_constant = 300;
		break;
	case 4:
		cout << "Implicit solver!\n";
		cout << "Simple showcase of the implicit solver, with a hot spot in the upper left corner\n\n";
		T->data[1][1] = 1;
		break;
	case 5:
		cout << "Implicit - middle\n";
		cout << "Simple showcase of the implicit solver, with a hot spot in the middle\n\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		break;
	case 6:
		cout << "Implicit - random\n";
		cout << "Generate a number of points randomly on the grid\n";
		cout << "WHITE - positive values, RED - negative values\n\n";
		fillTRand(T, nb_rand_points);
		break;
	case 7:
		cout << "Implicit - stable\n";
		cout << "Implicit solver with a large diffusion constant (should be stable)\n";
		cout << "The purpose of this is to compare it with the explicit solver (Explicit - unstable)\n\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		diffusion_constant = 300;
		break;

	default:
		cout << "Empty Test!\n";

		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {
	Grid* newT = new Grid(T->rows, T->cols);
	//make sure that the temperature in boundary cells stays zero
	
	for (int i = 1; i < T->rows - 1; i++) {
		for (int j = 1; j < T->cols - 1; j++) {

		//	float x_val = (T->data[i + 1][j] - 2 * T->data[i][j] + T->data[i - 1][j]) / (delta_x * delta_x);
		//	float y_val = (T->data[i][j + 1] - 2 * T->data[i][j] + T->data[i][j - 1]) / (delta_y * delta_y);
			float x_val = (T->data[i + 1][j] - 2 * T->data[i][j] + T->data[i - 1][j]);
			float y_val = (T->data[i][j + 1] - 2 * T->data[i][j] + T->data[i][j - 1]);


			newT->data[i][j] = T->data[i][j] + timeStep * diffusion_constant * (x_val + y_val);
		}
	}

	delete T;
	
	return newT;
}

void setupB(std::vector<Real>& b, int N, const Grid* T) {

	int k = 0;

	for (int i = 0; i < T->rows; i++) {
		for (int j = 0; j < T->cols; j++) {
			b.at(k++) = T->data[i][j];
		}
	}

	b.at(0) = 0;
	b.at(N - 1) = 0;
}

void fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
}

void setupA(SparseMatrix<Real>& A, double factor, int rows, int cols) {
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	int N = rows * cols;
	for (int i = 0; i < N; i++) {
			A.set_element(i, i, 1); // set diagonal
	}

	for (int row = 0; row < N; row++) {

		//extract row in T
		int i = row / cols;

		//extract column in T
		int j = row % cols;

		//if on boundary
		if (i == 0 || i == rows - 1 || j == 0 || j == cols - 1)
			continue;

		//set elemenets on the row relative to the original indices in the T matrix
		A.set_element(row, (i - 1) * cols + j, -factor);
		A.set_element(row, (i + 1) * cols + j, -factor);
		A.set_element(row, i * cols + j, 1 + 4 * factor);
		A.set_element(row, i * cols + (j - 1), -factor);
		A.set_element(row, i * cols + (j + 1), -factor);
	}

}

void DiffusionSimulator::resetT() {
	for (int i = 0; i < T->rows; i++) {
		for (int j = 0; j < T->cols; j++) {
			T->data[i][j] = 0;
		}
	}
}

void DiffusionSimulator::diffuseTemperatureImplicit(float timeStep) {//add your own parameters
	// solve A T = b
	const int N = T->rows * T->cols;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, diffusion_constant * timeStep, T->rows, T->cols);
	setupB(*b, N, T);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	//fillT();//copy x to T

	int k = 0;
	for (int i = 0; i < T->rows; i++) {
		for (int j = 0; j < T->cols; j++) {
			T->data[i][j] = x[k++];
		}
	}


	//set boundary to 0
	for (int i = 0; i < T->rows; i++) {
		for (int j = 0; j < T->cols; j++) {
			if(i == 0 || j == 0 || i == T->rows - 1 || j == T->cols - 1)
				T->data[i][j] = 0;
		}
	}

	//printT(T);

	delete b;
	delete A;
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0: [[fallthrough]]
	case 1: [[fallthrough]]
	case 2: [[fallthrough]]
	case 3:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 4: [[fallthrough]]
	case 5: [[fallthrough]]
	case 6: [[fallthrough]]
	case 7: 
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	constexpr float x_step = 0.04;
	constexpr float y_step = 0.04;
	constexpr float sphere_scale = 0.02;

	
	for (int i = 0; i < T->rows; i++) {
		for (int j = 0; j < T->cols; j++) {

			float val = T->data[i][j];

			if (i == 0 || j == 0 || i == T->rows - 1 || j == T->cols - 1) {
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0.2, 0.2, 0.7));	
			}
			else {

				if (val > 0) {
					DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 50 * Vec3(val, val, val));
				}
				else {
					DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 50 * Vec3(-val, 0, 0));
				}
			}
			
			
			Vec3 position((double)x_step * j - 0.5, -(double)y_step * i + 0.5, 0);
			DUC->drawSphere(position, sphere_scale);
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
