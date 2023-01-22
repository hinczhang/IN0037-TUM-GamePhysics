#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"
#include <vector>

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

#define FLOOR_LIMIT -0.5f
#define CEILING_LIMIT 0.5f
#define LEFT_WALL_LIMIT -0.5f
#define RIGHT_WALL_LIMIT 0.5f
#define FAR_WALL_LIMIT 0.5f
#define CLOSE_WALL_LIMIT -0.5f

// normal gravity squashes my cube :(
#define VEC3_GRAVITY Vec3(0, -0.81f, 0)

struct MassPoint {
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	bool isFixed;
	Vec3 acceleration;

	MassPoint(Vec3 position, Vec3 velocity, bool isFixed) : position{ position }, velocity{ velocity }, isFixed{ isFixed }, force{Vec3(0,0,0)} {}

	void print() {
		std::cout << "[p=(" << this->position.x << ", " << this->position.y << ", " << this->position.z << "), v=("
			<< this->velocity.x << ", " << this->velocity.y << ", " << this->velocity.z << "), f=("
			<< this->force.x << ", " << this->force.y << ", " << this->force.z << "), isFixed = "
			<< this->isFixed << "]" << endl;
	}
};

struct Spring {
public:
	int point0;
	int point1;
	//float stiffness;
	float initialLength;

	Spring(int point0, int point1, float initialLength) : 
		point0{ point0 }, point1{ point1 }, initialLength{ initialLength }{}

	void print() {
		std::cout << "[p0 = " << this->point0 << ", p1 = " << this->point1 
			<< ", L = " << this->initialLength << std::endl;
	}
};

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	bool m_collisions = false;
	bool m_gravity = false;
	std::vector<MassPoint> m_points;
	std::vector<Spring> m_springs;
	

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	float m_pontScaleVal = 0.05f;
	Vec3 m_pointScale = Vec3(m_pontScaleVal, m_pontScaleVal, m_pontScaleVal);
	Vec3 m_springColor = Vec3(0.0f, 0.5f, 1.0f);
	Vec3 m_externalForceColor = Vec3(0.0f, 1.0f, 0.2f);
	int m_integrator_ui = 0;

	void drawPoints();
	void drawSprings();
	void clearForces();
	Vec3 computeInternalForce(const MassPoint& p0, const MassPoint& p1, float restLength); 
	
	void euler(float step, bool gravity, bool collisions);
	void midpoint(float step, bool gravity, bool collisions);
	void leapfrog(float step, bool gravity, bool collisions);
	const char* getIntegratorStr();
	
	bool m_drawExternalForce = false;
	Vec3 m_externalForceDrawBegin;
	Vec3 m_externalForceDrawEnd;
};
#endif