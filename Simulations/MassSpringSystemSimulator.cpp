#include "MassSpringSystemSimulator.h"

// --------------- CONSTRUCTORS --------------- //
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
    m_iTestCase = 0;
    m_fDamping = 0.0f;
    m_fMass = 1.0f;
    m_fStiffness = 10.0f;
    m_externalForce = Vec3(0, 0, 0);
}

// --------------- UI FUNCTIONS --------------- //
const char* MassSpringSystemSimulator::getTestCasesStr()
{
    return "Demo 1 (printing), Demo 2 (Euler), Demo 3 (Midpoint), Demo 4 (Complex), Demo 5 (Leapfrog), Test space :)";
}

const char* MassSpringSystemSimulator::getIntegratorStr() {
    return "Euler, Leapfrog, Midpoint";
}

// This function adds sliders in the ui
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;

    switch (m_iTestCase)
    {
    case 5: [[fallthrough]];
    case 3: {
        TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Integrator", getIntegratorStr());
        TwAddVarRW(this->DUC->g_pTweakBar, "Integrator", TW_TYPE_TESTCASE, &m_integrator_ui, "");
        TwAddVarRW(this->DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "step=0.01 min=0.0");
        TwAddVarRW(this->DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "step=0.1 min=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "step=0.1 min=1");
    } break;
    default: break;
    }
    
}

void MassSpringSystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawPoints() {
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(1, 0, 0));

    for (const MassPoint& p : m_points) {
        DUC->drawSphere(p.position, m_pointScale);
    }
}

void MassSpringSystemSimulator::drawSprings() {
    DUC->beginLine();
    for (const Spring& s : m_springs) {
        DUC->drawLine(m_points[s.point0].position, m_springColor,
            m_points[s.point1].position, m_springColor);
    }
    DUC->endLine();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    drawPoints();
    drawSprings(); 
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
    m_points.clear();
    m_springs.clear();

    m_iTestCase = testCase;
    switch (m_iTestCase)
    {
    case 0: {
        cout << "Demo 1!\n";
        //set demo constants of points
        for (int i = 0; i <= 1; i++) {
            setMass(10.0f);
            setStiffness(40.0f);
            setDampingFactor(0.0f);
            m_collisions = false;
            m_gravity = false;

            //add points
            int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
            int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

            //add springs
            addSpring(p0, p1, 1);

            if (i == 0) {
                // do Euler
                setIntegrator(EULER);
                euler(0.1f, false, false);
                std::cout << "After the first Euler step, we have the following positions: " << std::endl;
            }
            else {
                setIntegrator(MIDPOINT);
                midpoint(0.1f, false, false);
                std::cout << "After the first Midpoint step, we have the following positions: " << std::endl;
            }

            std::cout << "p0: position= " << m_points[p0].position << ", velocity=" << m_points[p0].velocity << std::endl;
            std::cout << "p1: position= " << m_points[p1].position << ", velocity=" << m_points[p1].velocity << std::endl;
            std::cout << std::endl;

            m_points.clear();
            m_springs.clear();
        }



    }break;
    case 1: {
        cout << "Demo 2!\n";
        // set constants
        setMass(10.0f);
        setStiffness(40.0f);
        setDampingFactor(0.0f);
        
        m_collisions = false;
        m_gravity = false;
        setIntegrator(EULER);

        int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
        int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);


        addSpring(p0, p1, 1);
    } break;
    case 2: {
        cout << "Demo 3!\n";
        // set constants
        setMass(10.0f);
        setStiffness(40.0f);
        setDampingFactor(0.0f);

        m_collisions = false;
        m_gravity = false;
        setIntegrator(MIDPOINT);

        int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
        int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
        addSpring(p0, p1, 1);
    }break;
    case 3: {
        cout << "Demo 4!\n";
        cout << "INSTRUCITONS: Hold left mouse button and drag to apply external force!\n";
        setMass(10.0f);
        setStiffness(100.0f);
        setDampingFactor(0.3f);
        m_collisions = true;
        m_gravity = true;

        int p0 = addMassPoint(Vec3(0.25, 0.25, 0.25), Vec3(0, 0, 0), false);
        int p1 = addMassPoint(Vec3(0.25, 0.25, -0.25), Vec3(0, 0, 0), false);
        int p2 = addMassPoint(Vec3(-0.25, 0.25, 0.25), Vec3(0, 0, 0), false);
        int p3 = addMassPoint(Vec3(-0.25, 0.25, -0.25), Vec3(0, 0, 0), false);
        int p4 = addMassPoint(Vec3(0.25, -0.25, 0.25), Vec3(0, 0, 0), false);
        int p5 = addMassPoint(Vec3(0.25, -0.25, -0.25), Vec3(0, 0, 0), false);
        int p6 = addMassPoint(Vec3(-0.25, -0.25, 0.25), Vec3(0, 0, 0), false);
        int p7 = addMassPoint(Vec3(-0.25, -0.25, -0.25), Vec3(0, 0, 0), false);

        int p8 = addMassPoint(Vec3(-0.5, 0.5, 0.5), Vec3(0, 0, 0), false);
        int p9 = addMassPoint(Vec3(0, 0.5, 0), Vec3(0, 0, 0), true);

        addSpring(p8, p3, 0.25);
        addSpring(p9, p8, 0.25);

        //diagonals
        addSpring(p0, p5, 0.707107);
        addSpring(p1, p4, 0.707107);

        addSpring(p0, p3, 0.707107);
        addSpring(p1, p2, 0.707107);

        addSpring(p0, p6, 0.707107);
        addSpring(p2, p4, 0.707107);

        addSpring(p6, p5, 0.707107);
        addSpring(p4, p7, 0.707107);

        addSpring(p3, p6, 0.707107);
        addSpring(p2, p7, 0.707107);

        addSpring(p3, p5, 0.707107);
        addSpring(p1, p7, 0.707107);


        //sides
        addSpring(p0, p1, 0.5);
        addSpring(p2, p3, 0.5);
        addSpring(p0, p2, 0.5);
        addSpring(p1, p3, 0.5);

        addSpring(p4, p5, 0.5);
        addSpring(p6, p7, 0.5);
        addSpring(p4, p6, 0.5);
        addSpring(p5, p7, 0.5);

        addSpring(p0, p4, 0.5);
        addSpring(p1, p5, 0.5);
        addSpring(p2, p6, 0.5);
        addSpring(p3, p7, 0.5);

        //space diagonals
        addSpring(p0, p7, 0.8660254f);
        addSpring(p1, p6, 0.8660254f);
        addSpring(p2, p5, 0.8660254f);
        addSpring(p3, p4, 0.8660254f);

    } break;
    case 4: {
          cout << "Demo 5!\n";
          setMass(10.0f);
          setStiffness(40.0f);
          setDampingFactor(0.0f);

          m_collisions = false;
          m_gravity = false;
          setIntegrator(LEAPFROG);

          int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
          int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

          addSpring(p0, p1, 1);
    } break;
    case 5: {
        cout << "Test space :)\n";
        setMass(10.0f);
        setStiffness(20.0f);
        setDampingFactor(0.0f);
        m_collisions = false;
        m_gravity = true;
        
        int p0 = addMassPoint(Vec3(0, 0.5, 0), Vec3(0, 0, 0), true);
        int p1 = addMassPoint(Vec3(0.5, 0.5, 0), Vec3(0, 0, 0), true);
        int p2 = addMassPoint(Vec3(0.0, 0.0, 0), Vec3(0, 0, 0), false);
        int p3 = addMassPoint(Vec3(0.5, 0.0, 0), Vec3(0, 0, 0), false);

        addSpring(p0, p2, 0.2);
        addSpring(p1, p3, 0.2);
        addSpring(p2, p3, 0.3);
    } break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
        // find a proper scale!
        float inputScale = 0.001f;
        inputWorld = inputWorld * inputScale;

        m_externalForce = inputWorld * 70;
        m_externalForce.y *= -1;
        m_externalForce.x *= -1;
    }
    else {
        m_externalForce = (0, 0, 0);
    }
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
    
    // update integrator for cases when we have the UI slider and apply external force
    if (m_iTestCase == 3 || m_iTestCase == 5) {
        applyExternalForce(m_externalForce);
        m_iIntegrator = m_integrator_ui;
    }

    switch (m_iIntegrator) {
    case EULER:
        euler(timeStep, m_gravity, m_collisions);
        break;
    case MIDPOINT:
        midpoint(timeStep, m_gravity, m_collisions);
        break;
    case LEAPFROG:
        leapfrog(timeStep, m_gravity, m_collisions);
    default: break;
    }
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}


// -------------- SPECIFIC FUNCTIONS -------------- //

void MassSpringSystemSimulator::setMass(float mass)
{
    m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
    m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
    m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
    int index = m_points.size();
    m_points.emplace_back(position, Velocity, isFixed);

    return index;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
    m_springs.emplace_back(masspoint1, masspoint2, initialLength);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
    return m_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
    return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
    return m_points[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
    return m_points[index].velocity;
}

void MassSpringSystemSimulator::clearForces() {
    for (MassPoint& p : m_points) {
        p.force = Vec3(0, 0, 0);
    }
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
    for (MassPoint& p : m_points) {
        p.force += m_externalForce;
    }
}



Vec3 MassSpringSystemSimulator::computeInternalForce(const MassPoint& p0,const MassPoint& p1, float restLength) {
    float currLen = norm(p0.position - p1.position);
    return -m_fStiffness * (currLen - restLength) * (p0.position - p1.position) / currLen;
}

void doCollision(MassPoint& p) {
    if (p.position.y < FLOOR_LIMIT) p.position.y = FLOOR_LIMIT;
    if (p.position.y > CEILING_LIMIT) p.position.y = CEILING_LIMIT;
    if (p.position.x < LEFT_WALL_LIMIT) p.position.x = LEFT_WALL_LIMIT;
    if (p.position.x > RIGHT_WALL_LIMIT) p.position.x = RIGHT_WALL_LIMIT;
    if (p.position.z < CLOSE_WALL_LIMIT) p.position.z = CLOSE_WALL_LIMIT;
    if (p.position.z > FAR_WALL_LIMIT) p.position.z = FAR_WALL_LIMIT; 

}

void MassSpringSystemSimulator::euler(float step, bool gravity, bool collision) {

    //compute new forces
    for (const Spring& s : m_springs) {
        // compute internal force for p0
        Vec3 internal_force = computeInternalForce(m_points[s.point0], m_points[s.point1], s.initialLength);
        m_points[s.point0].force += internal_force;
        // for p1, force in opposite direction
        m_points[s.point1].force -= internal_force;
    }

    //compute new positions
    for (MassPoint& p : m_points) {
        if (p.isFixed)
            continue;
        // compute new position
        p.position += step * p.velocity;

        if (collision) {
            doCollision(p);
        }

        // compute acceleration for p
        Vec3 acc = (p.force - m_fDamping * p.velocity) / m_fMass;

        if (gravity)
            acc += VEC3_GRAVITY;

        //compute next velocity
        p.velocity += step * acc;
    }

    //clear forces for the next step
    clearForces();

}

void MassSpringSystemSimulator::midpoint(float step, bool gravity, bool collision) {

    // take half an euler step
    std::vector<MassPoint> midpoints = m_points;
    for (MassPoint& mid : midpoints) {
        if (mid.isFixed) continue;
        mid.position += (step / 2) * mid.velocity;

        if (collision)
            doCollision(mid);
    }

    // compute forces at new position
    for (const Spring& s : m_springs) {
        // compute internal force for p0
        Vec3 internal_force = computeInternalForce(midpoints[s.point0], midpoints[s.point1], s.initialLength);
        midpoints[s.point0].force += internal_force;
        // for p1, force in opposite direction
        midpoints[s.point1].force -= internal_force;
    }

    //compute new positions
    for (MassPoint& m : midpoints) {
        if (m.isFixed)
            continue;

        // compute acceleration for m
        Vec3 acc = (m.force - m_fDamping * m.velocity) / m_fMass;

        if (gravity)
            acc += VEC3_GRAVITY;

        m.acceleration = acc;
        //compute next velocity
        m.velocity += (step/2) * acc;
    }

    //update actual values
    for (int i = 0; i < m_points.size(); i++) {
        if (m_points[i].isFixed) continue;
        m_points[i].position += step * midpoints[i].velocity;
        m_points[i].velocity += step * midpoints[i].acceleration;

        if (collision) {
            doCollision(m_points[i]);
        }
    }

    //clear forces for next step
    clearForces();

}

void MassSpringSystemSimulator::leapfrog(float step, bool gravity, bool collision) {
    //compute new forces
    for (const Spring& s : m_springs) {
        // compute internal force for p0
        Vec3 internal_force = computeInternalForce(m_points[s.point0], m_points[s.point1], s.initialLength);
        m_points[s.point0].force += internal_force;
        // for p1, force in opposite direction
        m_points[s.point1].force -= internal_force;
    }

    //compute new positions
    for (MassPoint& p : m_points) {
        if (p.isFixed)
            continue;

        // compute acceleration for p
        Vec3 acc = (p.force - m_fDamping * p.velocity) / m_fMass;

        if (gravity)
            acc += VEC3_GRAVITY;

        //compute new velocity
        p.velocity += step * acc;
        // compute new position
        p.position += step * p.velocity;

        if (collision) {
            doCollision(p);
        }
    }

    //clear forces for next step
    clearForces();

}






