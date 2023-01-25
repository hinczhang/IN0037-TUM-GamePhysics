#include "ThreebodySimulator.h"

#define PI 3.141592653589
#define G sqrt(2)/64

ThreebodySimulator::ThreebodySimulator() {

}

const char* ThreebodySimulator::getTestCasesStr() {
    return "Demo 1 (one-step), Demo 2 (single body), Demo 3 (two-body collision), Demo 4 (Three body: stable solution), Demo 5 (Three body: non-stable solution)";
}
void ThreebodySimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;

    switch (m_iTestCase)
    {
    case 1:
        TwAddVarRW(this->DUC->g_pTweakBar, "ExtForceLoc.x", TW_TYPE_FLOAT, &m_externalForcePointX, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "ExtForceLoc.y", TW_TYPE_FLOAT, &m_externalForcePointY, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "ExtForceLoc.z", TW_TYPE_FLOAT, &m_externalForcePointZ, "step=0.1");
        break;
    case 2:
        TwAddVarRW(this->DUC->g_pTweakBar, "Speed Obj 1", TW_TYPE_FLOAT, &m_collisionObjSpeed1, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Speed Obj 2", TW_TYPE_FLOAT, &m_collisionObjSpeed2, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Mass Obj 1", TW_TYPE_INT32, &m_collisionObjMass1, "step=1 min=1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Mass Obj 2", TW_TYPE_INT32, &m_collisionObjMass2, "step=1 min=1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Bounciness", TW_TYPE_FLOAT, &bounciness, "step=0.1 min = 0 max = 1");
        break;

    default: break;
    }
}

void ThreebodySimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

Mat4 ThreebodySimulator::obj2World(const RigidBody& body) {
    Mat4 scale_mat, rot_mat, translat_mat;

    scale_mat.initScaling(body.size.x, body.size.y, body.size.z);
    rot_mat = body.orientation.getRotMat();
    translat_mat.initTranslation(body.position.x, body.position.y, body.position.z);

    return scale_mat * rot_mat * translat_mat;
}

void ThreebodySimulator::drawRigidBodies() {


    for (int i = 0; i < m_bodies.size(); i++) {
        if (m_iTestCase == 3 && i == 0) {
            DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.2, 0.75, 0.2));
        }
        else {
            DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
        }
        //DUC->drawRigidBody(obj2World(m_bodies[i]));
		DUC->drawSphere(m_bodies[i].position, m_bodies[i].size);
    }
}

void ThreebodySimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
    drawRigidBodies();
    drawTracks();
}

void ThreebodySimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    m_bodies.clear();
    m_gravity = false;
    switch (m_iTestCase)
    {
    case 0: {
        std::cout << "Demo 1 (One time step simulation)" << std::endl;
        Vec3 x1(0.3, 0.5, 0.25);
        int b1 = addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        setOrientationOf(b1, Quat(0, 0, M_PI / 2));
        applyForceOnBody(b1, x1, Vec3(1, 1, 0));
        simulateTimestep(2);
        Vec3 v = getLinearVelocityOfRigidBody(b1);
        Vec3 w = getAngularVelocityOfRigidBody(b1);
        std::cout << "Position of body: " << getPositionOfRigidBody(b1) << endl;
        std::cout << "Linear velocity of body: " << v << endl;
        std::cout << "Angular velocity of body: " << w << endl;
        std::cout << "World velocity of point: " << v + cross(w, x1) << endl;
        std::cout << std::endl;
        m_bodies.clear();

    }break;
    case 1: {
        std::cout << "Demo 2 (Simple single body simulation)" << std::endl;
        std::cout << "INSTRUCTIONS:\n" <<
            "1. Use the slider in the UI to change the location where the external force will be applied.\n" <<
            "2. Click and drag with the left mouse button to apply the force.\n\n";
        int b1 = addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        setOrientationOf(b1, Quat(0, 0, M_PI / 2));
    }break;
    case 2: {
        std::cout << "Demo 3 (Two body collision)\n" << std::endl;
        int b1 = addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.5, 0.5), 2e9);
        setOrientationOf(b1, Quat(0, -M_PI / 4, M_PI / 4));
        setVelocityOf(b1, Vec3(m_collisionObjSpeed1, 0, 0));

        int b2 = addRigidBody(Vec3(2, 0, 0), Vec3(0.5, 1, 0.5), 2e9);
        setOrientationOf(b2, Quat(0, 0, M_PI / 2));
        setVelocityOf(b2, Vec3(-m_collisionObjSpeed2, 0, 0));
    }break;
    case 3: {
        std::cout << "Demo 4 (Three body: stable solution)\n";

        std::cout << "INSTRUCTIONS:\n" <<
            "1. Hold the left mouse button and drag to apply a force on the green object.\n\n";

        bounciness = 0.3;

        int b1 = addRigidBody(Vec3(0, 0, 1), Vec3(0.05, 0.05, 0.05), 4);
        setVelocityOf(b1, Vec3(-1, 1, 0) * sqrt(2));
        int b2 = addRigidBody(Vec3(1, 0, 0), Vec3(0.05, 0.05, 0.05), 4);
        setVelocityOf(b2, Vec3(0, -1, 1) * sqrt(2));
        int b3 = addRigidBody(Vec3(0, 1, 0), Vec3(0.05, 0.05, 0.05), 4);
        setVelocityOf(b3, Vec3(1, 0, -1) * sqrt(2));

    } break;
    case 4: {
        std::cout << "Demo 5 (Three body: non-stable solution)\n";
		int b1 = addRigidBody(Vec3(0, 0, 1), Vec3(0.05, 0.05, 0.05), 128);
        setVelocityOf(b1, Vec3(-1, 1, 0) * 1);
		int b2 = addRigidBody(Vec3(1, 0, 0), Vec3(0.05, 0.05, 0.05), 128);
        setVelocityOf(b2, Vec3(0, -1, 1) * 1);
        int b3 = addRigidBody(Vec3(0, 1, 0), Vec3(0.05, 0.05, 0.05), 128);
        setVelocityOf(b3, Vec3(1, 0, -1) * 1);
		int b4 = addRigidBody(Vec3(1, 1, 1), Vec3(0.01, 0.01, 0.01), 1);
		setVelocityOf(b4, Vec3(-0.003, -0.005, 0.01));
        // int b4 = addRigidBody(Vec3(0, 0, 0), Vec3(0.0001, 0.0001, 0.0001), 1);
        //setFixedOf(b4, true);
    }
    default:
        break;
    }
}

void ThreebodySimulator::externalForcesCalculations(float timeElapsed) {
    Point2D mouseDiff;
    mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
    mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (mouseDiff.x != 0 || mouseDiff.y != 0)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
        worldViewInv = worldViewInv.inverse();
        Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
        Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);

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

Vec3 ThreebodySimulator::computeGravitationalForce(const RigidBody& p0, const RigidBody& p1)
{
    float currLen = norm(p0.position - p1.position);
    Vec3 force = -G * p0.mass * p1.mass / (currLen * currLen * currLen) * (p0.position - p1.position);
	return force;
}

void ThreebodySimulator::addExternalForces() {
    switch (m_iTestCase)
    {
    case 1:
        applyForceOnBody(0,
            Vec3(m_externalForcePointX, m_externalForcePointY, m_externalForcePointZ),
            m_externalForce);
        break;
    case 3:
        applyForceOnBody(0, Vec3(0, 0, 0), m_externalForce);
        break;
    default:
        break;
    }
}

void ThreebodySimulator::angularCalculations(float timestep) {
    for (RigidBody& b : m_bodies) {
        // if (b.isFixed)
        //     continue;

         //compute torque
        Vec3 torque = Vec3(0, 0, 0);
        for (const ExternalForce& f : b.ext_forces) {
            torque += cross(f.m_contact_point, f.m_force);
        }

        //update orientation
        b.orientation += (timestep / 2) * Quat(b.angular_vel.x, b.angular_vel.y, b.angular_vel.z, 0) * b.orientation;
        //renormalize quaternions
        if (b.orientation.norm() != 0)
            b.orientation /= b.orientation.norm();

        //update momentum
        b.momentum += timestep * torque;

        //update angular velocity
        Mat4 rot = b.orientation.getRotMat();
        Mat4 rot_trans = b.orientation.getRotMat();
        rot_trans.transpose();
        Mat4 I_inv = rot * b.inverseI0 * rot_trans;
        // b.angular_vel = I_inv * b.momentum;
        b.angular_vel = I_inv.transformVector(b.momentum);
        b.current_inv_I = I_inv;


    }
}

void ThreebodySimulator::euler(float timestep) {
    int body_num = m_bodies.size();
    for (int i = 0; i < body_num - 1; ++i) {
        for (int j = i + 1; j < body_num; ++j) {
			RigidBody& p0 = m_bodies[i];
			RigidBody& p1 = m_bodies[j];

			Vec3 force = computeGravitationalForce(p0, p1);
			applyForceOnBody(i, p0.position, force);
			applyForceOnBody(j, p1.position, -force);
        }
    }
    
    for (RigidBody& b : m_bodies) {
        // if (b.isFixed)
        //     continue;

         // add external forces
        Vec3 force = Vec3(0, 0, 0);

        for (const ExternalForce& f : b.ext_forces)
            force += f.m_force;

        //update position
        if (!b.isFixed) {
            b.position += timestep * b.linear_vel;
            b.linear_vel += timestep * (force / b.mass);
        }
        if (m_gravity && !b.isFixed)
            b.linear_vel += timestep * VEC3_GRAVITY;
    }

}

void ThreebodySimulator::collide(RigidBody& b1, RigidBody& b2, const CollisionInfo& info) {

    // compute local collision points
    Vec3 x1 = info.collisionPointWorld - b1.position;
    Vec3 x2 = info.collisionPointWorld - b2.position;

    // compute velocities at collision point
    Vec3 v1 = b1.linear_vel + cross(b1.angular_vel, x1);
    Vec3 v2 = b2.linear_vel + cross(b2.angular_vel, x2);

    // compute relative velocity
    Vec3 v_rel = v1 - v2;

    Vec3 normal = info.normalWorld;

    auto separating = v_rel * normal;
    if (dot(v_rel, normal) > 0) {
        // bodies are separating
        return;
    }
    //compute impulse
    Vec3 J;
    Vec3 J_top = -dot((1 + bounciness) * v_rel, normal);

    Vec3 aux1 = cross(b1.current_inv_I * cross(x1, normal), x1);
    Vec3 aux2 = cross(b2.current_inv_I * cross(x2, normal), x2);
    Vec3 J_bottom = 1.0f / b1.mass + 1.0f / b2.mass + dot((aux1 + aux2), normal);
    J = J_top / J_bottom;

    //update values
    b1.linear_vel += J * normal / b1.mass;
    b2.linear_vel -= J * normal / b2.mass;

    b1.momentum += cross(x1, J * normal);
    b2.momentum -= cross(x2, J * normal);

}

void ThreebodySimulator::drawTracks()
{
    int index = 0;
	double deltaRadius = 0.01 / double(int(tracks.size()));
    for (auto& p: tracks) {
        DUC->setUpLighting(Vec3(0.2, 0.1, 0.5), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
        double radius = deltaRadius * double(index);
        Vec3 normDirection = p.linear_vel/norm(p.linear_vel);
		DUC->drawSphere(p.position - 0.05*normDirection, radius);
		index++;
    }
}

void ThreebodySimulator::do_collisions() {

    if (m_bodies.size() <= 1) {
        return;
    }

    for (int i = 0; i < m_bodies.size() - 1; ++i) {
        for (int j = i + 1; j < m_bodies.size(); ++j) {
            CollisionInfo info = checkCollisionSAT(obj2World(m_bodies[i]), obj2World(m_bodies[j]));
            if (info.isValid) {
                // found collision
                collide(m_bodies[i], m_bodies[j], info);
            }
        }
    }


}

void ThreebodySimulator::simulateTimestep(float timeStep) {

    // add any externalForces if needed
    // addExternalForces();

    // first update the linear stuff
    euler(timeStep);

    // do rotation stuff
    angularCalculations(timeStep);

        if (tracks.size() > 150 * m_bodies.size()) {
            tracks.erase(tracks.begin(), tracks.begin() +  m_bodies.size());
        }
        
        for (auto b : m_bodies) {
			tracks.push_back(b);
	    }
    
    

    // check collisions
    do_collisions();

    //clear forces for next step
    for (RigidBody& b : m_bodies) {
        b.ext_forces.clear();
    }
}

void ThreebodySimulator::onClick(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
}

void ThreebodySimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}


//---------------------- EXTRA FUNCTIONS -----------------------//


int ThreebodySimulator::getNumberOfRigidBodies()
{
    return m_bodies.size();
}

Vec3 ThreebodySimulator::getPositionOfRigidBody(int i)
{
    RigidBody body = m_bodies[i];

    return body.position;
}

Vec3 ThreebodySimulator::getLinearVelocityOfRigidBody(int i)
{
    return m_bodies[i].linear_vel;
}

Vec3 ThreebodySimulator::getAngularVelocityOfRigidBody(int i)
{
    return m_bodies[i].angular_vel;
}

void ThreebodySimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
    m_bodies[i].ext_forces.emplace_back(force, loc);
}

int ThreebodySimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    int index = m_bodies.size();
    m_bodies.emplace_back(position, size, mass);
    return index;
}

void ThreebodySimulator::setOrientationOf(int i, Quat orientation)
{
    m_bodies[i].orientation = orientation;
}

void ThreebodySimulator::setVelocityOf(int i, Vec3 velocity)
{
    m_bodies[i].linear_vel = velocity;
}

void ThreebodySimulator::setFixedOf(int i, bool fixed)
{
    m_bodies[i].isFixed = fixed;
}




