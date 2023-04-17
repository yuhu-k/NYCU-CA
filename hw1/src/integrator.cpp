#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 2 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  for (const auto &p : particles) {
    // Write code here!
    p->position() += deltaTime * p->velocity();
    p->velocity() += deltaTime * p->acceleration();
  }
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.
  std::vector<Particles> backup;
  // Write code here!

  for (const auto &p : particles) {
    backup.push_back(*p);
  }
  simulateOneStep();
  int i = 0;
  for (const auto &p : particles) {
    p->position() = backup[i].position() + p->velocity() * deltaTime;
    p->velocity() = backup[i].velocity() + p->acceleration() * deltaTime;
    i++;
  }
}
void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!
  std::vector<Particles> backup;
  for (const auto &p : particles) {
    backup.push_back(*p);
  }
  simulateOneStep();
  int i = 0;
  for (const auto &p : particles) {
    p->position() = backup[i].position() + (p->velocity() + backup[i].velocity()) / 2 * deltaTime;
    p->velocity() = backup[i].velocity() + (p->acceleration() + backup[i].acceleration()) / 2 * deltaTime;
    i++;
  }
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!
  std::vector<Particles> backup;
  std::vector<Eigen::Matrix4Xf> k1,k2,k3,k4;
  int i = 0;
  for (const auto &p : particles) {
    backup.push_back(*p);
    k1.push_back(deltaTime * p->velocity());
    p->position() += k1[i]/2;
    i++;
  }
  simulateOneStep();
   i = 0;
  for (const auto &p : particles) {
    k2.push_back(deltaTime * (p->velocity() + backup[i].velocity()) / 2);
    p->position() = backup[i].position() + k2[i] / 2;
    p->velocity() = backup[i].velocity();
    i++;
  }
  
  simulateOneStep();
  i = 0;
  for (const auto &p : particles) {
    k3.push_back(deltaTime * (p->velocity() + backup[i].velocity()) / 2);
    p->position() = backup[i].position() + k3[i];
    p->velocity() = backup[i].velocity()+deltaTime*backup[i].acceleration();
    k4.push_back(deltaTime * p->velocity());
    i++;
  }
  simulateOneStep();
  i = 0;
  for (const auto &p : particles) {
    p->position() = backup[i].position() + k1[i];
    p->velocity() = backup[i].velocity() + backup[i].acceleration() * deltaTime;
    i++;
  }
}
