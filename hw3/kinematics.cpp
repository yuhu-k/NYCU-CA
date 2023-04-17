#include "kinematics.h"

#include <algorithm>
#include <iostream>
#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // Same as HW2, but have some minor change
  // Hint:
  //   1. If you don't use `axis` in this function, you can copy-paste your code
  // Note:
  //   1. bone.axis becomes quaternion instead of vector3f

  if (bone->parent != nullptr) {
    bone->startPosition = bone->parent->endPosition;
    bone->rotation = bone->parent->rotation * bone->rotationParentCurrent * bone->axis * posture.rotations[bone->idx];
  } else {
    bone->startPosition = Eigen::Vector3f::Zero();
    bone->rotation = bone->axis * posture.rotations[bone->idx];
  }

  bone->endPosition = bone->startPosition + posture.translations[bone->idx] +
                      bone->rotation * (bone->direction).normalized() * bone->length;

  if (bone->child != nullptr) {
    bone->child->rotationParentCurrent = bone->axis.inverse();
    forwardKinematics(posture, bone->child);
  }

  if (bone->sibling != nullptr) {
    bone->sibling->rotationParentCurrent = bone->rotationParentCurrent;
    forwardKinematics(posture, bone->sibling);
  }
}

Eigen::VectorXf leastSquareSolver(const Eigen::Matrix3Xf& jacobian, const Eigen::Vector3f& target) {
  // TODO (find x which min(| jacobian * x - target |))
  // Hint:
  //   1. Linear algebra - least squares solution
  //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
  // Note:
  //   1. SVD or other pseudo-inverse method is useful
  //   2. Some of them have some limitation, if you use that method you should check it.
  Eigen::VectorXf solution(jacobian.cols());
  solution = jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(target);
  return solution;
}

void inverseKinematics(const Eigen::Vector3f& target, Bone* start, Bone* end, Posture& posture) {
  constexpr int maxIterations = 10000;
  constexpr float epsilon = 1E-3f;
  constexpr float step = 0.1f;
  // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is root.
  Bone* root = start - start->idx;
  std::vector<Bone*> boneList;
  // TODO
  // Hint:
  //   1. Traverse from end to start is easier than start to end (since there is only 1 parent)
  //   2. If start bone is not reachable from end. Go to root first.
  // Note:
  //   1. Both start and end should be in the list

  // Write your code here.
  for (Bone* tmp = end;1; tmp = tmp->parent) {
    boneList.push_back(tmp);
    if (tmp == root || tmp == start) break;
  }


  size_t boneNum = boneList.size();
  std::cout << boneNum << std::endl;
  Eigen::Matrix3Xf jacobian(3, 3 * boneNum);
  jacobian.setZero();

  for (int i = 0; i < maxIterations; ++i) {
    forwardKinematics(posture, root);
    // TODO (compute jacobian)
    //   1. Compute jacobian columns
    //   2. Compute dTheta
    // Hint:
    //   1. You should not put rotation in jacobian if it doesn't have that DoF.
    //   2. jacobian.col(/* some column index */) = /* jacobian column */
    //   3. Call leastSquareSolver to compute dTheta

    // Write your code here.
    Bone* tmp;
    for (size_t j = 0; j < boneNum; j++) {
      tmp = boneList[j];
      Eigen::Matrix3f rotation = tmp->rotation.matrix();
      Eigen::Vector3f r = tmp->startPosition;
      Eigen::Vector3f p = end->endPosition;
      for (int k = 0; k < 3; k += 1) {
        Eigen::Vector3f a;
        switch (k) {
          case 0: 
              a = (rotation * Eigen::Vector3f(1, 0, 0)).normalized(); break;
          case 1: 
              a = (rotation * Eigen::Vector3f(0, 1, 0)).normalized(); break;
          case 2: 
              a = (rotation * Eigen::Vector3f(0, 0, 1)).normalized(); break;
        }
        jacobian.col(j * 3 + k) = a.cross(p - r);
      }
    }
    Eigen::VectorXf dTheta = step * leastSquareSolver(jacobian, (target - end->endPosition));
    for (size_t j = 0; j < boneNum; j++) {
      const auto& bone = *boneList[j];
      // TODO (update rotation)
      //   1. Update posture's eulerAngle using deltaTheta
      // Hint:
      //   1. Use posture.eulerAngle to get posture's eulerAngle
      //   2. All angles are in radians.
      //   3. You can ignore rotation limit of the bone.
      // Bonus:
      //   1. You cannot ignore rotation limit of the bone.

      // Write your code here.
      posture.eulerAngle[bone.idx][0] += dTheta[j * 3];
      posture.eulerAngle[bone.idx][1] += dTheta[j * 3 + 1];
      posture.eulerAngle[bone.idx][2] += dTheta[j * 3 + 2];

      posture.rotations[bone.idx] = Eigen::AngleAxisf(posture.eulerAngle[bone.idx][2], Eigen::Vector3f::UnitZ()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][1], Eigen::Vector3f::UnitY()) *
                                    Eigen::AngleAxisf(posture.eulerAngle[bone.idx][0], Eigen::Vector3f::UnitX());
    }
  }
}
