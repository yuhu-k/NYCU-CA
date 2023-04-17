#include "kinematics.h"

#include <algorithm>

#include "utils.h"
void forwardKinematics(const Posture& posture, Bone* bone) {
  // TODO (FK)
  // You should set these variables:
  //     bone->startPosition = Eigen::Vector3f::Zero();
  //     bone->endPosition = Eigen::Vector3f::Zero();
  //     bone->rotation = Eigen::Quaternionf::Identity();
  // The sample above just set everything to initial state
  // Hint:
  //   1. posture.translations, posture.rotations
  // Note:
  //   1. This function will be called with bone == root bone of the skeleton

  // Write your code here
  Eigen::Quaternionf Rasf = Eigen::AngleAxisf(bone->axis[2], Eigen::Vector3f::UnitZ()) *
                           Eigen::AngleAxisf(bone->axis[1], Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(bone->axis[0], Eigen::Vector3f::UnitX());
  if (bone->parent != nullptr) {
    bone->startPosition = bone->parent->endPosition;
    bone->rotation = bone->parent->rotation * bone->rotationParentCurrent * Rasf * posture.rotations[bone->idx];
  } else {
    bone->startPosition = Eigen::Vector3f::Zero();
    bone->rotation = Rasf * posture.rotations[bone->idx];
  }
  
  bone->endPosition = bone->startPosition + posture.translations[bone->idx] +
                      bone->rotation * (bone->direction).normalized() * bone->length;
  
  
  if (bone->child != nullptr) {
    bone->child->rotationParentCurrent = Rasf.inverse();
    forwardKinematics(posture, bone->child);
  }
  
  if (bone->sibling != nullptr) {
    bone->sibling->rotationParentCurrent = bone->rotationParentCurrent;
    forwardKinematics(posture, bone->sibling);
  }
  
}

Motion motionWarp(const Motion& motion, int oldKeyframe, int newKeyframe) {
  Motion newMotion = motion;
  int totalFrames = static_cast<int>(motion.size());
  int totalBones = static_cast<int>(motion.posture(0).rotations.size());
  std::vector<Posture> postures(newKeyframe * totalFrames / oldKeyframe);
  for (int i = 0; i < newKeyframe * totalFrames / oldKeyframe; ++i) {
    // Maybe set some per=Frame variables here
    float ori = (float)oldKeyframe / newKeyframe * i;
    int frame = ori;
    float ratio = ori - frame;
    Posture p;
    for (int j = 0; j < totalBones; ++j) {
      // TODO (Time warping)
      // original: |--------------|---------------|
      // new     : |------------------|-----------|
      // OR
      // original: |--------------|---------------|
      // new     : |----------|-------------------|
      // You should set these variables:
      //     newMotion.posture(i).translations[j] = Eigen::Vector3f::Zero();
      //     newMotion.posture(i).rotations[j] = Eigen::Quaternionf::Identity();
      // The sample above just set to initial state
      // Hint:
      //   1. Your should scale the frames before and after key frames.
      //   2. You can use linear interpolation with translations.
      //   3. You should use spherical linear interpolation for rotations.

      // Write your code here
      Eigen::Quaternionf q;
      Eigen::Vector3f v3f;

      v3f = motion.posture(frame).translations[j] * (1 - ratio) + motion.posture(frame + 1).translations[j] * ratio;
      q = motion.posture(frame).rotations[j];
      q = q.slerp(ratio, motion.posture(frame + 1).rotations[j]);
      p.rotations.push_back(q);
      p.translations.push_back(v3f);
    }
    postures[i] = p;
  }
  newMotion.posture() = postures;
  return newMotion;
}

Motion motionBlend(const Motion& motionA, const Motion& motionB) {
  Motion newMotion;
  constexpr int blendFrameCount = 20;
  constexpr float blendFactor = 1.0f / blendFrameCount;
  constexpr int matchRange = 10;
  float difference[matchRange] = {};
  // TODO (Bonus)
  // motionA: |--------------|--matchRange--|--blendFrameCount--|
  // motionB:                               |--blendFrameCount--|--------------|
  // The starting frame of `blendFrameCount` can be in `matchRange`
  // Hint:
  //   1. Find motionB's starting posture
  //   2. Match it with the minimum cost posture in `matchRange`
  //   3. Find to translation and rotation offset between matched motionA and motionB's start
  //   4. Begin from the matched frame, blend `blendFrameCount` of frames,
  //      with a blendFactor from 1 / `blendFrameCount` to 1
  //   5. Add remaining motionB to newMotion
  // Note:
  //   1. The offset found in 3 should apply to 4 and 5
  //   2. A simple cost function is translation offsets between two posture.
  //   3. A better one considered both translations and rotations.
  //   4. Your animation should smoothly change from motionA to motionB.
  //   5. You can adjust those `constexpr`s above by yourself if you need.

  // Write your code here
  int totalFrames = static_cast<int>(motionA.size());
  int totalBones = static_cast<int>(motionA.posture(0).rotations.size());
  int totalFramesB = static_cast<int>(motionB.size());
  std::vector<Posture> postures;

  for (int i = 0; i < totalFrames - blendFrameCount; i++) {
    postures.push_back(motionA.posture(i));
  }

  for (int i = 0; i < blendFrameCount; i++) {
    float rate = i * blendFactor;
    Posture p;
    for (int j = 0; j < totalBones; j++) {
      Eigen::Quaternionf q;
      Eigen::Vector3f v3f;
      v3f = motionA.posture(totalFrames - blendFrameCount).translations[j];
      v3f += (motionA.posture(i + totalFrames - blendFrameCount).translations[j] -
              motionA.posture(totalFrames - blendFrameCount).translations[j]) *
                 (1 - rate) +
             (motionB.posture(i).translations[j] - motionB.posture(0).translations[j]) * rate;
      q = motionA.posture(i + totalFrames - blendFrameCount).rotations[j];
      q = q.slerp(rate, motionB.posture(blendFrameCount).rotations[j]);
      p.rotations.push_back(q);
      p.translations.push_back(v3f);
    }
    postures.push_back(p);
  }

  for (int i = blendFrameCount; i < totalFramesB; i++) {
    Posture p;
    for (int j = 0; j < totalBones; j++) {
      Eigen::Quaternionf q;
      Eigen::Vector3f v3f;
      v3f = motionA.posture(totalFrames - blendFrameCount).translations[j] +
            (motionB.posture(i).translations[j] - motionB.posture(0).translations[j]);
      q = motionB.posture(i).rotations[j];
      p.translations.push_back(v3f);
      p.rotations.push_back(q);
    }
    postures.push_back(p);
  }
  newMotion.posture() = postures;
  
  return newMotion;
}
