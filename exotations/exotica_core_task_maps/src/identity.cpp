// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <exotica_core_task_maps/identity.h>

REGISTER_TASKMAP_TYPE("Identity", exotica::Identity);

namespace exotica
{
Identity::Identity() = default;
Identity::~Identity() = default;

void Identity::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != joint_map_.size()) ThrowNamed("Wrong size of phi!");
    for (int i = 0; i < joint_map_.size(); i++)
    {
        phi(i) = x(joint_map_[i]) - joint_ref_(i);
    }
}

void Identity::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != joint_map_.size()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != joint_map_.size() || jacobian.cols() != N_) ThrowNamed("Wrong size of jacobian! " << N_);
    jacobian.setZero();
    for (int i = 0; i < joint_map_.size(); i++)
    {
        phi(i) = x(joint_map_[i]) - joint_ref_(i);
        jacobian(i, joint_map_[i]) = 1.0;
    }
}

// void Identity::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::VectorXdRef phidot, Eigen::MatrixXdRef jacobian, Eigen::MatrixXdRef Jdot)
// {
//     if (phi.rows() != joint_map_.size()) ThrowNamed("Wrong size of phi!");
//     if (jacobian.rows() != joint_map_.size() || jacobian.cols() != N_) ThrowNamed("Wrong size of jacobian! " << N_);
//     if (Jdot.rows() != joint_map_.size() || Jdot.cols() != N_) ThrowNamed("Wrong size of jacobian! " << N_);
//     jacobian.setZero();
//     Jdot.setZero();
//     for (int i = 0; i < joint_map_.size(); i++)
//     {
//         phi(i) = x(joint_map_[i]) - joint_ref_(i);
//         phidot(i) = x(joint_map_[i] + N_) - joint_ref_(i + joint_map_.size());
//         jacobian(i, joint_map_[i]) = 1.0;
//         Jdot(i, joint_map_[i]) = 1.0;
//     }
// }

void Identity::Instantiate(IdentityInitializer& init)
{
    init_ = init;
}

void Identity::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void Identity::Initialize()
{
    N_ = scene_->GetKinematicTree().GetNumControlledJoints();
    if (init_.joint_map.rows() > 0)
    {
        joint_map_.resize(init_.joint_map.rows());
        for (int i = 0; i < init_.joint_map.rows(); i++)
        {
            joint_map_[i] = init_.joint_map(i);
        }
    }
    else
    {
        joint_map_.resize(N_);
        for (int i = 0; i < N_; i++)
        {
            joint_map_[i] = i;
        }
    }

    if (init_.joint_ref.rows() > 0)
    {
        joint_ref_ = init_.joint_ref;
        if (joint_ref_.rows() != joint_map_.size()) ThrowNamed("Invalid joint reference size! Expecting " << joint_map_.size());
    }
    else
    {
        joint_ref_ = Eigen::VectorXd::Zero(joint_map_.size());
    }
}

int Identity::TaskSpaceDim()
{
    return joint_map_.size();
}
}
