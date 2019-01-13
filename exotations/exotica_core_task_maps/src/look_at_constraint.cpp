//
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

#include <exotica_core_task_maps/look_at_constraint.h>

REGISTER_TASKMAP_TYPE("LookAtConstraint", exotica::LookAtConstraint);

namespace exotica
{
LookAtConstraint::LookAtConstraint() = default;
LookAtConstraint::~LookAtConstraint() = default;


void LookAtConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
  for (int i = 0; i<n_; ++i) {
    phi(i) = -kinematics[0].Phi(i).p.data[2];
  }
}

void LookAtConstraint::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
  for (int i = 0; i<n_; ++i) {
    phi(i) = -kinematics[0].Phi(i).p.data[2];
    jacobian.row(i) = -kinematics[0].jacobian[i].data.row(2);
  }  
}

void LookAtConstraint::Instantiate(LookAtConstraintInitializer& init)
{
  n_ = frames_.size();
}

int LookAtConstraint::TaskSpaceDim()
{
    return n_;
}
}
