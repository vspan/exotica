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

#include <exotica_core_task_maps/avoid_look_at.h>

REGISTER_TASKMAP_TYPE("AvoidLookAt", exotica::AvoidLookAt);

namespace exotica
{
AvoidLookAt::AvoidLookAt() = default;
AvoidLookAt::~AvoidLookAt() = default;

void AvoidLookAt::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
  
  // Get EffPoint
  Eigen::Vector3d c = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(0).p.data);
  double c_squared_norm = c.squaredNorm();
  
  for (int i = 0; i < n_objects_; ++i) {

    // Get object position
    Eigen::Vector3d o = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(i+1).p.data);

    // Compute orthogonal projection onto eff approach axis
    Eigen::Vector3d a = c * o.dot(c) / c_squared_norm;
    
    // Compute distance between object and orthogonal projection onto eff approach axis
    double delta = (a - o).norm();

    // Compute phi
    phi(i) = 0.5 * (radii2_(i) - delta*delta);
    
  }
  
}

void AvoidLookAt::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{

  // Get EffPoint
  Eigen::Vector3d c = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(0).p.data);
  double c_squared_norm = c.squaredNorm();
  
  for (int i = 0; i < n_objects_; ++i) {

    int object_id = i+1;

    // Get object position
    Eigen::Vector3d o = Eigen::Map<Eigen::Vector3d>(kinematics[0].Phi(object_id).p.data);

    // Compute orthogonal projection onto eff approach axis
    double alpha = o.dot(c) / c_squared_norm;
    Eigen::Vector3d a = alpha*c;
    
    // Compute distance between object and orthogonal projection onto eff approach axis
    double delta = (a - o).norm();

    // Compute phi
    phi(i) = 0.5 * (radii2_(i) - delta*delta);

    // Compute jacobian
    for (int j = 0; j < jacobian.cols(); ++j) {

      Eigen::Vector3d od = kinematics[0].jacobian[object_id].data.topRows<3>().col(j);
      double alphad = od.dot(c)/c_squared_norm;
      Eigen::Vector3d ad = alphad * c;
      jacobian(i, j) = (od - ad).dot(a - o);
      
    }
    
  }
  
}

void AvoidLookAt::Instantiate(AvoidLookAtInitializer& init)
{

  // Setup
  n_objects_ = init.Radii.size();
  radii2_.resize(n_objects_);
  for (int i = 0; i < n_objects_; ++i) {
    radii2_(i) = init.Radii(i)*init.Radii(i);
  }
  
}

int AvoidLookAt::TaskSpaceDim()
{
    return n_objects_;
}
}
