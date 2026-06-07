function g = gRvect(W,B,R,r_bg,r_bb)
% 计算恢复力向量g，相对于参考点CO
% g = gRvect(W,B,R,r_bg,r_bb) computes the 6x1 vector of restoring
% forces about an arbitrarily point CO for a submerged body using the
% rotation matrix R as input. Use g = gvect(W,B,theta,phi,r_bg,r_bb) for
% Euler angle inputs. For floating vessels, use Gmtrx.m. The following
% examples show how to use gvect.m and gRvect.m.
%
% Euler angles:
%  R = Rzyx(phi,theta,psi)
%  g = gRvect(W,B,R,r_bg,r_bb)            input: rotation matrix R
%  g = gvect(W,B,theta,phi,r_bg,r_bb)     input: Euler angles phi, theta
% Unit quaternions:
%  Rq = Rquat(q)
%  g = gRvect(W,B,Rq,r_bg,r_bb)           input: rotation matrix Rq
%
% Inputs:
%  W, B: weight and buoyancy (kg) 重量和浮力（千克）
%  R: rotation matrix Rzyx (Euler angles) or Rquat (unit quaternions) 旋转矩阵
%  r_bg = [x_g y_g z_g]: location of the CG with respect to the CO (m) 重心位置向量
%  r_bb = [x_b y_b z_b]: location of the CB with respect to th CO (m) 浮心位置向量
%
% Author:    Thor I. Fossen
% Date:      16 Dec 2021
% Revisions:

g = [...
    -(W-B) * R(3,1)
    -(W-B) * R(3,2)
    -(W-B) * R(3,3)
    -(r_bg(2)*W - r_bb(2)*B) * R(3,3) + (r_bg(3)*W - r_bb(3)*B) * R(3,2)
    -(r_bg(3)*W - r_bb(3)*B) * R(3,1) + (r_bg(1)*W - r_bb(1)*B) * R(3,3)
    -(r_bg(1)*W - r_bb(1)*B) * R(3,2) + (r_bg(2)*W - r_bb(2)*B) * R(3,1) ];

end

