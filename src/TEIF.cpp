#include "TEIF.h"


target_EIF::target_EIF(int state_size)
{
	target_state_size = state_size;
	target_measurement_size = 3;
	filter_init = false;
	EIF_data_init(target_state_size, target_measurement_size, &T);
	Q.block(0, 0, 3, 3) = 1e-3*Eigen::MatrixXd::Identity(3, 3);
	Q.block(3, 3, 3, 3) = 7e-2*Eigen::MatrixXd::Identity(3, 3);
	R = 1e-5*Eigen::MatrixXd::Identity(3, 3);

	Mav_curr.v.setZero();
}
target_EIF::~target_EIF(){}

void target_EIF::setInitialState(Eigen::Vector3d Bbox)
{
	Eigen::Matrix3d K;
	Eigen::Matrix3d R_w2c = R_b2c*Mav_eigen_self.R_w2b;
	K << cam.fx(), 0, cam.cx(),
		0, cam.fy(), cam.cy(),
		0, 0, Bbox(2);
	
	//T.X.segment(0, 3) = R_w2c.inverse()*K.inverse()*Bbox;
	T.X.segment(0, 3) << 0, 0, 5;
	T.X.segment(3, 3) << 0, 0, 0;
	std::cout << "Init:\n" << T.X.segment(0, 3) << std::endl;
	T.P.setIdentity();
	T.P *= 1e-3;
	filter_init = true;
}

void target_EIF::setMeasurement(Eigen::Vector3d bBox){boundingBox = bBox;}

void target_EIF::setSEIFpredData(EIF_data self_data)
{
	self = self_data;
	self.X_hat.segment(0, 3) = self.X_hat.segment(0, 3) + Mav_eigen_self.R_w2b.inverse()*t_b2c;
}

void target_EIF::computePredPairs(double delta_t)
{
	double dt = static_cast<double>(delta_t);
	
	///////////////////////////// X, F ////////////////////////////////
	std::cout << "u:\n" << u << "\n";

	T.X_hat.segment(0, 3) = T.X.segment(0, 3) + T.X.segment(3, 3)*dt + 1/2*u*dt*dt;
	T.X_hat.segment(3, 3) = T.X.segment(3, 3) + u*dt;

	T.F.setIdentity();
	T.F.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity(3, 3)*dt;

	T.P_hat = T.F*T.P*T.F.transpose() + Q;
}
void target_EIF::computeCorrPairs()
{
	T.z = boundingBox;

	T.s.setZero();
	T.y.setZero();
	self.s.setZero();
	self.y.setZero();
	if(T.z != T.pre_z && T.z(2) >= 2.0 && T.z(2) <= 12.0)
	{
		Eigen::MatrixXd R_hat, R_bar;
		Eigen::Matrix3d R_w2c = R_b2c*Mav_eigen_self.R_w2b;
		Eigen::Vector3d r_qc_c = R_w2c*(T.X_hat.segment(0, 3) - self.X_hat.segment(0, 3));

		X = r_qc_c(0)/r_qc_c(2);
		Y = r_qc_c(1)/r_qc_c(2);
		Z = r_qc_c(2);

		T.h(0) = cam.fx()*X + cam.cx();
		T.h(1) = cam.fy()*Y + cam.cy();
		T.h(2) = Z;
		self.z = T.z;
		self.h = T.h;

		T.H(0, 0) = (cam.fx()/Z)*(R_w2c(0, 0) - R_w2c(2, 0)*X);
		T.H(0, 1) = (cam.fx()/Z)*(R_w2c(0, 1) - R_w2c(2, 1)*X);
		T.H(0, 2) = (cam.fx()/Z)*(R_w2c(0, 2) - R_w2c(2, 2)*X);
		T.H(1, 0) = (cam.fy()/Z)*(R_w2c(1, 0) - R_w2c(2, 0)*Y);
		T.H(1, 1) = (cam.fy()/Z)*(R_w2c(1, 1) - R_w2c(2, 1)*Y);
		T.H(1, 2) = (cam.fy()/Z)*(R_w2c(1, 2) - R_w2c(2, 2)*Y);
		T.H(2, 0) = R_w2c(2, 0);
		T.H(2, 1) = R_w2c(2, 1);
		T.H(2, 2) = R_w2c(2, 2);

		self.H = -T.H;

		R_hat = R + self.H*self.P_hat*self.H.transpose();
		R_bar = R + T.H*T.P_hat*T.H.transpose();

		T.s = T.H.transpose()*R_hat.inverse()*T.H;
		T.y = T.H.transpose()*R_hat.inverse()*(T.z - T.h + T.H*T.X_hat);

		self.s = self.H.transpose()*R_bar.inverse()*self.H;
		self.y = self.H.transpose()*R_bar.inverse()*(self.z - self.h + self.H*self.X_hat);
	}
	T.P = (T.P_hat.inverse() + T.s).inverse();
	T.X = T.P*(T.P_hat.inverse()*T.X_hat + T.y);
	T.pre_z = T.z;
}

void target_EIF::setFusionPairs(Eigen::MatrixXd fusedP, Eigen::VectorXd fusedX, double time)
{
    T.P = fusedP;
    T.X = fusedX;
}

EIF_data target_EIF::getTgtData(){return T;}
EIF_data target_EIF::getSelfData(){return self;}
void target_EIF::setEstAcc(Eigen::Vector3d acc)
{
	u = acc;
}