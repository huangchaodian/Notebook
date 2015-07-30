///////////////////////////////////////////////////////////////////////////////
///   "Sparse Iterative Closest Point"
///   by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///   Copyright (C) 2013  LGG, EPFL
///////////////////////////////////////////////////////////////////////////////
///   1) This file contains different implementations of the ICP algorithm.
///   2) This code requires EIGEN and NANOFLANN.
///   3) If OPENMP is activated some part of the code will be parallelized.
///   4) This code is for now designed for 3D registration
///   5) Two main input types are Eigen::Matrix3Xd or Eigen::Map<Eigen::Matrix3Xd>
///////////////////////////////////////////////////////////////////////////////
///   namespace nanoflann: NANOFLANN KD-tree adaptor for EIGEN
///   namespace RigidMotionEstimator: functions to compute the rigid motion
///   namespace SICP: sparse ICP implementation
///   namespace ICP: reweighted ICP implementation
///////////////////////////////////////////////////////////////////////////////
#ifndef ICP_H
#define ICP_H
#include "nanoflann.hpp"
#include <pcl/console/time.h>   // TicToc
#include <Eigen/Dense>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//pcl::visualization::CloudViewer cloudviewer("show cloud");
pcl::PointCloud<pcl::PointXYZ>cloudForShow;
void showmatrix3xd(Eigen::Matrix3Xd ma){
	pcl::PointCloud<pcl::PointXYZ> cloud;
	for (int i=0;i<ma.cols();i++)
	{
		pcl::PointXYZ p(ma(0,i),ma(1,i),ma(2,i));
		cloud.push_back(p);
	}
	//cloudviewer.showCloud(cloud.makeShared());
}
void AddMatrix3xdToCloud(Eigen::Matrix3Xd ma){
	for (int i=0;i<ma.cols();i++)
	{
		pcl::PointXYZ p(ma(0,i),ma(1,i),ma(2,i));
		cloudForShow.push_back(p);
	}
}
///////////////////////////////////////////////////////////////////////////////
namespace nanoflann {
	/// KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
	/// This code is adapted from the KDTreeEigenMatrixAdaptor class of nanoflann.hpp
	template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
	struct KDTreeAdaptor {
		typedef KDTreeAdaptor<MatrixType,DIM,Distance> self_t;
		typedef typename MatrixType::Scalar              num_t;
		typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
		typedef KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;
		index_t* index;
		KDTreeAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat) {
			const size_t dims = mat.rows();
			index = new index_t( dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims ) );
			index->buildIndex();
		}
		~KDTreeAdaptor() {delete index;}
		const MatrixType &m_data_matrix;
		/// Query for the num_closest closest points to a given point (entered as query_point[0:dim-1]).
		inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq) const {
			nanoflann::KNNResultSet<typename MatrixType::Scalar,IndexType> resultSet(num_closest);
			resultSet.init(out_indices, out_distances_sq);
			index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
		}
		/// Query for the closest points to a given point (entered as query_point[0:dim-1]).
		inline IndexType closest(const num_t *query_point) const {
			IndexType out_indices;
			num_t out_distances_sq;
			query(query_point, 1, &out_indices, &out_distances_sq);
			return out_indices;
		}
		const self_t & derived() const {return *this;}
		self_t & derived() {return *this;}
		inline size_t kdtree_get_point_count() const {return m_data_matrix.cols();}
		/// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
		inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const {
			num_t s=0;
			for (size_t i=0; i<size; i++) {
				const num_t d= p1[i]-m_data_matrix.coeff(i,idx_p2);
				s+=d*d;
			}
			return s;
		}
		/// Returns the dim'th component of the idx'th point in the class:
		inline num_t kdtree_get_pt(const size_t idx, int dim) const {
			return m_data_matrix.coeff(dim,idx);
		}
		/// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		template <class BBOX> bool kdtree_get_bbox(BBOX&) const {return false;}
	};
}
///////////////////////////////////////////////////////////////////////////////
/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace RigidMotionEstimator {
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Confidence weights
	template <typename Derived1, typename Derived2, typename Derived3>
	Eigen::Matrix4d point_to_point(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y,
		const Eigen::MatrixBase<Derived3>& w) {
			/// Normalize weight vector
			Eigen::VectorXd w_normalized = w/w.sum();
			/// De-mean
			Eigen::Vector3d X_mean, Y_mean;
			for(int i=0; i<3; ++i) {
				X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
				Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
			}
			X.colwise() -= X_mean;
			Y.colwise() -= Y_mean;
			/// Compute transformation
			Eigen::Affine3d transformation;
			Eigen::Matrix3d sigma = X * w_normalized.asDiagonal() * Y.transpose();
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
			if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
				Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
				transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
			} else {
				transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
			}
			transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
			/// Apply transformation
			X = transformation*X;
			/// Re-apply mean
			X.colwise() += X_mean;
			Y.colwise() += Y_mean;

			Eigen::Matrix4d rt=Eigen::Matrix4d::Identity();
			rt(0,3)=X_mean(0);rt(1,3)=X_mean(1);rt(2,3)=X_mean(2);
			Eigen::Matrix4d result=Eigen::Matrix4d::Identity();
			result=rt*transformation.matrix()*rt.inverse();
			/// Return transformation
			return result;
	}
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	template <typename Derived1, typename Derived2>
	inline Eigen::Matrix4d point_to_point(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y) {
			return point_to_point(X, Y, Eigen::VectorXd::Ones(X.cols()));
	}
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Confidence weights
	/// @param Right hand side
	template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
	Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y,
		Eigen::MatrixBase<Derived3>& N,
		const Eigen::MatrixBase<Derived4>& w,
		const Eigen::MatrixBase<Derived5>& u) {
			typedef Eigen::Matrix<double, 6, 6> Matrix66;
			typedef Eigen::Matrix<double, 6, 1> Vector6;
			typedef Eigen::Block<Matrix66, 3, 3> Block33;
			/// Normalize weight vector
			Eigen::VectorXd w_normalized = w/w.sum();
			/// De-mean
			Eigen::Vector3d X_mean;
			for(int i=0; i<3; ++i)
				X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
			X.colwise() -= X_mean;
			Y.colwise() -= X_mean;
			/// Prepare LHS and RHS
			Matrix66 LHS = Matrix66::Zero();
			Vector6 RHS = Vector6::Zero();
			Block33 TL = LHS.topLeftCorner<3,3>();
			Block33 TR = LHS.topRightCorner<3,3>();
			Block33 BR = LHS.bottomRightCorner<3,3>();
			Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
#pragma omp parallel
			{
#pragma omp for
				for(int i=0; i<X.cols(); i++) {
					C.col(i) = X.col(i).cross(N.col(i));
				}
#pragma omp sections nowait
				{
#pragma omp section
					for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
#pragma omp section
					for(int i=0; i<X.cols(); i++) TR += (C.col(i)*N.col(i).transpose())*w(i);
#pragma omp section
					for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i), w(i));
#pragma omp section
					for(int i=0; i<C.cols(); i++) {
						double dist_to_plane = -((X.col(i) - Y.col(i)).dot(N.col(i)) - u(i))*w(i);
						RHS.head<3>() += C.col(i)*dist_to_plane;
						RHS.tail<3>() += N.col(i)*dist_to_plane;
					}
				}
			}
			LHS = LHS.selfadjointView<Eigen::Upper>();
			/// Compute transformation
			Eigen::Affine3d transformation;
			Eigen::LDLT<Matrix66> ldlt(LHS);
			RHS = ldlt.solve(RHS);
			transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
			transformation.translation() = RHS.tail<3>();
			/// Apply transformation
			X = transformation*X;
			/// Re-apply mean
			X.colwise() += X_mean;
			Y.colwise() += X_mean;
			/// Return transformation
			return transformation;
	}
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Confidence weights
	template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
	inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Yp,
		Eigen::MatrixBase<Derived3>& Yn,
		const Eigen::MatrixBase<Derived4>& w) {
			return point_to_plane(X, Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
	}
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP {
	class Parameters {
	public:
		Parameters() : use_penalty(false),
			p(1.0),
			mu(10.0),
			alpha(1.2),
			max_mu(1e5),
			max_icp(10),
			max_outer(100),
			max_inner(1),
			stop(1e-5) {}
		/// Parameters
		bool use_penalty; /// if use_penalty then penalty method else ADMM or ALM (see max_inner)
		double p;         /// p norm
		double mu;        /// penalty weight
		double alpha;     /// penalty increase factor
		double max_mu;    /// max penalty
		int max_icp;      /// max ICP iteration
		int max_outer;    /// max outer iteration
		int max_inner;    /// max inner iteration. If max_inner=1 then ADMM else ALM
		double stop;      /// stopping criteria
	};
	/// Shrinkage operator (Automatic loop unrolling using template)
	template<unsigned int I>
	inline double shrinkage(double mu, double n, double p, double s) {
		return shrinkage<I-1>(mu, n, p, 1.0 - (p/mu)*std::pow(n, p-2.0)*std::pow(s, p-1.0));
	}
	template<>
	inline double shrinkage<0>(double, double, double, double s) {return s;}
	/// 3D Shrinkage for point-to-point
	template<unsigned int I>
	inline void shrink(Eigen::Matrix3Xd& Q, double mu, double p) {
		double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
		double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
#pragma omp parallel for
		for(int i=0; i<Q.cols(); ++i) {
			double n = Q.col(i).norm();
			double w = 0.0;
			if(n > ha) w = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
			Q.col(i) *= w;
		}
	}
	/// 1D Shrinkage for point-to-plane
	template<unsigned int I>
	inline void shrink(Eigen::VectorXd& y, double mu, double p) {
		double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
		double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
#pragma omp parallel for
		for(int i=0; i<y.rows(); ++i) {
			double n = std::abs(y(i));
			double s = 0.0;
			if(n > ha) s = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
			y(i) *= s;
		}
	}
	/// Sparse ICP with point to point
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Parameters
	template <typename Derived1, typename Derived2>
	Eigen::Matrix4d point_to_point(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y,
		Parameters par = Parameters()) {
			/// Build kd-tree
			nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);

			Eigen::Matrix4d finaltrans=Eigen::Matrix4d::Identity();

			/// Buffers
			Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::Matrix3Xd Xo1 = X;
			Eigen::Matrix3Xd Xo2 = X;
			/// ICP
			for(int icp=0; icp<par.max_icp; ++icp) {
				/// Find closest point
#pragma omp parallel for
				for(int i=0; i<X.cols(); ++i) {
					Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
				}
				pcl::console::TicToc time;time.tic();
				/// Computer rotation and translation
				double mu = par.mu;
				for(int outer=0; outer<par.max_outer; ++outer) {
					double dual = 0.0;
					for(int inner=0; inner<par.max_inner; ++inner) {
						/// Z update (shrinkage)
						Z = X-Q+C/mu;
						shrink<3>(Z, mu, par.p);
						/// Rotation and translation update
						Eigen::Matrix3Xd U = Q+Z-C/mu;
						Eigen::Matrix4d tran=RigidMotionEstimator::point_to_point(X, U);
						finaltrans=tran*finaltrans;
						/// Stopping criteria
						dual = (X-Xo1).colwise().norm().maxCoeff();
						Xo1 = X;
						if(dual < par.stop) break;
					}
					/// C update (lagrange multipliers)
					Eigen::Matrix3Xd P = X-Q-Z;
					if(!par.use_penalty) C.noalias() += mu*P;
					/// mu update (penalty)
					if(mu < par.max_mu) mu *= par.alpha;
					/// Stopping criteria
					double primal = P.colwise().norm().maxCoeff();
					if(primal < par.stop && dual < par.stop) break;
				}
				std::cout<<time.toc()<<"ms"<<std::endl;
				//showmatrix3xd(X);
				//AddMatrix3xdToCloud(Q);
				//AddMatrix3xdToCloud(Z);
				//AddMatrix3xdToCloud(C);
//				AddMatrix3xdToCloud(X);
				//AddMatrix3xdToCloud(Y);
				/// Stopping criteria
				double stop = (X-Xo2).colwise().norm().maxCoeff();
				Xo2 = X;
				if(stop < par.stop) break;
			}
			return finaltrans;
	}
	/// Sparse ICP with point to plane
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Parameters
	template <typename Derived1, typename Derived2, typename Derived3>
	void point_to_plane(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y,
		Eigen::MatrixBase<Derived3>& N,
		Parameters par = Parameters()) {
			/// Build kd-tree
			nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
			/// Buffers
			Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::VectorXd Z = Eigen::VectorXd::Zero(X.cols());
			Eigen::VectorXd C = Eigen::VectorXd::Zero(X.cols());
			Eigen::Matrix3Xd Xo1 = X;
			Eigen::Matrix3Xd Xo2 = X;
			/// ICP
			for(int icp=0; icp<par.max_icp; ++icp) {
				/// Find closest point
#pragma omp parallel for
				for(int i=0; i<X.cols(); ++i) {
					int id = kdtree.closest(X.col(i).data());
					Qp.col(i) = Y.col(id);
					Qn.col(i) = N.col(id);
				}
				/// Computer rotation and translation
				double mu = par.mu;
				for(int outer=0; outer<par.max_outer; ++outer) {
					double dual = 0.0;
					for(int inner=0; inner<par.max_inner; ++inner) {
						/// Z update (shrinkage)
						Z = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()+C.array()/mu;
						shrink<3>(Z, mu, par.p);
						/// Rotation and translation update
						Eigen::VectorXd U = Z-C/mu;
						RigidMotionEstimator::point_to_plane(X, Qp, Qn, Eigen::VectorXd::Ones(X.cols()), U);
						/// Stopping criteria
						dual = (X-Xo1).colwise().norm().maxCoeff();
						Xo1 = X;
						if(dual < par.stop) break;
					}
					/// C update (lagrange multipliers)
					Eigen::VectorXf P = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()-Z.array();
					if(!par.use_penalty) C.noalias() += mu*P;
					/// mu update (penalty)
					if(mu < par.max_mu) mu *= par.alpha;
					/// Stopping criteria
					double primal = P.array().abs().maxCoeff();
					if(primal < par.stop && dual < par.stop) break;
				}
				/// Stopping criteria
				double stop = (X-Xo2).colwise().norm().maxCoeff();
				Xo2 = X;
				if(stop < par.stop) break;
			}
	}
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using iterative reweighting
namespace ICP {
	enum Function {
		PNORM,
		TUKEY,
		FAIR,
		LOGISTIC,
		TRIMMED,
		NONE
	};
	class Parameters {
	public:
		Parameters() : f(NONE),
			p(0.1),
			max_icp(100),
			max_outer(100),
			stop(1e-5) {}
		/// Parameters
		Function f;     /// robust function type
		double p;       /// paramter of the robust function
		int max_icp;    /// max ICP iteration
		int max_outer;  /// max outer iteration
		double stop;    /// stopping criteria
	};
	/// Weight functions
	/// @param Residuals
	/// @param Parameter
	void uniform_weight(Eigen::VectorXd& r) {
		r = Eigen::VectorXd::Ones(r.rows());
	}
	/// @param Residuals
	/// @param Parameter
	void pnorm_weight(Eigen::VectorXd& r, double p, double reg=1e-8) {
		for(int i=0; i<r.rows(); ++i) {
			r(i) = p/(std::pow(r(i),2-p) + reg);
		}
	}
	/// @param Residuals
	/// @param Parameter
	void tukey_weight(Eigen::VectorXd& r, double p) {
		for(int i=0; i<r.rows(); ++i) {
			if(r(i) > p) r(i) = 0.0;
			else r(i) = std::pow((1.0 - std::pow(r(i)/p,2.0)), 2.0);
		}
	}
	/// @param Residuals
	/// @param Parameter
	void fair_weight(Eigen::VectorXd& r, double p) {
		for(int i=0; i<r.rows(); ++i) {
			r(i) = 1.0/(1.0 + r(i)/p);
		}
	}
	/// @param Residuals
	/// @param Parameter
	void logistic_weight(Eigen::VectorXd& r, double p) {
		for(int i=0; i<r.rows(); ++i) {
			r(i) = (p/r(i))*std::tanh(r(i)/p);
		}
	}
	struct sort_pred {
		bool operator()(const std::pair<int,double> &left,
			const std::pair<int,double> &right) {
				return left.second < right.second;
		}
	};
	/// @param Residuals
	/// @param Parameter
	void trimmed_weight(Eigen::VectorXd& r, double p) {
		std::vector<std::pair<int, double> > sortedDist(r.rows());
		for(int i=0; i<r.rows(); ++i) {
			sortedDist[i] = std::pair<int, double>(i,r(i));
		}
		std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
		r.setZero();
		int nbV = r.rows()*p;
		for(int i=0; i<nbV; ++i) {
			r(sortedDist[i].first) = 1.0;
		}
	}
	/// @param Function type
	/// @param Residuals
	/// @param Parameter
	void robust_weight(Function f, Eigen::VectorXd& r, double p) {
		switch(f) {
		case PNORM: pnorm_weight(r,p); break;
		case TUKEY: tukey_weight(r,p); break;
		case FAIR: fair_weight(r,p); break;
		case LOGISTIC: logistic_weight(r,p); break;
		case TRIMMED: trimmed_weight(r,p); break;
		case NONE: uniform_weight(r); break;
		default: uniform_weight(r); break;
		}
	}
	/// Reweighted ICP with point to point
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Parameters
	template <typename Derived1, typename Derived2>
	void point_to_point(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y,
		Parameters par = Parameters()) {
			/// Build kd-tree
			nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
			/// Buffers
			Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
			Eigen::Matrix3Xd Xo1 = X;
			Eigen::Matrix3Xd Xo2 = X;
			/// ICP
			for(int icp=0; icp<par.max_icp; ++icp) {
				/// Find closest point
#pragma omp parallel for
				for(int i=0; i<X.cols(); ++i) {
					Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
				}
				/// Computer rotation and translation
				for(int outer=0; outer<par.max_outer; ++outer) {
					/// Compute weights
					W = (X-Q).colwise().norm();
					robust_weight(par.f, W, par.p);
					/// Rotation and translation update
					RigidMotionEstimator::point_to_point(X, Q, W);
					/// Stopping criteria
					double stop1 = (X-Xo1).colwise().norm().maxCoeff();
					Xo1 = X;
					if(stop1 < par.stop) break;
				}
				/// Stopping criteria
				double stop2 = (X-Xo2).colwise().norm().maxCoeff();
				Xo2 = X;
				if(stop2 < par.stop) break;
			}
	}
	/// Reweighted ICP with point to plane
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Target normals (one 3D normal per column)
	/// @param Parameters
	template <typename Derived1, typename Derived2, typename Derived3>
	void point_to_plane(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y,
		Eigen::MatrixBase<Derived3>& N,
		Parameters par = Parameters()) {
			/// Build kd-tree
			nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
			/// Buffers
			Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
			Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
			Eigen::Matrix3Xd Xo1 = X;
			Eigen::Matrix3Xd Xo2 = X;
			/// ICP
			for(int icp=0; icp<par.max_icp; ++icp) {
				/// Find closest point
#pragma omp parallel for
				for(int i=0; i<X.cols(); ++i) {
					int id = kdtree.closest(X.col(i).data());
					Qp.col(i) = Y.col(id);
					Qn.col(i) = N.col(id);
				}
				/// Computer rotation and translation
				for(int outer=0; outer<par.max_outer; ++outer) {
					/// Compute weights
					W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
					robust_weight(par.f, W, par.p);
					/// Rotation and translation update
					RigidMotionEstimator::point_to_plane(X, Qp, Qn, W);
					/// Stopping criteria
					double stop1 = (X-Xo1).colwise().norm().maxCoeff();
					Xo1 = X;
					if(stop1 < par.stop) break;
				}
				/// Stopping criteria
				double stop2 = (X-Xo2).colwise().norm().maxCoeff() ;
				Xo2 = X;
				if(stop2 < par.stop) break;
			}
	}
}
void sparseicpmain(){

	SICP::Parameters sicpparam;
	Eigen::Affine3d trans=Eigen::Affine3d::Identity();
	Eigen::Matrix3Xd x(3,3);
	Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(1, Eigen::Vector3d(1, 0, 0)));
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
	double theta = M_PI / 4;  // The angle of rotation in radians
	transformation_matrix (0, 0) = cos (theta);
	transformation_matrix (0, 1) = -sin (theta);
	transformation_matrix (1, 0) = sin (theta);
	transformation_matrix (1, 1) = cos (theta);
	transformation_matrix=rx.matrix();
	//Eigen::MatrixXd a(3,3),b(3,3);
	pcl::PointCloud<pcl::PointXYZ>rabbitcloud;
	pcl::io::loadPLYFile("C:\\Users\\hcd\\Desktop\\tu_sub.ply",rabbitcloud);
	Eigen::Matrix3Xd a(3,rabbitcloud.size()),b(3,rabbitcloud.size());
	for (size_t i = 0; i < a.cols(); ++i)
	{
		 a(0,i)=rabbitcloud.at(i).x;
		 a(1,i)=rabbitcloud.at(i).y;
		 a(2,i)=rabbitcloud.at(i).z; 
		 b(0,i)=a(0,i);
		 b(1,i)=a(1,i);
		 b(2,i)=a(2,i);
	}
//	AddMatrix3xdToCloud(b);
	a=rx*a;
//	AddMatrix3xdToCloud(a);
	//AddMatrix3xdToCloud(a);
	//a=transformation_matrix*a;
	//Eigen::Matrix3Xd
	
	//MatrixXf Ham(6, 6);
	//Eigen::MatrixBase<Eigen::Vector3d>a,b;
	//Eigen::MatrixX4d a,b;
	pcl::console::TicToc time;
	time.tic();
	Eigen::Matrix4d t=SICP::point_to_point(a,b,sicpparam);
	Eigen::Affine3d tmp;
	tmp.matrix()=t;
	//for (int i=0;i<3;i++)
	//{
	//	for (int j=0;j<3;j++)
	//	{
	//		tmp(i,j)=t(i,j);
	//	}
	//}

	AddMatrix3xdToCloud(a);
	b=tmp*rx*b;
	Eigen::Vector3d aft;
	aft(0)=t(0,3);
	aft(1)=t(1,3);
	aft(2)=t(2,3);
	b.colwise()+=aft;
	AddMatrix3xdToCloud(b);
	std::cout<<time.toc()<<std::endl;
	pcl::io::savePLYFile("C:\\Users\\hcd\\Desktop\\test.ply",cloudForShow);
//	cloudviewer.showCloud(cloudForShow.makeShared());
	while (true)
	{

	}
}
///////////////////////////////////////////////////////////////////////////////
#endif