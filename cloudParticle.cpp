#include "config.h"

#include "cloudParticle.h"

QVec CloudParticle::varianceT;
QVec CloudParticle::varianceR;

void CloudParticle::initialize(const CloudPFInputData &data, const CloudPFControl &control, const CloudPFConfig *config)
{
	T = QVec::vec3(0,0,0);
	velT = QVec::vec3(0,0,0);

	R = QVec::vec3(0,0,0);
	velR = QVec::vec3(0,0,0);

	transformation = RTMat(0,0,0, QVec::vec3(0,0,0));
}

void CloudParticle::adapt(const CloudPFControl &controlBack, const CloudPFControl &controlNew, const bool noValidCandidates)
{
	gsl_rng_env_setup();

	QVec noiseT = QVec::vec3(getRandom(varianceT(0)), getRandom(varianceT(1)), getRandom(varianceT(2)));
// 	QVec noiseT = QVec::vec3(0,0,0);
	QVec T2 = T + noiseT;
	velT = T2 - T;
	T = T2;

	QVec noiseR = QVec::vec3(getRandom(varianceR(0)), getRandom(varianceR(1)), getRandom(varianceR(2)/10));
	QVec R2 = R + noiseR;
	velR = R2 - R;
	R = R2;

	transformation = RTMat(R(0), R(1), R(2), T);
}

void CloudParticle::computeWeight(const CloudPFInputData &data)
{
	std::vector<int> pIdxSearch(1);
	std::vector<float> pDistance(1);

	double sum = 0;
	particleError = 0;

	for (uint i=0; i<data.cloud_moves->points.size(); i++)
	{
		const QVec p_rc = transformation*QVec::vec4(data.cloud_moves->points[i].x,
		                                            data.cloud_moves->points[i].y,
		                                            data.cloud_moves->points[i].z, 1);
		const pcl::PointXYZ p_pcl(p_rc(0), p_rc(1), p_rc(2));
		if (data.kdtree->nearestKSearch(p_pcl, 1, pIdxSearch, pDistance) > 0)
		{
			const float d = pDistance[0];
// 			particleError += d*d;
			if (d > PARTICLE_DISTANCE_THRESHOLD)
				particleError+=1;

			if (d < PARTICLE_OPTIMIZATION_MAX)
			{
				if (d > PARTICLE_OPTIMIZATION_MIN)
				{
					if (DISTANCE_FUNCTION == 0)  /// BINARY
					{
						if (d > PARTICLE_DISTANCE_THRESHOLD)
							sum+=1;
					}
					else if (DISTANCE_FUNCTION == 1) /// Exponential
					{
						sum+= 1./(pow(10, (d/100.))+0.00001);
					}
					else if (DISTANCE_FUNCTION == 2) /// Simple
					{
						sum+= 1./(d+0.00001);
					}
					else if (DISTANCE_FUNCTION == 3) /// Potential
					{
						sum+= 1./((d*d)+0.00001);
					}
				}
			}
		}
	}
	this->weight = sum;
// 	sum /= data.cloud_moves->points.size();
// 	this->weight = 1./(sum+0.00000000000000000001);
}


QMat CloudParticle::getQMatTransformation()
{
	return transformation.invert();
}


Eigen::Matrix4f CloudParticle::getEigenTransformation()
{
	QMat TT = transformation.invert();
	Eigen::Matrix4f r;
	for (int col=0; col<4; ++col)
	{
		for (int row=0; row<4; ++row)
		{
			r(row, col) = TT(row, col);
		}
	}
	return r;
}



float CloudParticle::getRandom(float var)
{
	double U = double(rand())/RAND_MAX;
	double V = double(rand())/RAND_MAX;
	return sqrt(-2.*log(U))*cos(2.*M_PIl*V)*var;
}

