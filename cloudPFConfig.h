#ifndef CLOUDPFCONFIG_H
#define CLOUDPFCONFIG_H

#include <particleFiltering/particleFilter.h>
#include <QMat/QMatAll>

using namespace RMat;

class CloudPFConfig : public RCParticleFilter_Config
{
public:
	CloudPFConfig(uint32_t particles_, float vTX, float vTY, float vTZ, float vRX, float vRY, float vRZ, float annealingConstant_=0, uint32_t iters_=1)
	{
		particles         = particles_;
		varianceR         = QVec::vec3(vRX, vRY, vRZ);
		varianceT         = QVec::vec3(vTX, vTY, vTZ);
		annealingConstant = annealingConstant_;
		iters             = iters_;
	}
	QVec varianceR, varianceT;
	float annealingConstant;
	uint32_t iters;
};

#endif
