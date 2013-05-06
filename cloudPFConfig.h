#ifndef CLOUDPFCONFIG_H
#define CLOUDPFCONFIG_H

#include <particleFiltering/particleFilter.h>

class CloudPFConfig : public RCParticleFilter_Config
{
public:
	CloudPFConfig(uint32_t particles_, float varianceR_, float varianceT_, float annealingConstant_=0, uint32_t iters_=1)
	{
		particles         = particles_;
		varianceR         = varianceR_;
		varianceT         = varianceT_;
		annealingConstant = annealingConstant_;
		iters             = iters_;
	}
	float varianceR, varianceT;
	float annealingConstant;
	uint32_t iters;
};

#endif
