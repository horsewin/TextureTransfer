/*
 * ModelCorrespondence.h
 *
 *  Created on: 2012/09/20
 *      Author: umakatsu
 */

#ifndef MODELCORRESPONDENCE_H_
#define MODELCORRESPONDENCE_H_

#include <vector>

namespace TextureTransfer
{
	class ModelCorresPondense
	{
	public:
		ModelCorresPondense(): mTextureNumber(0), mModelFaceIdx(0){};
		ModelCorresPondense(const int & lscmIdx, const int & modelFaceIdx)
		: mTextureNumber(lscmIdx), mModelFaceIdx(modelFaceIdx){};
		~ModelCorresPondense(){};

	public:
		std::vector<int> mVertexIdxArray;
		int mTextureNumber;
		int mModelFaceIdx;
	};
}

#endif /* MODELCORRESPONDENCE_H_ */
