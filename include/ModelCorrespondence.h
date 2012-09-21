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
		ModelCorresPondense(): mTextureNumber(0), mModelFacetsIdx(0), mModelVertexIdx(0){};
		ModelCorresPondense(const int & lscmIdx, const int & modelTexIdx, const int & modelIdx)
		: mTextureNumber(lscmIdx), mModelFacetsIdx(modelTexIdx), mModelVertexIdx(modelIdx){};
		~ModelCorresPondense(){};

	public:
		std::vector<int> mFaceIdx;
		int mTextureNumber;
		int mModelFacetsIdx;
		int mModelVertexIdx;
	};
}

#endif /* MODELCORRESPONDENCE_H_ */
