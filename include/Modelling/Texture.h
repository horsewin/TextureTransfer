#ifndef INCLUDED__TEXTURE_H_
#define INCLUDED__TEXTURE_H_

#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <cvd/image.h>
#include <TooN/se3.h>
#include <vector>
#include <string>
#include "ImageType.h"

//namespace TooN{
//  class SE3<>;
//}

namespace TextureTransfer
{

	class Texture{
	public:
		typedef CVD::Rgb< CVD::byte > Buftype;

		explicit Texture( const CVD::SubImage< Texture::Buftype > &  src , const char * name);
		Texture( const std::vector< Buftype > & data, size_t width, size_t height );
		~Texture( void );

		const Buftype &  operator[ ]( unsigned i ) const;

		size_t  getWidth( void ) const;
		size_t  getHeight( void ) const;
		const std::vector< Buftype > & getData( void ) const;

		void  bind( void ) const;
		void  unbind( void ) const;

		void  draw( void ) const;//for debug
		int		getTex_id( void ) const{ return (int)tex_id; }

		const char * GetName(void) const { return mName;}

	private:
		unsigned  tex_width;
		unsigned  tex_height;
		std::vector< Buftype >  data;

		unsigned  tex_id;

		char mName[50];
	};

}
#endif
