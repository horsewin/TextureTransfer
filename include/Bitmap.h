//
// GLUTsaveBMP
// Image/Bitmap.h
//
// The MIT License
//
// Copyright (c) 2009 sonson, sonson@Picture&Software
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////
//
//	Bitmapファイル書き込みヘッダ
//
///////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>

#include "OpenGL.h"

///////////////////////////////////////////////////////////////////////////////
//Bitmapファイル
//BitmapHeader構造体
typedef struct _BitmapHeader{
	char	distinct1;
	char	distinct2;
	int		filesize;
	short	reserve1;
	short	reserve2;
	int		offset;
}BitmapHeader;
//BitmapInfoHeader構造体
typedef struct _BitmapInfoHeader{
	int		header;
	int		width;
	int		height;
	short	plane;
	short	bits;
	int		compression;
	int		comp_image_size;
	int		x_resolution;
	int		y_resolution;
	int		pallet_num;
	int		important_pallet_num;
}BitmapInfoHeader;

///////////////////////////////////////////////////////////////////////////////
//ピクセルデータをビットマップに書き込む
int WriteBitmapFromGL(const char*, const int & viewport_x, const int & viewport_y, const int & width, const int & height);
